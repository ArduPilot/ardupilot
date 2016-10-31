/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>

#include "Util.h"
#include "Scheduler.h"

extern bool _px4_thread_should_exit;

namespace PX4 {

uint8_t PX4::PX4_I2C::instance;

/*
  constructor for I2C wrapper class
 */    
PX4_I2C::PX4_I2C(uint8_t bus) :
  I2C(devname, devpath, map_bus_number(bus), 0, 400000UL)
{}

/*
  map ArduPilot bus numbers to PX4 bus numbers
 */    
uint8_t PX4_I2C::map_bus_number(uint8_t bus) const
{
    switch (bus) {
    case 0:
        // map to internal bus
#ifdef PX4_I2C_BUS_ONBOARD
        return PX4_I2C_BUS_ONBOARD;
#else
        return 0;
#endif

    case 1:
        // map to expansion bus
#ifdef PX4_I2C_BUS_EXPANSION
        return PX4_I2C_BUS_EXPANSION;
#else
        return 1;
#endif
        
    }
    // default to bus 1
    return 1;
}
    
/*
  implement wrapper for PX4 I2C driver
 */
bool PX4_I2C::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    set_address(address);
    if (!init_done) {
        init_done = true;
        // we do late init() so we can setup the device paths
        snprintf(devname, sizeof(devname), "AP_I2C_%u", instance);
        snprintf(devpath, sizeof(devpath), "/dev/api2c%u", instance);
        init_ok = (init() == OK);
        if (init_ok) {
            instance++;
        }
    }
    if (!init_ok) {
        return false;
    }
    bool ret = (transfer(send, send_len, recv, recv_len) == OK);
    return ret;
}

PX4::I2CDevice::bus_info PX4::I2CDevice::businfo[num_buses];
    
I2CDevice::I2CDevice(uint8_t bus, uint8_t address) :
    _busnum(bus),
    _px4dev(_busnum),
    _address(address)
{
}
    
I2CDevice::~I2CDevice()
{
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    return _px4dev.do_transfer(_address, send, send_len, recv, recv_len);
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    return false;
}

/*
  per-bus i2c callback thread
*/
void *I2CDevice::i2c_thread(void *arg)
{
    struct bus_info *binfo = (struct bus_info *)arg;
    while (!_px4_thread_should_exit) {
        uint64_t now = AP_HAL::micros64();
        uint64_t next_needed = 0;
        struct callback_info *callback;

        // find a callback to run
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with semaphore held
                if (binfo->semaphore.take(0)) {
                    callback->cb();
                    binfo->semaphore.give();
                }
            }
            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
            }
        }

        // delay for at most 50ms, to handle newly added callbacks
        now = AP_HAL::micros64();
        uint32_t delay = 50000;
        if (next_needed > now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        hal.scheduler->delay_microseconds(delay);
    }
    return nullptr;
}
    

/*
  find or create thread for this bus
*/
I2CDevice::bus_info *I2CDevice::find_thread(void)
{
    if (_busnum >= num_buses) {
        return nullptr;
    }
    struct bus_info &binfo = businfo[_busnum];
    if (binfo.thread_started) {
        return &binfo;
    }

    binfo.bus = _busnum;
    binfo.thread_started = true;
    
    pthread_attr_t thread_attr;
    struct sched_param param;
    
    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 1024);
    
    param.sched_priority = APM_I2C_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);
    pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);
    
    pthread_create(&binfo.thread_ctx, &thread_attr, &I2CDevice::i2c_thread, &binfo);

    return &binfo;
}
    
/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    struct bus_info *binfo = find_thread();
    if (binfo == nullptr) {
        return nullptr;
    }
    struct callback_info *callback = new callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = binfo->callbacks;
    binfo->callbacks = callback;

    return callback;
}
    

/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    struct bus_info *binfo = find_thread();
    if (binfo == nullptr) {
        return nullptr;
    }

    for (struct callback_info *callback = binfo->callbacks; callback; callback = callback->next) {
        if (h == (AP_HAL::Device::PeriodicHandle)callback) {
            callback->period_usec = period_usec;
            return true;
        }
    }
    return false;
}
    
AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address));
    return dev;
}

}
