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

I2CDevice::I2CDevice(uint8_t bus, uint8_t address) :
    _busnum(bus),
    _px4dev(_busnum),
    _address(address)
{
    set_device_bus(_px4dev.map_bus_number(bus));
    set_device_address(address);
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
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    if (_busnum >= num_buses) {
        return nullptr;
    }
    struct DeviceBus &binfo = businfo[_busnum];
    return binfo.register_periodic_callback(period_usec, cb);
}
    

/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return false;
}
    
AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address));
    return dev;
}

}
