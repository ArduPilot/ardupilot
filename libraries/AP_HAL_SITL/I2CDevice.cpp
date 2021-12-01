/*
 * Copyright (C) 2020  Peter Barker. All rights reserved.
 *
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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

/*
 * I2CBus
 */
class I2CBus {
    friend class I2CDeviceManager;
public:

    I2CBus() { bus = i2c_buscount++; }

    uint8_t bus;
    Semaphore sem;
    int ioctl(uint8_t ioctl_number, void *data) {
        return _ioctl(ioctl_number, data);
    }
    void _timer_tick(); // in lieu of a thread-per-bus
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb);

private:
    int _ioctl(uint8_t ioctl_number, void *data);

    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    } *callbacks;

    static uint8_t i2c_buscount;

};

uint8_t I2CBus::i2c_buscount;

int I2CBus::_ioctl(uint8_t ioctl_number, void *data)
{
    SITL::SIM *sitl = AP::sitl();
    return sitl->i2c_ioctl(ioctl_number, data);
}

AP_HAL::Device::PeriodicHandle I2CBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    // mostly swiped from ChibiOS:
    I2CBus::callback_info *callback = new I2CBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;
    return callback;
}

void I2CBus::_timer_tick()
{
    const uint64_t now = AP_HAL::micros64();
    for (struct callback_info *ci = callbacks; ci != nullptr; ci = ci->next) {
        if (ci->next_usec < now) {
            WITH_SEMAPHORE(sem);
            ci->cb();
            ci->next_usec += ci->period_usec;
        }
    }
}

/*
 * I2CDeviceManager
 */

I2CBus I2CDeviceManager::buses[NUM_SITL_I2C_BUSES] {};

I2CDeviceManager::I2CDeviceManager()
{
    for (uint8_t i=0; i<ARRAY_SIZE(buses); i++) {
        buses[i].bus = i;
    }
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus,
                             uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(buses)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(buses[bus], address));
    return dev;
}

void I2CDeviceManager::_timer_tick()
{
    for (auto &bus : buses) {
        bus._timer_tick();
    }
}

/*
 * I2CDevice
 */

I2CDevice::I2CDevice(I2CBus &bus, uint8_t address)
    : _bus(bus)
    , _address(address)
{
    // ::fprintf(stderr, "bus.bus=%u address=0x%02x\n", bus.bus, address);
    set_device_bus(bus.bus);
    set_device_address(address);
}

#include <stdio.h>

#include <signal.h>

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    _bus.sem.check_owner();

    // combined transfer
    return _transfer(send, send_len, recv, recv_len);
}

bool I2CDevice::_transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len)
{
    struct i2c_msg msgs[2] = { };
    unsigned nmsgs = 0;

    if (send && send_len != 0) {
        msgs[nmsgs].bus = _bus.bus;
        msgs[nmsgs].addr = _address;
        msgs[nmsgs].flags = 0;
        msgs[nmsgs].buf = const_cast<uint8_t*>(send);
        msgs[nmsgs].len = send_len;
        nmsgs++;
    }

    if (recv && recv_len != 0) {
        msgs[nmsgs].bus = _bus.bus;
        msgs[nmsgs].addr = _address;
        msgs[nmsgs].flags = I2C_M_RD;
        msgs[nmsgs].buf = recv;
        msgs[nmsgs].len = recv_len;
        nmsgs++;
    }

    /* interpret it as an input error if nothing has to be done */
    if (!nmsgs) {
        return false;
    }

    struct i2c_rdwr_ioctl_data i2c_data = { };

    i2c_data.msgs = msgs;
    i2c_data.nmsgs = nmsgs;

    int r;
    unsigned retries = _retries;
    do {
        r = _bus.ioctl(I2C_RDWR, &i2c_data);
    } while (r == -1 && retries-- > 0);

    return r != -1;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    ::fprintf(stderr, "read_registers_multiple called\n");
    return false;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore() {
    return &_bus.sem;
}

AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus.register_periodic_callback(period_usec, cb);
}

bool I2CDevice::adjust_periodic_callback(Device::PeriodicHandle h, uint32_t period_usec)
{
    return false;
}

#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
