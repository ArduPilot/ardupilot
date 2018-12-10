/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
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
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

namespace Empty {

class I2CDevice : public AP_HAL::I2CDevice {
public:
    I2CDevice()
    {
    }

    virtual ~I2CDevice() { }

    /* AP_HAL::I2CDevice implementation */

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override { }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override { }


    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override
    {
        return true;
    }

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override
    {
        return true;
    }


    /* See AP_HAL::Device::set_speed() */
    bool set_speed(enum AP_HAL::Device::Speed speed) override { return true; }

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override { return nullptr; }

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override
    {
        return nullptr;
    }

    /* See Device::adjust_periodic_callback() */
    virtual bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override
    {
        return true;
    }
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    I2CDeviceManager() { }

    /* AP_HAL::I2CDeviceManager implementation */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
                                                 uint32_t bus_clock=400000,
                                                 bool use_smbus = false,
                                                 uint32_t timeout_ms=4) override
    {
        return nullptr;
    }
};

}
