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
#include <AP_HAL/SPIDevice.h>

#include "Semaphores.h"

namespace Empty {

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice()
    {
    }

    virtual ~SPIDevice() { }

    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(AP_HAL::Device::Speed speed) override
    {
        return true;
    }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override
    {
        return true;
    }

    /* See AP_HAL::SPIDevice::transfer_fullduplex() */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override
    {
        return true;
    }

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override
    {
        return &_semaphore;
    }

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override
    {
        return nullptr;
    }

private:
    Semaphore _semaphore;
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    SPIDeviceManager() { }
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) override
    {
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice());
    }
};

}
