/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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

#include "SPIDriver.h"

namespace Linux {

class SPIBus;

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &bus, SPIDeviceDriver &device_desc);

    virtual ~SPIDevice();

    /* AP_HAL::SPIDevice implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override;

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle *register_periodic_callback(
        uint32_t period_usec, AP_HAL::MemberProc) override
    {
        return nullptr;
    }

    /* See AP_HAL::Device::get_fd() */
    int get_fd() override;

protected:
    SPIBus &_bus;
    SPIDeviceDriver &_desc;

    /*
     * Select device if using userspace CS
     */
    void _cs_assert();

    /*
     * Deselect device if using userspace CS
     */
    void _cs_release();
};

}
