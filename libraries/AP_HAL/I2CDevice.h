/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

class I2CDevice : public Device {
public:
    virtual ~I2CDevice() { }

    /*
     * Change device address. Note that this is the 7 bit address, it
     * does not include the bit for read/write.
     */
    virtual void set_address(uint8_t address) = 0;

    /* set number of retries on transfers */
    virtual void set_retries(uint8_t retries) = 0;

    /* Device implementation */

    /* See Device::set_speed() */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /* See Device::transfer() */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /*
     * Read location from device multiple times, advancing the buffer each
     * time
     */
    virtual bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                         uint32_t recv_len, uint8_t times) = 0;

    /* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;

    /* See Device::register_periodic_callback() */
    virtual Device::PeriodicHandle *register_periodic_callback(
        uint32_t period_usec, MemberProc) override = 0;

    /* See Device::get_fd() */
    virtual int get_fd() override = 0;
};

class I2CDeviceManager {
public:
    /* Get a device handle */
    virtual OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) = 0;
};

}
