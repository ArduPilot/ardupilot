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
 *
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/I2CDevice.h>

#include "DeviceBus.h"
#include "Semaphores.h"

#ifdef __ZEPHYR__
#include <zephyr/device.h>
#endif

namespace Zephyr {

class I2CDevice : public AP_HAL::I2CDevice {
public:
    I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock, uint32_t timeout_ms);

    void set_address(uint8_t address) override
    {
        _address = address;
        set_device_address(address);
    }

    void set_retries(uint8_t retries) override
    {
        _retries = retries;
    }

    bool set_speed(AP_HAL::Device::Speed speed) override;

    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override;

    AP_HAL::Semaphore *get_semaphore() override
    {
        // bus semaphore is shared between devices on the same bus,
        // matching AP_HAL_ChibiOS
        return _bus_handle != nullptr ? &_bus_handle->semaphore : &_semaphore;
    }

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                  uint32_t period_usec) override;

    void set_split_transfers(bool set) override
    {
        _split_transfers = set;
    }

private:
#ifdef __ZEPHYR__
    const struct device *_dev = nullptr;
#endif

    DeviceBus *_bus_handle = nullptr;
    uint8_t _bus;
    uint8_t _address;
    uint8_t _retries = 2;
    uint32_t _bus_clock;
    uint32_t _timeout_ms;
    bool _split_transfers = false;
    Semaphore _semaphore;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
                                      uint32_t bus_clock = 400000,   // I2C fast-mode
                                      bool use_smbus = false,
                                      uint32_t timeout_ms = 4) override;

    uint32_t get_bus_mask(void) const override;
    uint32_t get_bus_mask_external(void) const override;
    uint32_t get_bus_mask_internal(void) const override;
};

}  // namespace Zephyr
