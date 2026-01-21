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
/*
 * RP2350 I2C device implementation using the Raspberry Pi Pico SDK.
 */

#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

using namespace RP;

extern const AP_HAL::HAL& hal;

// I2C bus configuration
// RP2350 has i2c0 and i2c1
// These will be configured from hwdef.dat in the future
static const I2CBusDesc i2c_bus_descs[] = {
    {i2c0, 4, 5, 400000, false},  // i2c0: SDA=GPIO4, SCL=GPIO5, 400kHz, external
    {i2c1, 6, 7, 400000, false},  // i2c1: SDA=GPIO6, SCL=GPIO7, 400kHz, external
};

/*
 * I2CBus implementation
 */
I2CBus::I2CBus(uint8_t bus_num) :
    semaphore(),
    bus_num(bus_num),
    bus_clock(400000),
    initialized(false),
    _instance(nullptr),
    _sda(0),
    _scl(0)
{
}

I2CBus::~I2CBus()
{
    // I2C deinit is handled by Pico SDK automatically
}

void I2CBus::init()
{
    if (initialized) {
        return;
    }

    if (bus_num >= ARRAY_SIZE(i2c_bus_descs)) {
        return;
    }

    const I2CBusDesc &desc = i2c_bus_descs[bus_num];
    _instance = desc.instance;
    _sda = desc.sda;
    _scl = desc.scl;
    bus_clock = desc.speed;

    // Initialize I2C at the specified speed
    i2c_init(_instance, bus_clock);

    // Set GPIO functions for SDA and SCL
    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_set_function(_scl, GPIO_FUNC_I2C);

    // Enable pull-ups on SDA and SCL
    gpio_pull_up(_sda);
    gpio_pull_up(_scl);

    initialized = true;
}

bool I2CBus::transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                      uint8_t *recv, uint32_t recv_len)
{
    if (!initialized || !_instance) {
        return false;
    }

    int ret;

    if (send_len > 0 && recv_len > 0) {
        // Write then read
        ret = i2c_write_blocking(_instance, address, send, send_len, false);
        if (ret < 0) {
            return false;
        }
        ret = i2c_read_blocking(_instance, address, recv, recv_len, false);
    } else if (send_len > 0) {
        // Write only
        ret = i2c_write_blocking(_instance, address, send, send_len, true);
    } else if (recv_len > 0) {
        // Read only
        ret = i2c_read_blocking(_instance, address, recv, recv_len, false);
    } else {
        // Nothing to do
        return true;
    }

    return ret >= 0;
}

/*
 * I2CDevice implementation
 */
I2CDevice::I2CDevice(I2CBus &bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    _bus(bus),
    _address(address),
    _retries(0),
    _split_transfers(false),
    _use_smbus(use_smbus),
    _timeout_ms(timeout_ms)
{
    set_device_bus(bus.bus_num);
    set_device_address(address);
}

I2CDevice::~I2CDevice()
{
}

bool I2CDevice::_transfer(const uint8_t *send, uint32_t send_len,
                           uint8_t *recv, uint32_t recv_len)
{
    if (!_bus.initialized) {
        return false;
    }

    // Retry logic
    for (uint8_t i = 0; i <= _retries; i++) {
        if (_bus.transfer(_address, send, send_len, recv, recv_len)) {
            return true;
        }
        // Small delay between retries
        if (i < _retries) {
            hal.scheduler->delay_microseconds(100);
        }
    }

    return false;
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len)
{
    if (_split_transfers && send_len > 0 && recv_len > 0) {
        // Split transfer: write then read with stop condition between
        return _transfer(send, send_len, nullptr, 0) &&
               _transfer(nullptr, 0, recv, recv_len);
    }

    return _transfer(send, send_len, recv, recv_len);
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    if (!_bus.initialized) {
        return false;
    }

    // Read the same register multiple times
    for (uint8_t i = 0; i < times; i++) {
        if (!_transfer(&first_reg, 1, recv + (i * recv_len), recv_len)) {
            return false;
        }
    }

    return true;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &_bus.semaphore;
}

AP_HAL::Device::PeriodicHandle
I2CDevice::register_periodic_callback(uint32_t period_usec,
                                      AP_HAL::Device::PeriodicCb)
{
    // No dedicated device-thread support yet on RP HAL.
    return nullptr;
}

bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                          uint32_t period_usec)
{
    // Not implemented yet
    return false;
}

/*
 * I2CDeviceManager implementation
 */
I2CDeviceManager::I2CDeviceManager() :
    _num_buses(0),
    _bus_mask(0),
    _bus_mask_external(0),
    _bus_mask_internal(0)
{
    // Initialize all buses
    for (uint8_t i = 0; i < ARRAY_SIZE(i2c_bus_descs) && i < MAX_I2C_BUSES; i++) {
        _buses[i] = NEW_NOTHROW I2CBus(i);
        if (_buses[i]) {
            _buses[i]->init();
            _num_buses++;
            _bus_mask |= (1U << i);
            if (i2c_bus_descs[i].internal) {
                _bus_mask_internal |= (1U << i);
            } else {
                _bus_mask_external |= (1U << i);
            }
        }
    }
}

I2CDeviceManager::~I2CDeviceManager()
{
    for (uint8_t i = 0; i < _num_buses; i++) {
        delete _buses[i];
    }
}

AP_HAL::I2CDevice *
I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                  uint32_t bus_clock,
                                  bool use_smbus,
                                  uint32_t timeout_ms)
{
    if (bus >= _num_buses || !_buses[bus]) {
        return nullptr;
    }

    return NEW_NOTHROW I2CDevice(*_buses[bus], address, bus_clock, use_smbus, timeout_ms);
}

uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return _bus_mask;
}

uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return _bus_mask_external;
}

uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    return _bus_mask_internal;
}

