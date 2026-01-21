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

#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "Semaphores.h"
#include "Scheduler.h"

// Pico SDK
#include "hardware/i2c.h"
#include "hardware/gpio.h"

namespace RP {

struct I2CBusDesc {
    i2c_inst_t *instance;
    uint        sda;
    uint        scl;
    uint32_t    speed;
    bool        internal;
};

class I2CBus {
public:
    I2CBus(uint8_t bus_num);
    ~I2CBus();

    void init();
    bool transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len);

    RP::Semaphore semaphore;
    uint8_t bus_num;
    uint32_t bus_clock;
    bool initialized;

private:
    i2c_inst_t *_instance;
    uint _sda;
    uint _scl;
};

class I2CDevice : public AP_HAL::I2CDevice {
public:
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
        return static_cast<I2CDevice*>(dev);
    }

    I2CDevice(I2CBus &bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms);
    ~I2CDevice();

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override { _address = address; }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override { _retries = retries; }

    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(enum Device::Speed speed) override { return true; }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override;

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See AP_HAL::Device::adjust_periodic_callback() */
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    /* set split transfers flag */
    void set_split_transfers(bool set) override {
        _split_transfers = set;
    }

private:
    bool _transfer(const uint8_t *send, uint32_t send_len,
                   uint8_t *recv, uint32_t recv_len);

    I2CBus &_bus;
    uint8_t _address;
    uint8_t _retries;
    bool _split_transfers;
    bool _use_smbus;
    uint32_t _timeout_ms;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    I2CDeviceManager();
    ~I2CDeviceManager();

    /* AP_HAL::I2CDeviceManager implementation */
    AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
                                      uint32_t bus_clock=400000,
                                      bool use_smbus = false,
                                      uint32_t timeout_ms=4) override;

    /*
      get mask of bus numbers for all configured I2C buses
     */
    uint32_t get_bus_mask(void) const override;

    /*
      get mask of bus numbers for all configured external I2C buses
     */
    uint32_t get_bus_mask_external(void) const override;

    /*
      get mask of bus numbers for all configured internal I2C buses
     */
    uint32_t get_bus_mask_internal(void) const override;

private:
    static const uint8_t MAX_I2C_BUSES = 2;  // RP2350 has i2c0 and i2c1
    I2CBus *_buses[MAX_I2C_BUSES];
    uint8_t _num_buses;
    uint32_t _bus_mask;
    uint32_t _bus_mask_external;
    uint32_t _bus_mask_internal;
};

} // namespace RP
