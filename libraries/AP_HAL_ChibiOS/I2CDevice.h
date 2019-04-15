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
 * Modified for use in AP_HAL_ChibiOS by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "AP_HAL_ChibiOS.h"

#if HAL_USE_I2C == TRUE

#include "Semaphores.h"
#include "Device.h"
#include "shared_dma.h"

namespace ChibiOS {

class I2CBus : public DeviceBus {
public:
    I2CConfig i2ccfg;
    uint8_t busnum;
    uint32_t busclock;

    // we need an additional lock in the dma_allocate and
    // dma_deallocate functions to cope with 3-way contention as we
    // have two DMA channels that we are handling with the shared_dma
    // code
    mutex_t dma_lock;
    
    void dma_allocate(Shared_DMA *);
    void dma_deallocate(Shared_DMA *);
    void dma_init(void);
    static void clear_all(void);
    static void clear_bus(uint8_t busidx);
    static uint8_t read_sda(uint8_t busidx);
};
    
class I2CDevice : public AP_HAL::I2CDevice {
public:
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
        return static_cast<I2CDevice*>(dev);
    }

    I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms);
    ~I2CDevice();

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override { _address = address; }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override { _retries = retries; }

    /* See AP_HAL::Device::set_speed(): Empty implementation, not supported. */
    bool set_speed(enum Device::Speed speed) override { return true; }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override;

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See AP_HAL::Device::adjust_periodic_callback() */
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    AP_HAL::Semaphore* get_semaphore() override {
        // if asking for invalid bus number use bus 0 semaphore
        return &bus.semaphore;
    }

    void set_split_transfers(bool set) override {
        _split_transfers = set;
    }
    
private:
    I2CBus &bus;
    bool _transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);

    /* I2C interface #2 */
    uint8_t _retries;
    uint8_t _address;
    char *pname;
    bool _split_transfers;
    bool _use_smbus;
    uint32_t _timeout_ms;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    static I2CBus businfo[];
    
    // constructor
    I2CDeviceManager();
    
    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
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
};
}

#endif // HAL_USE_I2C

