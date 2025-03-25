/*
 * Copyright (C) 2021 Peter Barker. All rights reserved.
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

class SPIBus;

namespace HALSITL {

struct SPIDesc {
    SPIDesc(const char *_name, uint8_t _bus, uint8_t _cs_pin)
        : name(_name), bus(_bus), cs_pin(_cs_pin)
    { }

    const char *name;
    uint8_t bus;
    uint8_t cs_pin;  // cs
};

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &_bus, SPIDesc &_device_desc);

    // no concept of speed in SITL yet
    bool set_speed(AP_HAL::Device::Speed speed) override { return true; }

    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override {
        abort();
    }

    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override {
        abort();
    }

    AP_HAL::Semaphore *get_semaphore() override;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

private:
    SPIBus &bus;
    SPIDesc &device_desc;

    Semaphore _semaphore;

    bool do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len);
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:

    SPIDeviceManager();

    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;

    static SPIDesc device_table[];
    static SPIBus *buses;
};

}
