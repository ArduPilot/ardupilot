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
 * RP2350 SPI device implementation using the Raspberry Pi Pico SDK.
 *
 * The SPI bus and device configuration comes from the board hwdef.dat
 * via the generated HAL_RP2350_SPI_BUSES and HAL_RP2350_SPI_DEVICES
 * tables (see hwdef/scripts/rp_hwdef.py).
 */

#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "Semaphores.h"

// Pico SDK
#include "hardware/spi.h"
#include "hardware/gpio.h"

// helpers for hwdef speed macros (e.g. 2*MHZ, 400*KHZ)
#ifndef MHZ
#define MHZ (1000U*1000U)
#endif
#ifndef KHZ
#define KHZ (1000U)
#endif

namespace RP {

struct SPIBusDesc {
    spi_inst_t *host;
    int         dma_ch_tx;  // DMA channel for TX (transmit)
    int         dma_ch_rx;  // DMA channel for RX (receive)
    uint        mosi;
    uint        miso;
    uint        sclk;
};

struct SPIDeviceDesc {
    const char *name;
    uint8_t     bus;
    uint8_t     device;
    uint        cs;
    uint16_t    mode;
    uint32_t    lspeed;
    uint32_t    hspeed;
};

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(const SPIDeviceDesc &desc);
    ~SPIDevice() override = default;

    /* AP_HAL::Device implementation */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;

    // uses base-class optimisation for in-place transfers

    AP_HAL::Semaphore *get_semaphore() override {
        return &_semaphore;
    }

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

    /* See AP_HAL::SPIDevice::clock_pulse() */
    bool clock_pulse(uint32_t len) override;

private:
    bool do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len);
    void configure_bus();

    SPIDeviceDesc           _desc;
    AP_HAL::Device::Speed   _speed;
    uint32_t                _current_baud;
    RP::Semaphore           _semaphore;
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    SPIDeviceManager() = default;

    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;
};

} // namespace RP
