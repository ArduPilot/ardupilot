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

class SPIDevice : public Device {
public:
    SPIDevice() : Device(BUS_TYPE_SPI) { }

    virtual ~SPIDevice() { }
    /* Device implementation */

    /* See Device::set_speed() */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /* See Device::transfer() */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /*
     * Like #transfer(), but both @send and @recv buffers are transmitted at
     * the same time: because of this they need to be of the same size.
     */
    virtual bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                     uint32_t len) = 0;

    /*
     * Like #transfer(), but len bytes are both transmitted and received from/into @send_recv.
     * This function is optimized for DMA-enabled multi-transfers without buffer copying
     */
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override {
        return transfer(send_recv, len, send_recv, len);
    }

    /* 
     *  send N bytes of clock pulses without taking CS. This is used
     *  when initialising microSD interfaces over SPI
    */
    virtual bool clock_pulse(uint32_t len) { return false; }
    
    /* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;

    /* See Device::register_periodic_callback() */
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    /* See Device::adjust_periodic_callback() */
    virtual bool adjust_periodic_callback(
        PeriodicHandle h, uint32_t period_usec) override { return false; }

    // setup a bus clock slowdown factor (optional interface)
    virtual void set_slowdown(uint8_t slowdown) {}
};

class SPIDeviceManager {
public:
    virtual SPIDevice *get_device_ptr(const char *name) = 0;
    OwnPtr<SPIDevice> get_device(const char *name) {
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(get_device_ptr(name));
    }

    /* Return the number of SPI devices currently registered. */
    virtual uint8_t get_count() { return 0; }

    /* Get spi device name at @idx */
    virtual const char *get_device_name(uint8_t idx) { return nullptr; }

    virtual void set_register_rw_callback(const char* name, AP_HAL::Device::RegisterRWCb cb) {}
};

}
