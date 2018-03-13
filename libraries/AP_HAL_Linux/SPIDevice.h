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
#include <vector>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

namespace Linux {

class SPIBus;
class SPIDesc;

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &bus, SPIDesc &device_desc);

    virtual ~SPIDevice();

    /* AP_HAL::SPIDevice implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /* See AP_HAL::SPIDevice::transfer_fullduplex() */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override;

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See AP_HAL::Device::adjust_periodic_callback() */
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

protected:
    SPIBus &_bus;
    SPIDesc &_desc;
    AP_HAL::DigitalSource *_cs;
    uint32_t _speed;

    /*
     * Select device if using userspace CS
     */
    void _cs_assert();

    /*
     * Deselect device if using userspace CS
     */
    void _cs_release();
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;

    static SPIDeviceManager *from(AP_HAL::SPIDeviceManager *spi_mgr)
    {
        return static_cast<SPIDeviceManager*>(spi_mgr);
    }

    SPIDeviceManager()
    {
        /* Reserve space up-front for 3 buses */
        _buses.reserve(3);
    }

    /* AP_HAL::SPIDeviceManager implementation */
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name);

    /*
     * Stop all SPI threads and block until they are finalized. This doesn't
     * free memory because they can still be used by devices, however device
     * drivers won't receive any new event
     */
    void teardown();

    /* See AP_HAL::SPIDeviceManager::get_count() */
    uint8_t get_count();

    /* See AP_HAL::SPIDeviceManager::get_device_name() */
    const char *get_device_name(uint8_t idx);

protected:
    void _unregister(SPIBus &b);
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _create_device(SPIBus &b, SPIDesc &device_desc) const;

    std::vector<SPIBus*> _buses;

    static const uint8_t _n_device_desc;
    static SPIDesc _device[];
};

}
