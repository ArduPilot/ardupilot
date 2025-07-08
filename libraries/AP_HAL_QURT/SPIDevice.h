/*
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
#include "AP_HAL_QURT.h"

#include "Semaphores.h"
#include "Scheduler.h"
#include "DeviceBus.h"

namespace QURT
{

class SPIBus : public DeviceBus
{
public:
    SPIBus();
    int fd = -1;
};

class SPIDevice : public AP_HAL::SPIDevice
{
public:
    SPIDevice(const char *name, SPIBus &bus);
    virtual ~SPIDevice();

    bool set_speed(AP_HAL::Device::Speed speed) override;
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len) override;
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

private:
    SPIBus &bus;
    const char *pname;
    void acquire_bus(bool accuire);
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    friend class SPIDevice;

    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;
};
}

