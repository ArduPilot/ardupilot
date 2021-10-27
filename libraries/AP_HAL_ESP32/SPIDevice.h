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
#include "AP_HAL_ESP32.h"

#include "Semaphores.h"
#include "Scheduler.h"
#include "DeviceBus.h"

#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"


namespace ESP32
{

struct SPIDeviceDesc {
    const char *name;
    uint8_t bus;
    uint8_t device;
    gpio_num_t cs;
    uint16_t mode;
    uint32_t lspeed;
    uint32_t hspeed;
};

struct SPIBusDesc {
    spi_host_device_t host;
    int dma_ch;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
};

class SPIBus : public DeviceBus
{
public:
    SPIBus(uint8_t _bus);
    uint8_t bus;
};

class SPIDevice : public AP_HAL::SPIDevice
{
public:
    SPIDevice(SPIBus &_bus, SPIDeviceDesc &_device_desc);
    virtual ~SPIDevice();

    bool set_speed(AP_HAL::Device::Speed speed) override;
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

private:
    SPIBus &bus;
    SPIDeviceDesc &device_desc;
    Speed speed;
    char *pname;
    spi_device_handle_t low_speed_dev_handle;
    spi_device_handle_t high_speed_dev_handle;
    spi_device_handle_t current_handle();
    void acquire_bus(bool accuire);
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    friend class SPIDevice;

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) override;

private:
    SPIBus *buses;
};
}

