/*
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <drivers/device/spi.h>
#include "Semaphores.h"
#include "SPIWrapper.h"

namespace PX4 {

class SPIDesc;

struct SPIBus {
    struct SPIBus *next;
    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;        
    } *callbacks;
    Semaphore semaphore;
    pthread_t thread_ctx;
    bool thread_started;
    uint8_t bus;
};

struct SPIDesc {
    SPIDesc(const char *_name, uint8_t _bus, 
            enum spi_dev_e _device, enum spi_mode_e _mode,
            uint32_t _lowspeed, uint32_t _highspeed)
        : name(_name), bus(_bus), device(_device), mode(_mode),
          lowspeed(_lowspeed), highspeed(_highspeed)
    {
        snprintf(devname, sizeof(devname), "/dev/%s", name);
    }

    const char *name;
    char devname[20];
    uint8_t bus;
    enum spi_dev_e device;
    enum spi_mode_e mode;
    uint32_t lowspeed;
    uint32_t highspeed;
};
    
    
class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &_bus, SPIDesc &_device_desc);

    virtual ~SPIDevice();

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
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

private:
    SPIBus &bus;
    SPIDesc &device_desc;
    PX4_SPI px4dev;

    static void *spi_thread(void *arg);
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;

    static SPIDeviceManager *from(AP_HAL::SPIDeviceManager *spi_mgr)
    {
        return static_cast<SPIDeviceManager*>(spi_mgr);
    }

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name);

private:
    static SPIDesc device_table[];
    struct SPIBus *buses;
};

}
