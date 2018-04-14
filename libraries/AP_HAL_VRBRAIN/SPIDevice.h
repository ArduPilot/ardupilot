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
#include "Device.h"
#include "Scheduler.h"

namespace VRBRAIN {

class SPIDesc;
    
class SPIBus : public DeviceBus {
public:
    SPIBus(void) :
        DeviceBus(APM_SPI_PRIORITY) {}
    struct spi_dev_s *dev;
    uint8_t bus;
};

struct SPIDesc {
    SPIDesc(const char *_name, uint8_t _bus, 
            enum spi_dev_e _device, enum spi_mode_e _mode,
            uint32_t _lowspeed, uint32_t _highspeed)
        : name(_name), bus(_bus), device(_device), mode(_mode),
          lowspeed(_lowspeed), highspeed(_highspeed)
    {
    }

    const char *name;
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

    // low level transfer function
    void do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len);
    
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

    bool set_chip_select(bool set) override;
    
private:
    SPIBus &bus;
    SPIDesc &device_desc;
    uint32_t frequency;
    perf_counter_t perf;
    char *pname;
    bool cs_forced;
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
    SPIBus *buses;
};

}
