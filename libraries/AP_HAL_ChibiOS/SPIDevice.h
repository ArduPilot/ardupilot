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
#include "Semaphores.h"
#include "Scheduler.h"
#include "Device.h"

#define SPIDEV_BMP280           0
#define SPIDEV_LSM303D          1
#define SPIDEV_L3GD20H          2
#define SPIDEV_MS5611           3
#define SPIDEV_EXT_MS5611       4
#define SPIDEV_MPU              5
#define SPIDEV_EXT_MPU          6
#define SPIDEV_EXT_LSM9DS0_G    7
#define SPIDEV_EXT_LSM9DS0_AM   8
#define SPIDEV_CYRF             9
#define SPIDEV_FLOW             10
#define SPIDEV_RAMTROM          11

#define SPIDEV_MODE0    0
#define SPIDEV_MODE1    SPI_CR1_CPHA
#define SPIDEV_MODE2    SPI_CR1_CPOL
#define SPIDEV_MODE3    SPI_CR1_CPOL | SPI_CR1_CPHA


namespace ChibiOS {

class SPIDesc;

class SPIBus : public DeviceBus {
public:
    SPIBus(uint8_t bus);
    struct spi_dev_s *dev;
    uint8_t bus;
    SPIConfig spicfg;
    void dma_allocate(void);
    void dma_deallocate(void);    
};

struct SPIDesc {
    SPIDesc(const char *_name, uint8_t _bus,
            uint8_t _device, ioportid_t _port, uint8_t _pin,
            uint16_t _mode, uint32_t _lowspeed, uint32_t _highspeed)
        : name(_name), bus(_bus), device(_device),
          port(_port), pin(_pin), mode(_mode),
          lowspeed(_lowspeed), highspeed(_highspeed)
    {
    }

    const char *name;
    uint8_t bus;
    uint8_t device;
    ioportid_t port;
    uint8_t pin;
    uint16_t mode;
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
    uint16_t freq_flag;
    uint16_t freq_flag_low;
    uint16_t freq_flag_high;
    char *pname;
    bool cs_forced;
    static void *spi_thread(void *arg);
    uint16_t derive_freq_flag(uint32_t _frequency);
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
