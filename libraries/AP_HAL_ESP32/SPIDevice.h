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


namespace ESP32 {

struct SPIDesc {
    const char *name;
    uint8_t device;
    uint8_t bus;
    uint8_t cs_gpio;
    uint16_t mode;
    uint32_t lowspeed;
    uint32_t highspeed;
};

class SPIBus : public DeviceBus {
public:
    SPIBus(uint8_t _bus);
    uint8_t bus;
};

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &_bus, SPIDesc &_device_desc);
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
    bool set_chip_select(bool set) override;

private:
    SPIBus &bus;
    SPIDesc &device_desc;
    Speed speed;
    char *pname;
    bool cs_forced;
    spi_device_handle_t low_speed_dev_handle;
    spi_device_handle_t high_speed_dev_handle;
    spi_device_handle_t current_handle();
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) override;

private:
    static SPIDesc device_table[];
    SPIBus *buses;
};
}

