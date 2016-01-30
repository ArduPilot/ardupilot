#pragma once

#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_HAL_Empty.h"
#include "Semaphores.h"

class Empty::SPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    SPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    Semaphore _semaphore;
};

class Empty::SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    SPIDeviceManager();

    void init();
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDeviceType, uint8_t index);
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) override;

private:
    SPIDeviceDriver _device;
};
