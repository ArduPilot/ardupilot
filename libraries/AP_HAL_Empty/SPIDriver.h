
#ifndef __AP_HAL_EMPTY_SPIDRIVER_H__
#define __AP_HAL_EMPTY_SPIDRIVER_H__

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
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice, uint8_t index);
private:
    SPIDeviceDriver _device;
};

#endif // __AP_HAL_EMPTY_SPIDRIVER_H__
