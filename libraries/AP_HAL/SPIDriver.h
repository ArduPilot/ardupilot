
#ifndef __AP_HAL_SPI_DRIVER_H__
#define __AP_HAL_SPI_DRIVER_H__

#include "AP_HAL_Namespace.h"


class AP_HAL::SPIDeviceManager {
public:
    virtual void init(void *) = 0;
    virtual AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice) = 0;
};

/**
 * We still need an abstraction for performing bulk
 * transfers to be portable to other platforms.
 */

class AP_HAL::SPIDeviceDriver {
public:
    virtual void init() = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) = 0;

    virtual void cs_assert() = 0;
    virtual void cs_release() = 0;
    virtual uint8_t transfer (uint8_t data) = 0;
    virtual void transfer (const uint8_t *data, uint16_t len) = 0;
};

#endif // __AP_HAL_SPI_DRIVER_H__

