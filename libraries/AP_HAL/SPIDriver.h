
#ifndef __AP_HAL_SPI_DRIVER_H__
#define __AP_HAL_SPI_DRIVER_H__

#include "AP_HAL_Namespace.h"

/**
 * A simple SPIDriver interface directly copied from Arduino SPI driver.
 * This will not be the final AP_HAL interface.
 * We will need an abstraction for selecting slave devices and performing bulk
 * transfers to be portable to other platforms.
 */

class AP_HAL::SPIDriver {
public:
    SPIDriver() {}
    virtual void init(void *) = 0;
    virtual void set_freq(uint32_t freq_hz) = 0;
    virtual uint8_t transfer (uint8_t data) = 0;
};

#endif // __AP_HAL_SPI_DRIVER_H__

