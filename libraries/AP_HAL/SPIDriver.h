
#ifndef __AP_HAL_SPI_DRIVER_H__
#define __AP_HAL_SPI_DRIVER_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::SPIDriver {
public:
    SPIDriver() {}
    virtual void init() = 0;
};

#endif // __AP_HAL_SPI_DRIVER_H__

