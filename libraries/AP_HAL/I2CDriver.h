
#ifndef __AP_HAL_I2C_DRIVER_H__
#define __AP_HAL_I2C_DRIVER_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::I2CDriver {
public:
    I2CDriver() {}
    virtual void init() = 0;
};

#endif // __AP_HAL_I2C_DRIVER_H__

