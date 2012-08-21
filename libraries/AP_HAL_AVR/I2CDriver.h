
#ifndef __AP_HAL_AVR_I2C_DRIVER_H__
#define __AP_HAL_AVR_I2C_DRIVER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRI2CDriver : public AP_HAL::I2CDriver {
public:
    AVRI2CDriver(): _init(0) {}
    void init() { _init = 1; } 
private:
    int _init;
};

#endif // __AP_HAL_AVR_I2C_DRIVER_H__

