
#ifndef __AP_HAL_I2C_DRIVER_H__
#define __AP_HAL_I2C_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

class AP_HAL::I2CDriver {
public:
    I2CDriver() {}
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void setTimeout(uint16_t ms) = 0;
    virtual void setHighSpeed(bool active) = 0;

    virtual uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val) = 0;
    virtual uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data) = 0;
    virtual uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data) = 0;
    virtual uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data) = 0;

    virtual uint8_t lockup_count();
};

#endif // __AP_HAL_I2C_DRIVER_H__

