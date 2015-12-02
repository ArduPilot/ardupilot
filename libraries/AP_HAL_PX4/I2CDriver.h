
#ifndef __AP_HAL_PX4_I2CDRIVER_H__
#define __AP_HAL_PX4_I2CDRIVER_H__

#include "AP_HAL_PX4.h"
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

class PX4::PX4I2CDriver : public AP_HAL::I2CDriver {
public:
    PX4I2CDriver(void);

    void begin() {}
    void end() {}
    void setTimeout(uint16_t ms) {}
    void setHighSpeed(bool active) {}

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);

    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);

    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
    
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);

    /* readRegister: read contigious device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data);
    
    uint8_t lockup_count() { return 0; }

    AP_HAL::Semaphore* get_semaphore() { return &semaphore; }

private:
    // we use an empty semaphore as the underlying I2C class already has a semaphore
    Empty::EmptySemaphore semaphore;
    PX4_I2C *px4_i2c = nullptr;
};

#endif // __AP_HAL_PX4_I2CDRIVER_H__
