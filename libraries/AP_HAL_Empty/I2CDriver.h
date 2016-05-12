#pragma once

#include "AP_HAL_Empty.h"

class Empty::I2CDriver : public AP_HAL::I2CDriver {
public:
    I2CDriver(AP_HAL::Semaphore* semaphore) : _semaphore(semaphore) {}
    void begin() override;
    void end() override;
    void setTimeout(uint16_t ms) override;
    void setHighSpeed(bool active) override;

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data) override;
    
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val) override;
    
    /* writeRegisters: write bytes to contiguous registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                           uint8_t len, uint8_t* data) override;

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data) override;
    
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data) override;
    
    /* readRegister: read contiguous device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                          uint8_t len, uint8_t* data) override;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    /* readRegistersMultiple: read contiguous device registers. 
       Equivalent to count calls to readRegisters() */
    virtual uint8_t readRegistersMultiple(uint8_t addr, uint8_t reg,
                                          uint8_t len, uint8_t count, 
                                          uint8_t* data) override;
#endif
    
    uint8_t lockup_count();

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

private:
    AP_HAL::Semaphore* _semaphore;
};
