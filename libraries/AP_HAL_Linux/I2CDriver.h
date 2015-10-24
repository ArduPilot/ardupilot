
#ifndef __AP_HAL_LINUX_I2CDRIVER_H__
#define __AP_HAL_LINUX_I2CDRIVER_H__

#include "AP_HAL_Linux.h"

class Linux::I2CDriver : public AP_HAL::I2CDriver {
public:
    I2CDriver(AP_HAL::Semaphore* semaphore, const char *device);
    I2CDriver(AP_HAL::Semaphore* semaphore, const char * const devpaths[]);
    ~I2CDriver();

    void begin();
    void end();
    void setTimeout(uint16_t ms);
    void setHighSpeed(bool active);

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);

    /* readRegister: read contigious device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data);

    uint8_t readRegistersMultiple(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t count, 
                                  uint8_t* data);

    uint8_t lockup_count();

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

private:
    bool set_address(uint8_t addr);

    AP_HAL::Semaphore* _semaphore;
    char *_device = NULL;
    int _fd = -1;
    uint8_t _addr;
};

#endif // __AP_HAL_LINUX_I2CDRIVER_H__
