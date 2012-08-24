
#ifndef __AP_HAL_AVR_I2C_DRIVER_H__
#define __AP_HAL_AVR_I2C_DRIVER_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVRI2CDRIVER_MAX_BUFFER_SIZE 32

class AP_HAL_AVR::AVRI2CDriver : public AP_HAL::I2CDriver {
public:
    AVRI2CDriver() {}

    void begin();
    void end();
    void setTimeout(uint16_t ms) { _timeoutDelay = ms; }
    void setHighSpeed(bool active);

    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
        /* Sometimes avr-gcc fails at dereferencing a uint8_t arg. */
        uint8_t data[1];
        data[0] = val;
        return writeRegisters(addr, reg, 1, data);
    }

    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                           uint8_t len, uint8_t* data);
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data) {
        return readRegisters(addr, reg, 1, data);
    }
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                          uint8_t len, uint8_t* data);
    uint8_t lockup_count() { return _lockup_count; }

private:
    uint8_t _start();
    uint8_t _stop();
    uint8_t _sendAddress(uint8_t addr);
    uint8_t _sendByte(uint8_t data);
    uint8_t _receiveByte(bool ack);
    void    _handleLockup();

    uint8_t _waitInterrupt();
    uint8_t _waitStop();
    
    uint8_t _lockup_count;
    uint16_t _timeoutDelay;
};

#endif // __AP_HAL_AVR_I2C_DRIVER_H__

