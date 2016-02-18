#pragma once

#include <vector>

#include <AP_HAL/utility/OwnPtr.h>

#include "AP_HAL_Linux.h"
#include "I2CDevice.h"

class Linux::I2CDriver : public AP_HAL::I2CDriver {
public:
    I2CDriver(uint8_t bus);
    I2CDriver(std::vector<const char *> devpaths);

    void begin() { }
    void end() { }
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

    AP_HAL::Semaphore *get_semaphore();

    bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                     uint8_t *recv, uint32_t recv_len)override;

private:
    bool set_address(uint8_t addr);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _fake_dev;
    char *_device = NULL;
    uint8_t _addr;
    bool _print_ioctl_error = true;
};
