#pragma once

#include "SIM_I2C.h"

#include <SITL/SIM_Aircraft.h>

namespace SITL {

class I2CRegEnum {
    // a class to hold register addresses as an enumeration
};

class I2CRegisters {

protected:

    virtual int rdwr(I2C::i2c_rdwr_ioctl_data *&data) = 0;

    void add_register(const char *name, uint8_t reg, int8_t mode);

    const char *regname[256];
    Bitmask<256> writable_registers;
    Bitmask<256> readable_registers;

};

class I2CRegisters_16Bit : public I2CRegisters {
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    void set_register(uint8_t reg, uint16_t value);
    void set_register(uint8_t reg, int16_t value);

protected:

    uint16_t word[256];
};

class I2CRegisters_8Bit : public I2CRegisters {
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    void set_register(uint8_t reg, uint8_t value);
    void set_register(uint8_t reg, int8_t value);

    uint8_t get_register(uint8_t num) {
        return byte[(uint8_t)num];
    }

protected:

    uint8_t byte[256];
};

class I2CDevice {
public:
    virtual void init() {}

    virtual void update(const class Aircraft &aircraft) { }

    virtual int rdwr(I2C::i2c_rdwr_ioctl_data *&data) = 0;
};

} // namespace SITL
