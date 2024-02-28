#pragma once

#include "SIM_I2CDevice.h"

namespace SITL {

class SMBusRegEnum : public I2CRegEnum {
};

class SMBusDevice : public I2CDevice, private I2CRegisters_16Bit {
public:

    SMBusDevice() :
        I2CDevice(),
        I2CRegisters_16Bit()
        { }

    void set_register(uint8_t reg, uint16_t value) {
        I2CRegisters_16Bit::set_register(reg, value);
    }
    void set_register(uint8_t reg, int16_t value) {
        I2CRegisters_16Bit::set_register(reg, value);
    }

    uint16_t get_reg_value(uint8_t reg) {
        return I2CRegisters_16Bit::get_reg_value(reg);
    }


protected:

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void set_block(uint8_t block, uint8_t *value, uint8_t valuelen);
    void add_block(const char *name, uint8_t reg, I2CRegisters::RegMode mode);

    void set_block(uint8_t block, const char *value) {
        set_block(block, (uint8_t*)value, strlen(value));
    }

    void add_register(const char *name, uint8_t reg, I2CRegisters::RegMode mode) {
        I2CRegisters_16Bit::add_register(name, reg, mode);
    }

private:

    const char *blockname[256];
    Bitmask<256> writable_blocks;
    Bitmask<256> readable_blocks;

    // 256 pointers-to-malloced-values:
    char *values[256];
    uint8_t value_lengths[256];
};

} // namespace SITL
