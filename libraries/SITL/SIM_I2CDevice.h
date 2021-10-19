#pragma once

#include "SIM_I2C.h"

#include <SITL/SIM_Aircraft.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/Bitmask.h>

namespace SITL {

class I2CRegEnum {
    // a class to hold register addresses as an enumeration
};

class I2CRegisters {

public:

    enum class RegMode {
        RDONLY = 11,
        WRONLY = 22,
        RDWR = 33,
    };

protected:

    virtual int rdwr(I2C::i2c_rdwr_ioctl_data *&data) = 0;

    void add_register(const char *name, uint8_t reg, RegMode mode);

    const char *regname[256];
    Bitmask<256> writable_registers;
    Bitmask<256> readable_registers;

    void set_debug(bool value) { debug = value; }
    bool get_debug() const { return debug; }

private:
    bool debug;
};

class I2CRegisters_ConfigurableLength : public I2CRegisters {
public:
    void add_register(const char *name, uint8_t reg, uint8_t len, RegMode mode);
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    void set_register(uint8_t reg, uint8_t *data, uint8_t len);
    void set_register(uint8_t reg, uint16_t value);
    void set_register(uint8_t reg, int16_t value);
    void set_register(uint8_t reg, uint8_t value);
    void set_register(uint8_t reg, int8_t value);

    // void get_reg_value(uint8_t reg, int8_t &value) const;
    void get_reg_value(uint8_t reg, uint8_t &value) const;
    // void get_reg_value(uint8_t reg, int16_t &value) const;
    // void get_reg_value(uint8_t reg, uint16_t &value) const;
    // void get_reg_value(uint8_t reg, uint8_t *value, uint8_t len) const;

protected:

    uint32_t reg_data[256];  // OK, so not *that* configurable ATM....
    uint8_t reg_data_len[256];
};

class I2CRegisters_16Bit : public I2CRegisters {
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    void set_register(uint8_t reg, uint16_t value);
    void set_register(uint8_t reg, int16_t value);

    uint16_t get_reg_value(uint8_t reg) {
        return be16toh(word[reg]);
    }

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

    // dies if register does not have value value
    void assert_register_value(uint8_t reg, uint8_t value);

protected:

    uint8_t byte[256];
};

/*
  for devices that take a command then will provide data to a read();
  for example, MCU writes 0x07 to device then expects to be able to
  read 2 bytes from it.
*/
class I2CCommandResponseDevice {
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data);

protected:

    // time taken for device to process command:
    uint16_t command_processing_time_ms() const { return 20; }
    virtual uint8_t command_take_reading() const = 0;
    virtual uint16_t reading() const = 0;

    uint32_t cmd_take_reading_received_ms;
};

class I2CDevice {
public:
    virtual void init() {}

    virtual void update(const class Aircraft &aircraft) { }

    virtual int rdwr(I2C::i2c_rdwr_ioctl_data *&data) = 0;
};

} // namespace SITL
