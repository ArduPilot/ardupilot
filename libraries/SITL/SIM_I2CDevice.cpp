#include "SIM_I2CDevice.h"
#include <AP_HAL/utility/sparse-endian.h>

void SITL::I2CRegisters::add_register(const char *name, uint8_t reg, int8_t mode)
{
    // ::fprintf(stderr, "Adding register %u (0x%02x) (%s)\n", reg, reg, name);
    regname[reg] = name;
    if (mode == O_RDONLY || mode == O_RDWR) {
        readable_registers.set((uint8_t)reg);
    }
    if (mode == O_WRONLY || mode == O_RDWR) {
        writable_registers.set((uint8_t)reg);
    }
}

void SITL::I2CRegisters_16Bit::set_register(uint8_t reg, uint16_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    // ::fprintf(stderr, "Setting %u (0x%02x) (%s) to 0x%02x (%c)\n", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value, value);
    word[reg] = htobe16(value);
}

void SITL::I2CRegisters_16Bit::set_register(uint8_t reg, int16_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    // ::fprintf(stderr, "Setting %s (%u) to 0x%02x (%c)\n", regname[reg], (unsigned)reg, (signed)value, value);
    word[reg] = htobe16(value);
}

int SITL::I2CRegisters_16Bit::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // data read request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_base_addr = data->msgs[0].buf[0];
        uint8_t bytes_copied = 0;
        while (bytes_copied < data->msgs[1].len) {
            const uint8_t reg_addr = reg_base_addr + bytes_copied/2;
            if (!readable_registers.get(reg_addr)) {
                // ::printf("Register 0x%02x is not readable!\n", reg_addr);
                return -1;
            }
            const uint16_t register_value = word[reg_addr];
            data->msgs[1].buf[bytes_copied++] = register_value >> 8;
            if (bytes_copied < data->msgs[1].len) {
                data->msgs[1].buf[bytes_copied++] = register_value & 0xFF;
            }
        }
        data->msgs[1].len = bytes_copied;
        return 0;
    }

    if (data->nmsgs == 1) {
        // data write request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        // FIXME: handle multi-register writes
        const uint8_t reg_addr = data->msgs[0].buf[0];
        if (!writable_registers.get(reg_addr)) {
            AP_HAL::panic("Register 0x%02x is not writable!", reg_addr);
        }
        const uint16_t register_value = data->msgs[0].buf[2] << 8 | data->msgs[0].buf[1];
        word[reg_addr] = register_value;
        return 0;
    }

    return -1;
};



void SITL::I2CRegisters_8Bit::set_register(uint8_t reg, uint8_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    // ::fprintf(stderr, "Setting %u (0x%02x) (%s) to 0x%02x (%c)\n", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value, value);
    byte[reg] = value;
}

void SITL::I2CRegisters_8Bit::set_register(uint8_t reg, int8_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    // ::fprintf(stderr, "Setting %s (%u) to 0x%02x (%c)\n", regname[reg], (unsigned)reg, (signed)value, value);
    byte[reg] = value;
}

int SITL::I2CRegisters_8Bit::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // data read request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_base_addr = data->msgs[0].buf[0];
        uint8_t bytes_copied = 0;
        while (bytes_copied < data->msgs[1].len) {
            const uint8_t reg_addr = reg_base_addr + bytes_copied;
            if (!readable_registers.get(reg_addr)) {
                // ::printf("Register 0x%02x is not readable!\n", reg_addr);
                return -1;
            }
            const uint8_t register_value = byte[reg_addr];
            data->msgs[1].buf[bytes_copied++] = register_value;
        }
        data->msgs[1].len = bytes_copied;
        return 0;
    }

    if (data->nmsgs == 1) {
        // data write request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_base_addr = data->msgs[0].buf[0];
        uint8_t bytes_copied = 0;
        while (bytes_copied < data->msgs[0].len-1) {
            const uint8_t reg_addr = reg_base_addr + bytes_copied;
            if (!writable_registers.get(reg_addr)) {
                AP_HAL::panic("Register 0x%02x is not writable!", reg_addr);
            }
            const uint8_t register_value = data->msgs[0].buf[1+bytes_copied];
            byte[reg_addr] = register_value;
            bytes_copied++;
        }
        return 0;
    }

    return -1;
};


void SITL::I2CRegisters_8Bit::assert_register_value(uint8_t reg, uint8_t value)
{
    if (byte[reg] != value) {
        AP_HAL::panic("Register 0x%02x (%s) was expected to have value (%02x) but has value (%02x)", reg, regname[reg], byte[reg], value);
    }
}

int SITL::I2CCommandResponseDevice::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    const uint32_t now = AP_HAL::millis();

    struct I2C::i2c_msg &msg = data->msgs[0];
    if (msg.flags == I2C_M_RD) {
        // driver is attempting to receive reading...
        if (now - cmd_take_reading_received_ms < command_processing_time_ms()) {
            // not sure we ought to be returning -1 here - what does
            // the real device do?  return stale data?  garbage data?
            return -1;
        }
        if (msg.len != 2) {
            AP_HAL::panic("Unxpected message length (%u)", msg.len);
        }

        const uint16_t value = reading();

        msg.buf[0] = value >> 8;
        msg.buf[1] = value & 0xff;

        return 0;
    }

    const uint8_t cmd = msg.buf[0];
    if (cmd != command_take_reading()) {
        AP_HAL::panic("Unknown command (%u)", cmd);
    }
    cmd_take_reading_received_ms = now;

    return 0;
}
