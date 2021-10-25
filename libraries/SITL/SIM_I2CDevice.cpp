#include "SIM_I2CDevice.h"
#include <AP_HAL/utility/sparse-endian.h>

#ifndef HAL_DEBUG_I2DEVICE
#define HAL_DEBUG_I2DEVICE 0
#endif

#if HAL_DEBUG_I2DEVICE
#include <GCS_MAVLink/GCS.h>
#define DEBUG(fmt, args ...)  do { if (get_debug()) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
#define DEBUG(fmt, args ...)
#endif

void SITL::I2CRegisters::add_register(const char *name, uint8_t reg, RegMode mode)
{
    DEBUG("Adding register %u (0x%02x) (%s)", reg, reg, name);
    regname[reg] = name;
    if (mode == RegMode::RDONLY || mode == RegMode::RDWR) {
        readable_registers.set((uint8_t)reg);
    }
    if (mode == RegMode::WRONLY || mode == RegMode::RDWR) {
        writable_registers.set((uint8_t)reg);
    }
}

void SITL::I2CRegisters_16Bit::set_register(uint8_t reg, uint16_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %u (0x%02x) (%s) to 0x%02x", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value);

    word[reg] = htobe16(value);
}

void SITL::I2CRegisters_16Bit::set_register(uint8_t reg, int16_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %s (%u) to 0x%02x", regname[reg], (unsigned)reg, (signed)value);
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
    DEBUG("Setting %u (0x%02x) (%s) to 0x%02x (%c)", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value, value);
    byte[reg] = value;
}

void SITL::I2CRegisters_8Bit::set_register(uint8_t reg, int8_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %s (%u) to 0x%02x (%c)", regname[reg], (unsigned)reg, (signed)value, value);
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


void SITL::I2CRegisters_ConfigurableLength::add_register(const char *name, uint8_t reg, uint8_t len, RegMode mode)
{
    SITL::I2CRegisters::add_register(name, reg, mode);
    if (len > 4) {
        AP_HAL::panic("Only up to 4 bytes");
    }
    reg_data_len[reg] = len;
}

void SITL::I2CRegisters_ConfigurableLength::set_register(uint8_t reg, uint16_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %u (0x%02x) (%s) to 0x%02x", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value);

    if (reg_data_len[reg] != 2) {
        AP_HAL::panic("Invalid set_register len");
    }
    reg_data[reg] = htobe16(value);
}

void SITL::I2CRegisters_ConfigurableLength::set_register(uint8_t reg, int16_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %s (%u) to 0x%02x", regname[reg], (unsigned)reg, (signed)value);

    if (reg_data_len[reg] != 2) {
        AP_HAL::panic("Invalid set_register len");
    }
    reg_data[reg] = htobe16(value);
}

void SITL::I2CRegisters_ConfigurableLength::set_register(uint8_t reg, uint8_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %u (0x%02x) (%s) to 0x%02x (%c)", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value, value);
    if (reg_data_len[reg] != 1) {
        AP_HAL::panic("Invalid set_register len");
    }
    reg_data[reg] = value;
}

void SITL::I2CRegisters_ConfigurableLength::set_register(uint8_t reg, int8_t value)
{
    if (regname[reg] == nullptr) {
        AP_HAL::panic("Setting un-named register %u", reg);
    }
    DEBUG("Setting %s (%u) to 0x%02x (%c)", regname[reg], (unsigned)reg, (signed)value, value);
    if (reg_data_len[reg] != 1) {
        AP_HAL::panic("Invalid set_register len");
    }
    reg_data[reg] = value;
}

int SITL::I2CRegisters_ConfigurableLength::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // data read request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        const uint8_t reg_addr = data->msgs[0].buf[0];
        if (data->msgs[1].len != reg_data_len[reg_addr]) {
            AP_HAL::panic("Invalid rdwr len");
        }
        if (!readable_registers.get(reg_addr)) {
            // ::printf("Register 0x%02x is not readable!\n", reg_addr);
            return -1;
        }
        const uint32_t register_value = reg_data[reg_addr];
        if (data->msgs[1].len == 1) {
            data->msgs[1].buf[0] = register_value >> 24;
        } else if (data->msgs[1].len == 2) {
            const uint16_t v = htobe16(register_value & 0xffff);
            memcpy(&(data->msgs[1].buf[0]), &v, 2);
        } else {
            AP_HAL::panic("Bad length"); // FIXME
        }
        data->msgs[1].len = reg_data_len[reg_addr];
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
        const uint8_t data_msg_len = data->msgs[0].len - 1;
        if (data_msg_len != reg_data_len[reg_addr]) {
            AP_HAL::panic("Invalid rdwr len");
        }
        memcpy((uint8_t*)&reg_data[reg_addr], &data->msgs[0].buf[1], data_msg_len);
        return 0;
    }

    return -1;
};

void SITL::I2CRegisters_ConfigurableLength::get_reg_value(uint8_t reg, uint8_t &value) const
{
    if (reg_data_len[reg] != 1) {
	    AP_HAL::panic("Invalid reg_reg_value len");
    }
    value = reg_data[reg];
}
