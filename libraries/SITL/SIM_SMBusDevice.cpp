#include "SIM_SMBusDevice.h"

int SITL::SMBusDevice::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    // see if this is a block...
    const uint8_t addr = data->msgs[0].buf[0];
    if (blockname[addr] == nullptr) {
        // not a block
        return I2CRegisters_16Bit::rdwr(data);
    }

    // it is a block.
    if (data->nmsgs == 2) {
        // data read request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }

        data->msgs[1].buf[0] = value_lengths[addr];
        const uint8_t to_copy = MIN(data->msgs[1].len-1, value_lengths[addr]);
        memcpy(&data->msgs[1].buf[1], values[addr], to_copy);
        data->msgs[1].len = to_copy + 1;
        return 0;
    }

    if (data->nmsgs == 1) {
        // data write request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        AP_HAL::panic("block writes not implemented");
    }

    return -1;
}

void SITL::SMBusDevice::add_block(const char *name, uint8_t reg, I2CRegisters::RegMode mode)
{
    // ::fprintf(stderr, "Adding block %u (0x%02x) (%s)\n", reg, reg, name);
    blockname[reg] = name;
    if (mode == I2CRegisters::RegMode::RDONLY ||
        mode == I2CRegisters::RegMode::RDWR) {
        readable_blocks.set((uint8_t)reg);
    }
    if (mode == I2CRegisters::RegMode::WRONLY ||
        mode == I2CRegisters::RegMode::RDWR) {
        writable_blocks.set((uint8_t)reg);
    }
}

void SITL::SMBusDevice::set_block(uint8_t block, uint8_t *value, uint8_t valuelen)
{
    if (blockname[block] == nullptr) {
        AP_HAL::panic("Setting un-named block %u", block);
    }
    // ::fprintf(stderr, "Setting %u (0x%02x) (%s) to 0x%02x (%c)\n", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value, value);
    if (valuelen == 0) {
        AP_HAL::panic("Zero-length values not permitted by spec");
    }
    if (values[block] != nullptr) {
        free(values[block]);
    }
    values[block] = (char*)malloc(valuelen);
    memcpy(values[block], value, valuelen);
    value_lengths[block] = valuelen;
}
