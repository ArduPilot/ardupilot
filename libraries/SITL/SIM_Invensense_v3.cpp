#include "SIM_Invensense_v3.h"

#include <stdio.h>

void SITL::InvensenseV3::update(const class Aircraft &aircraft)
{
    assert_storage_size<FIFOData, 16> _assert_fifo_size;
    (void)_assert_fifo_size;

    const SIM *sitl = AP::sitl();
    const int16_t xAccel = sitl->state.xAccel / accel_scale();
    const int16_t yAccel = sitl->state.yAccel / accel_scale();
    const int16_t zAccel = sitl->state.zAccel / accel_scale();

    const int16_t p = radians(sitl->state.rollRate) / gyro_scale();
    const int16_t q = radians(sitl->state.pitchRate) / gyro_scale();
    const int16_t r = radians(sitl->state.yawRate) / gyro_scale();

    struct FIFOData new_data  {
        0x68,
        { xAccel, yAccel, zAccel },
        { p, q, r },
        21,  // temperature
        AP_HAL::millis16()   // timestamp
    };

    for (uint8_t i=0; i<2; i++) {
        if (!write_to_fifo(InvensenseV3DevReg::FIFO_DATA, (uint8_t*)&new_data, sizeof(new_data))) {
            return;
        }
    }
    update_sample_count();
}

// assert_register_values ensures register states when we go to do
// various operations (e.g. reading from FIFO)
void SITL::InvensenseV3::assert_register_values()
{
    static const struct expected_register_values {
        uint8_t reg;
        uint8_t value;
    } expected[] {
        { InvensenseV3DevReg::FIFO_CONFIG, 0x80 },
        { InvensenseV3DevReg::FIFO_CONFIG1, 0x07 },
        { InvensenseV3DevReg::INTF_CONFIG0, 0xC0 },
        { InvensenseV3DevReg::SIGNAL_PATH_RESET, 2 },
        { InvensenseV3DevReg::PWR_MGMT0, 0x0f },
        { InvensenseV3DevReg::GYRO_CONFIG0, 0x05 },
        { InvensenseV3DevReg::ACCEL_CONFIG0, 0x05 },
    };
    for (const auto &stuff : expected) {
        assert_register_value(stuff.reg, stuff.value);
    }
}

int SITL::InvensenseV3::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    const uint8_t addr = data->msgs[0].buf[0];

    // see if it is a fifo...
    if (fifoname[addr] != nullptr) {
        return rdwr_fifo(data);
    }
    // see if it is a block...
    if (blockname[addr] != nullptr) {
        return rdwr_block(data);
    }

    return I2CRegisters_8Bit::rdwr(data);
}

int SITL::InvensenseV3::rdwr_fifo(I2C::i2c_rdwr_ioctl_data *&data)
{
    const uint8_t addr = data->msgs[0].buf[0];

    assert_register_values();

    // check for block/FIFO read/write bits and pieces
    if (data->nmsgs == 2) {
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }

        const uint8_t len = data->msgs[1].len;
        if (len > value_lengths[addr]) {
            if (value_lengths[addr] != 0) {
                // we expect reads and writes into the fifo to be the same size
                AP_HAL::panic("Read of unexpected size");
            }
            return -1;
        }
        memcpy(data->msgs[1].buf, values[addr], len);
        memmove(values[addr], values[addr]+len, value_lengths[addr]-len);
        value_lengths[addr] -= len;
        if (addr == InvensenseV3DevReg::FIFO_DATA) {    // bit of a hack... callback?
            update_sample_count();
        }
        return 0;
    }
    return -1;
}

void SITL::InvensenseV3::add_fifo(const char *name, uint8_t reg, I2CRegisters::RegMode mode)
{
    // ::fprintf(stderr, "Adding fifo %u (0x%02x) (%s)\n", reg, reg, name);
    fifoname[reg] = name;
    if (mode == I2CRegisters::RegMode::RDONLY ||
        mode == I2CRegisters::RegMode::RDWR) {
        readable_fifos.set((uint8_t)reg);
    }
    if (mode == I2CRegisters::RegMode::WRONLY ||
        mode == I2CRegisters::RegMode::RDWR) {
        writable_fifos.set((uint8_t)reg);
    }

    values[reg] = (char*)malloc(fifo_len); // allocate the fifo...
    if (values[reg] == nullptr) {
        AP_HAL::panic("Failed to allocate FIFO...");
    }
}

void SITL::InvensenseV3::update_sample_count()
{
    if (value_lengths[InvensenseV3DevReg::FIFO_DATA] % sizeof(FIFOData)) {
        AP_HAL::panic("fifo data not multiple of sample size");
    }
    uint16_t samplecount = value_lengths[InvensenseV3DevReg::FIFO_DATA]/sizeof(FIFOData);
    set_block(InvensenseV3DevReg::FIFO_COUNTH, (uint8_t*)&samplecount, 2);
}

bool SITL::InvensenseV3::write_to_fifo(uint8_t fifo, uint8_t *value, uint8_t valuelen)
{
    if (fifoname[fifo] == nullptr) {
        AP_HAL::panic("Setting un-named fifo %u", fifo);
    }
    // ::fprintf(stderr, "Setting %u (0x%02x) (%s) to 0x%02x (%c)\n", (unsigned)reg, (unsigned)reg, regname[reg], (unsigned)value, value);
    if (valuelen == 0) {
        AP_HAL::panic("Zero-length values not permitted by spec");
    }
    if (values[fifo] == nullptr) {
        AP_HAL::panic("Write to unallocated FIFO");
    }
    if (value_lengths[fifo] + valuelen > fifo_len) {
        // ::fprintf(stderr, "dropped\n");  // this happens a lot at startup
        return false;  // just drop it
    }
    memcpy(&(values[fifo][value_lengths[fifo]]), value, valuelen);
    value_lengths[fifo] += valuelen;
    if (fifo == InvensenseV3DevReg::FIFO_DATA) {    // bit of a hack... callback?
        update_sample_count();
    }
    return true;
}



void SITL::InvensenseV3::add_block(const char *name, uint8_t addr, uint8_t len, I2CRegisters::RegMode mode)
{
    // ::fprintf(stderr, "Adding block %u (0x%02x) (%s)\n", addr, addr, name);
    blockname[addr] = name;
    block_values[addr] = (char*)malloc(len);
    block_value_lengths[addr] = len;
    if (block_values[addr] == nullptr) {
        AP_HAL::panic("Allocation failed for block (len=%u)", len);
    }
    if (mode == I2CRegisters::RegMode::RDONLY ||
        mode == I2CRegisters::RegMode::RDWR) {
        readable_blocks.set((uint8_t)addr);
    }
    if (mode == I2CRegisters::RegMode::WRONLY ||
        mode == I2CRegisters::RegMode::RDWR) {
        writable_blocks.set((uint8_t)addr);
    }
}

void SITL::InvensenseV3::set_block(uint8_t addr, uint8_t *value, uint8_t valuelen)
{
    if (blockname[addr] == nullptr) {
        AP_HAL::panic("Setting un-named block %u", addr);
    }
    if (valuelen != block_value_lengths[addr]) {
        AP_HAL::panic("Invalid block write got=%u want=%u", valuelen, block_value_lengths[addr]);
    }
    memcpy(block_values[addr], value, valuelen);
}

int SITL::InvensenseV3::rdwr_block(I2C::i2c_rdwr_ioctl_data *&data)
{
    const uint8_t addr = data->msgs[0].buf[0];

    // it is a block.
    if (data->nmsgs == 2) {
        // data read request
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }

        if (data->msgs[1].len != block_value_lengths[addr]) {
            AP_HAL::panic("Block read length not equal to block length (got=%u want=%u)", data->msgs[1].len, block_value_lengths[addr]);
        }
        memcpy(&data->msgs[1].buf[0], block_values[addr], data->msgs[1].len);
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
