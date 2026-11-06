#include "SIM_config.h"

#if AP_SIM_INVENSENSEV3_ENABLED

#include "SIM_Invensense_v3.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// InvensenseV3_I2C: the device modelled on the I2C bus
// ---------------------------------------------------------------------------

void SITL::InvensenseV3_I2C::update(const class Aircraft &aircraft)
{
    ASSERT_STORAGE_SIZE(FIFOData, 16);

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
void SITL::InvensenseV3_I2C::assert_register_values()
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

int SITL::InvensenseV3_I2C::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
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

int SITL::InvensenseV3_I2C::rdwr_fifo(I2C::i2c_rdwr_ioctl_data *&data)
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

void SITL::InvensenseV3_I2C::add_fifo(const char *name, uint8_t reg, I2CRegisters::RegMode mode)
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

void SITL::InvensenseV3_I2C::update_sample_count()
{
    if (value_lengths[InvensenseV3DevReg::FIFO_DATA] % sizeof(FIFOData)) {
        AP_HAL::panic("fifo data not multiple of sample size");
    }
    uint16_t samplecount = value_lengths[InvensenseV3DevReg::FIFO_DATA]/sizeof(FIFOData);
    set_block(InvensenseV3DevReg::FIFO_COUNTH, (uint8_t*)&samplecount, 2);
}

bool SITL::InvensenseV3_I2C::write_to_fifo(uint8_t fifo, uint8_t *value, uint8_t valuelen)
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

void SITL::InvensenseV3_I2C::add_block(const char *name, uint8_t addr, uint8_t len, I2CRegisters::RegMode mode)
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

void SITL::InvensenseV3_I2C::set_block(uint8_t addr, uint8_t *value, uint8_t valuelen)
{
    if (blockname[addr] == nullptr) {
        AP_HAL::panic("Setting un-named block %u", addr);
    }
    if (valuelen != block_value_lengths[addr]) {
        AP_HAL::panic("Invalid block write got=%u want=%u", valuelen, block_value_lengths[addr]);
    }
    memcpy(block_values[addr], value, valuelen);
}

int SITL::InvensenseV3_I2C::rdwr_block(I2C::i2c_rdwr_ioctl_data *&data)
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

// ---------------------------------------------------------------------------
// InvensenseV3: the device modelled on the SPI bus
// ---------------------------------------------------------------------------

void SITL::InvensenseV3::update(const class Aircraft &aircraft)
{
    ASSERT_STORAGE_SIZE(FIFOData, 16);

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

// pop up to len bytes out of a fifo into dst
int SITL::InvensenseV3::read_from_fifo(uint8_t addr, uint8_t *dst, uint8_t len)
{
    if (len > value_lengths[addr]) {
        if (value_lengths[addr] != 0) {
            // we expect reads and writes into the fifo to be the same size
            AP_HAL::panic("Read of unexpected size");
        }
        return -1;
    }
    memcpy(dst, values[addr], len);
    memmove(values[addr], values[addr]+len, value_lengths[addr]-len);
    value_lengths[addr] -= len;
    if (addr == InvensenseV3DevReg::FIFO_DATA) {    // bit of a hack... callback?
        update_sample_count();
    }
    return 0;
}

/*
  SPI bus access.  In the Invensense v3 SPI protocol the first byte of a
  transaction is the register address with bit 7 set for reads; register
  reads auto-increment the address, while a read of FIFO_DATA streams fifo
  bytes.
 */
int SITL::InvensenseV3::rdwr(uint8_t count, SPI::spi_ioc_transfer *&tfrs)
{
    uint8_t reg = 0;
    bool is_read = false;
    bool have_reg = false;

    for (uint8_t i=0; i<count; i++) {
        const auto &tfr = tfrs[i];
        const uint8_t *tx = (const uint8_t *)tfr.tx_buf;
        uint8_t *rx = (uint8_t *)tfr.rx_buf;

        uint32_t off = 0;
        if (tx != nullptr) {
            if (!have_reg) {
                is_read = (tx[0] & 0x80) != 0;
                reg = tx[0] & 0x7f;
                have_reg = true;
                off = 1;
            }
            if (!is_read) {
                // remaining tx bytes are register writes (auto-incrementing)
                for (uint32_t j=off; j<tfr.len; j++) {
                    set_register(reg++, tx[j]);
                }
            }
        }

        if (rx != nullptr && is_read) {
            // a full-duplex transfer echoes the command byte in rx[0], so the
            // returned data starts one byte in; a separate read transfer
            // (tx==nullptr) returns data from the first byte
            if (reg == InvensenseV3DevReg::FIFO_DATA) {
                read_from_fifo(reg, &rx[off], tfr.len - off);
            } else {
                for (uint32_t j=off; j<tfr.len; j++) {
                    rx[j] = get_register(reg++);
                }
            }
        }
    }
    return 0;
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
    // store the count into the register file where the driver reads
    // FIFO_COUNTH/FIFO_COUNTL as two auto-incrementing registers over SPI
    rdwr_store_register_value(InvensenseV3DevReg::FIFO_COUNTH, uint8_t(samplecount & 0xff));
    rdwr_store_register_value(uint8_t(InvensenseV3DevReg::FIFO_COUNTH + 1), uint8_t(samplecount >> 8));
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

#endif  // AP_SIM_INVENSENSEV3_ENABLED
