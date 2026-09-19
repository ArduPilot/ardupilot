#pragma once

#include "SIM_config.h"

#if AP_SIM_INVENSENSEV3_ENABLED

#include "SIM_I2CDevice.h"
#include "SIM_SPIDevice.h"

namespace SITL {

class InvensenseV3DevReg : public I2CRegEnum {
public:
    static const uint8_t WHOAMI        = 0x75;
    // static const uint8_t INT_CONFIG    = 0x14;
    static const uint8_t FIFO_CONFIG   = 0x16;
    static const uint8_t PWR_MGMT0     = 0x4e;
    static const uint8_t GYRO_CONFIG0  = 0x4f;
    static const uint8_t ACCEL_CONFIG0 = 0x50;
    static const uint8_t FIFO_CONFIG1  = 0x5f;
    // static const uint8_t FIFO_CONFIG2  = 0x60;
    // static const uint8_t FIFO_CONFIG3  = 0x61;
    // static const uint8_t INT_SOURCE0   = 0x65;
    static const uint8_t SIGNAL_PATH_RESET = 0x4b;
    static const uint8_t INTF_CONFIG0  = 0x4c;
    static const uint8_t FIFO_COUNTH   = 0x2e;
    static const uint8_t FIFO_DATA     = 0x30;
    static const uint8_t BANK_SEL      = 0x76;

    // anti-alias filter config registers.  On hardware these live in
    // banks 1 (gyro) and 2 (accel), selected via BANK_SEL, but their
    // addresses do not collide with any bank 0 register the simulation
    // uses, so they are modelled in the one flat register file.
    static const uint8_t GYRO_CONFIG_STATIC2  = 0x0b;   // bank 1
    static const uint8_t GYRO_CONFIG_STATIC3  = 0x0c;   // bank 1
    static const uint8_t GYRO_CONFIG_STATIC4  = 0x0d;   // bank 1
    static const uint8_t GYRO_CONFIG_STATIC5  = 0x0e;   // bank 1
    static const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;   // bank 2
    static const uint8_t ACCEL_CONFIG_STATIC3 = 0x04;   // bank 2
    static const uint8_t ACCEL_CONFIG_STATIC4 = 0x05;   // bank 2
};

// The InvensenseV3 IMUs appear on both I2C and SPI buses on real hardware.
// The two buses are modelled as two independent simulators (rather than one
// class trying to be both), each backed by its own register file and FIFO.

// I2C flavour: register/block/fifo accesses come in as i2c_rdwr transactions.
class InvensenseV3_I2C : public I2CDevice, protected I2CRegisters_8Bit
{
public:
    void init() override {
        add_register("WHOAMI", InvensenseV3DevReg::WHOAMI, I2CRegisters::RegMode::RDONLY);
        add_register("FIFO_CONFIG", InvensenseV3DevReg::FIFO_CONFIG, I2CRegisters::RegMode::RDWR);
        add_register("FIFO_CONFIG1", InvensenseV3DevReg::FIFO_CONFIG1, I2CRegisters::RegMode::RDWR);
        add_register("INTF_CONFIG0", InvensenseV3DevReg::INTF_CONFIG0, I2CRegisters::RegMode::RDWR);
        add_register("SIGNAL_PATH_RESET", InvensenseV3DevReg::SIGNAL_PATH_RESET, I2CRegisters::RegMode::RDWR);
        add_register("PWR_MGMT0", InvensenseV3DevReg::PWR_MGMT0, I2CRegisters::RegMode::RDWR);
        add_register("GYRO_CONFIG0", InvensenseV3DevReg::GYRO_CONFIG0, I2CRegisters::RegMode::RDWR);
        add_register("ACCDEL_CONFIG0", InvensenseV3DevReg::ACCEL_CONFIG0, I2CRegisters::RegMode::RDWR);

        // sample count!
        add_block("FIFO_COUNTH", InvensenseV3DevReg::FIFO_COUNTH, 2, I2CRegisters::RegMode::RDONLY);

        add_fifo("FIFO_DATA", InvensenseV3DevReg::FIFO_DATA, I2CRegisters::RegMode::RDONLY);
    }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    virtual float accel_scale() const = 0;
    float gyro_scale() const { return (0.0174532f / 16.4f); }

private:

    // sets the FIFO_COUNTH block based on the number of samples in FIFO_DATA
    void update_sample_count();

    void assert_register_values();

    void add_fifo(const char *name, uint8_t reg, I2CRegisters::RegMode mode);
    bool write_to_fifo(uint8_t fifo, uint8_t *value, uint8_t valuelen);

    int rdwr_fifo(I2C::i2c_rdwr_ioctl_data *&data);
    static const uint8_t fifo_len = 64;
    const char *fifoname[256];
    Bitmask<256> writable_fifos;
    Bitmask<256> readable_fifos;

    // 256 pointers-to-malloced-values:
    char *values[256];
    uint8_t value_lengths[256];

    // swiped from the driver:
    struct PACKED FIFOData {
        uint8_t header;
        int16_t accel[3];
        int16_t gyro[3];
        int8_t temperature;
        uint16_t timestamp;
    };

    // block support
    // for things that are larger than a register....
    int rdwr_block(I2C::i2c_rdwr_ioctl_data *&data);
    void add_block(const char *name, uint8_t addr, uint8_t len, I2CRegisters::RegMode mode);
    void set_block(uint8_t addr, uint8_t *value, uint8_t valuelen);

    const char *blockname[256];
    Bitmask<256> writable_blocks;
    Bitmask<256> readable_blocks;

    // 256 pointers-to-malloced-values:
    char *block_values[256];
    uint8_t block_value_lengths[256];
};

// SPI flavour: a flat register file (I2CRegisters_8Bit, used only as register
// storage) and a FIFO back the register accesses and the FIFO_DATA burst the
// driver streams.
class InvensenseV3 : public SPIDevice, protected I2CRegisters_8Bit
{
public:
    void init() override {
        add_register("WHOAMI", InvensenseV3DevReg::WHOAMI, I2CRegisters::RegMode::RDONLY);
        add_register("FIFO_CONFIG", InvensenseV3DevReg::FIFO_CONFIG, I2CRegisters::RegMode::RDWR);
        add_register("FIFO_CONFIG1", InvensenseV3DevReg::FIFO_CONFIG1, I2CRegisters::RegMode::RDWR);
        add_register("INTF_CONFIG0", InvensenseV3DevReg::INTF_CONFIG0, I2CRegisters::RegMode::RDWR);
        add_register("SIGNAL_PATH_RESET", InvensenseV3DevReg::SIGNAL_PATH_RESET, I2CRegisters::RegMode::RDWR);
        add_register("PWR_MGMT0", InvensenseV3DevReg::PWR_MGMT0, I2CRegisters::RegMode::RDWR);
        add_register("GYRO_CONFIG0", InvensenseV3DevReg::GYRO_CONFIG0, I2CRegisters::RegMode::RDWR);
        add_register("ACCDEL_CONFIG0", InvensenseV3DevReg::ACCEL_CONFIG0, I2CRegisters::RegMode::RDWR);

        // bank select and the (banked) anti-alias filter config registers
        // the driver programs during startup
        add_register("BANK_SEL", InvensenseV3DevReg::BANK_SEL, I2CRegisters::RegMode::RDWR);
        add_register("GYRO_CONFIG_STATIC2", InvensenseV3DevReg::GYRO_CONFIG_STATIC2, I2CRegisters::RegMode::RDWR);
        add_register("GYRO_CONFIG_STATIC3", InvensenseV3DevReg::GYRO_CONFIG_STATIC3, I2CRegisters::RegMode::RDWR);
        add_register("GYRO_CONFIG_STATIC4", InvensenseV3DevReg::GYRO_CONFIG_STATIC4, I2CRegisters::RegMode::RDWR);
        add_register("GYRO_CONFIG_STATIC5", InvensenseV3DevReg::GYRO_CONFIG_STATIC5, I2CRegisters::RegMode::RDWR);
        add_register("ACCEL_CONFIG_STATIC2", InvensenseV3DevReg::ACCEL_CONFIG_STATIC2, I2CRegisters::RegMode::RDWR);
        add_register("ACCEL_CONFIG_STATIC3", InvensenseV3DevReg::ACCEL_CONFIG_STATIC3, I2CRegisters::RegMode::RDWR);
        add_register("ACCEL_CONFIG_STATIC4", InvensenseV3DevReg::ACCEL_CONFIG_STATIC4, I2CRegisters::RegMode::RDWR);

        add_fifo("FIFO_DATA", InvensenseV3DevReg::FIFO_DATA, I2CRegisters::RegMode::RDONLY);
    }

    void update(const class Aircraft &aircraft) override;

    // I2CRegisters_8Bit is used only as flat register storage here; pull its
    // i2c rdwr() into scope so the SPI rdwr() below overloads rather than hides
    // it (clang -Woverloaded-virtual, which is -Werror on macOS).
    using I2CRegisters_8Bit::rdwr;

    // SPI bus access
    int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) override;

    virtual float accel_scale() const = 0;
    float gyro_scale() const { return (0.0174532f / 16.4f); }

private:

    // sets FIFO_COUNTH/FIFO_COUNTL from the number of samples in FIFO_DATA
    void update_sample_count();

    void add_fifo(const char *name, uint8_t reg, I2CRegisters::RegMode mode);
    bool write_to_fifo(uint8_t fifo, uint8_t *value, uint8_t valuelen);

    // pop up to len bytes out of a fifo
    int read_from_fifo(uint8_t fifo, uint8_t *dst, uint8_t len);

    static const uint8_t fifo_len = 64;
    const char *fifoname[256];
    Bitmask<256> writable_fifos;
    Bitmask<256> readable_fifos;

    // 256 pointers-to-malloced-values:
    char *values[256];
    uint8_t value_lengths[256];

    // swiped from the driver:
    struct PACKED FIFOData {
        uint8_t header;
        int16_t accel[3];
        int16_t gyro[3];
        int8_t temperature;
        uint16_t timestamp;
    };
};

} // namespace SITL

#endif  // AP_SIM_INVENSENSEV3_ENABLED
