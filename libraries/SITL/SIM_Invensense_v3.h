#include "SIM_I2CDevice.h"

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
    // static const uint8_t BANK_SEL      = 0x76;
};

class InvensenseV3 : public I2CDevice, protected I2CRegisters_8Bit
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

} // namespace SITL
