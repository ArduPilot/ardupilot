#include "SIM_config.h"

#if AP_SIM_AS5600_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

// datasheet: https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf

class AS5600DevReg : public I2CRegEnum {
public:
    // low
    static const uint8_t ZMCO           = 0x00;
    static const uint8_t ZPOS_HIGH      = 0x01;
    static const uint8_t ZPOS_LOW       = 0x02;
    static const uint8_t MPOS_HIGH      = 0x03;
    static const uint8_t MPOS_LOW       = 0x04;
    static const uint8_t MANG_HIGH      = 0x05;
    static const uint8_t MANG_LOW       = 0x06;
    static const uint8_t CONF_HIGH      = 0x07;
    static const uint8_t CONF_LOW       = 0x08;

    // output registers
    static const uint8_t RAW_ANGLE_HIGH = 0x0C;
    static const uint8_t RAW_ANGLE_LOW  = 0x0D;
    static const uint8_t ANGLE_HIGH     = 0x0E;
    static const uint8_t ANGLE_LOW      = 0x0F;

    // status registers
    static const uint8_t STATUS         = 0x0B;
    static const uint8_t AGC            = 0x1A;
    static const uint8_t MAGNITUDE_HIGH = 0x1B;
    static const uint8_t MAGNITUDE_LOW  = 0x1C;

    // Burn commands
    static const uint8_t BURN           = 0xFF;
};

class AS5600 : public I2CDevice, protected I2CRegisters_8Bit
{
public:
    void init() override;

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CRegisters_8Bit::rdwr(data);
    }

private:

};

} // namespace SITL

#endif  // AP_SIM_AS5600_ENABLED
