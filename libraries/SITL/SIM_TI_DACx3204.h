#pragma once

// simulator for Texas Instruments DACx3204 devices

// TODO: support broadcast mode (how?!!?!?!)

#include "SIM_I2CDevice.h"

namespace SITL {

class TI_DACx3204RegEnum : public I2CRegEnum {
public:
    static constexpr uint8_t NOP                    = 0x00;
    static constexpr uint8_t DAC_0_MARGIN_HIGH      = 0x01;
    static constexpr uint8_t DAC_0_MARGIN_LOW       = 0x02;
    static constexpr uint8_t DAC_0_VOUT_CMP_CONFIG  = 0x03;
    static constexpr uint8_t DAC_0_IOUT_MISC_CONFIG = 0x04;
    static constexpr uint8_t DAC_0_CMP_MODE_CONFIG = 0x05;
    static constexpr uint8_t DAC_0_FUNC_CONFIG      = 0x06;

    static constexpr uint8_t DAC_1_MARGIN_HIGH      = 0x07;
    static constexpr uint8_t DAC_1_MARGIN_LOW       = 0x08;
    static constexpr uint8_t DAC_1_VOUT_CMP_CONFIG  = 0x09;
    static constexpr uint8_t DAC_1_IOUT_MISC_CONFIG = 0x0A;
    static constexpr uint8_t DAC_1_CMP_MODE_CONFIG = 0x0B;
    static constexpr uint8_t DAC_1_FUNC_CONFIG      = 0x0C;

    static constexpr uint8_t DAC_2_MARGIN_HIGH      = 0x0D;
    static constexpr uint8_t DAC_2_MARGIN_LOW       = 0x0E;
    static constexpr uint8_t DAC_2_VOUT_CMP_CONFIG  = 0x0F;
    static constexpr uint8_t DAC_2_IOUT_MISC_CONFIG = 0x10;
    static constexpr uint8_t DAC_2_CMP_MODE_CONFIG = 0x11;
    static constexpr uint8_t DAC_2_FUNC_CONFIG      = 0x12;

    static constexpr uint8_t DAC_3_MARGIN_HIGH      = 0x13;
    static constexpr uint8_t DAC_3_MARGIN_LOW       = 0x14;
    static constexpr uint8_t DAC_3_VOUT_CMP_CONFIG  = 0x15;
    static constexpr uint8_t DAC_3_IOUT_MISC_CONFIG = 0x16;
    static constexpr uint8_t DAC_3_CMP_MODE_CONFIG = 0x17;
    static constexpr uint8_t DAC_3_FUNC_CONFIG      = 0x18;

    static constexpr uint8_t DAC_0_DATA             = 0x19;
    static constexpr uint8_t DAC_1_DATA             = 0x1A;
    static constexpr uint8_t DAC_2_DATA             = 0x1B;
    static constexpr uint8_t DAC_3_DATA             = 0x1C;

    static constexpr uint8_t COMMON_CONFIG          = 0x1F;
    static constexpr uint8_t COMMON_TRIGGER         = 0x20;
    static constexpr uint8_t COMMON_DAC_TRIGGER     = 0x21;
};

class TI_DACx3204 : public I2CDevice, private I2CRegisters_16Bit {
public:

    TI_DACx3204();

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

protected:

    void set_register(uint8_t reg, uint16_t value) {
        I2CRegisters_16Bit::set_register(reg, value);
    }
    // void set_register(uint8_t reg, int16_t value) {
    //     I2CRegisters_16Bit::set_register(reg, value);
    // }

    uint16_t get_reg_value(uint8_t reg) {
        return I2CRegisters_16Bit::get_reg_value(reg);
    }

    void add_register(const char *name, uint8_t reg, I2CRegisters::RegMode mode) {
        I2CRegisters_16Bit::add_register(name, reg, mode);
    }

private:

    // do a power-on-reset:
    void reset();

    void assert_register_write_ok(I2C::i2c_rdwr_ioctl_data *&data);
    bool registers_are_locked;

    const uint16_t common_trigger_reset_bits_mask { 0b1111 << 8 };
    const uint16_t common_trigger_reset_bits_reset_value { 0b1010 };

    class DAC {
    public:
        DAC(uint16_t &margin_high,
            uint16_t &margin_low,
            uint16_t &vout_cmp_config,
            uint16_t &vout_misc_config,
            uint16_t &vout_mode_config,
            uint16_t &vout_func_config);

        void reset();
        void update();

    private:
        uint16_t &margin_high;
        uint16_t &margin_low;
        uint16_t &vout_cmp_config;
        uint16_t &vout_misc_config;
        uint16_t &vout_mode_config;
        uint16_t &vout_func_config;
    };

    DAC *dacs[4];
};

} // namespace SITL
