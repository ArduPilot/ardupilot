#pragma once

/*

  ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug --rgbled

  param set NTF_LED_TYPES 8198  # enable ToshibaLED and LP5562
  reboot

  param set NTF_LED_OVERRIDE 1
  led 255 0 0   # red
  led 0 255 0   # green
  led 0 0 255   # blue

 */


#include "SIM_config.h"

#if AP_SIM_LP5562_ENABLED

#include "SIM_I2CDevice.h"
#include "SIM_RGBLED.h"

namespace SITL {

class LP5562DevReg : public I2CRegEnum {
public:
    static constexpr uint8_t ENABLE = 0x00;
    static constexpr uint8_t OP_MODE = 0x01;

    static constexpr uint8_t B_PWM = 0x02;
    static constexpr uint8_t G_PWM = 0x03;
    static constexpr uint8_t R_PWM = 0x04;

    static constexpr uint8_t B_CURRENT = 0x05;
    static constexpr uint8_t G_CURRENT = 0x06;
    static constexpr uint8_t R_CURRENT = 0x07;

    static constexpr uint8_t CONFIG = 0x08;

    static constexpr uint8_t ENG1_PC = 0x09;
    static constexpr uint8_t ENG2_PC = 0x0A;
    static constexpr uint8_t ENG3_PC = 0x0B;

    static constexpr uint8_t STATUS = 0x0C;

    static constexpr uint8_t RESET = 0x0D;

    static constexpr uint8_t W_PWM = 0x0E;
    static constexpr uint8_t W_CURRENT = 0x0F;

    static constexpr uint8_t LED_MAP = 0x70;

    static constexpr uint8_t PROG_MEM_ENG1_01H = 0x10;
    static constexpr uint8_t PROG_MEM_ENG1_01L = 0x11;
    static constexpr uint8_t PROG_MEM_ENG1_02H = 0x12;
    static constexpr uint8_t PROG_MEM_ENG1_02L = 0x13;
    // .
    // .
    static constexpr uint8_t PROG_MEM_ENG1_16H = 0x2E;
    static constexpr uint8_t PROG_MEM_ENG1_16L = 0x2F;

    static constexpr uint8_t PROG_MEM_ENG2_01H = 0x30;
    static constexpr uint8_t PROG_MEM_ENG2_01L = 0x31;
    static constexpr uint8_t PROG_MEM_ENG2_02H = 0x32;
    static constexpr uint8_t PROG_MEM_ENG2_02L = 0x43;
    // .
    // .
    static constexpr uint8_t PROG_MEM_ENG2_16H = 0x4E;
    static constexpr uint8_t PROG_MEM_ENG2_16L = 0x4F;

    static constexpr uint8_t PROG_MEM_ENG3_01H = 0x50;
    static constexpr uint8_t PROG_MEM_ENG3_01L = 0x51;
    static constexpr uint8_t PROG_MEM_ENG3_02H = 0x52;
    static constexpr uint8_t PROG_MEM_ENG3_02L = 0x53;
    // .
    // .
    static constexpr uint8_t PROG_MEM_ENG3_16H = 0x6E;
    static constexpr uint8_t PROG_MEM_ENG3_16L = 0x6F;
};

class LP5562 : public I2CDevice, protected I2CRegisters_8Bit
{
public:

    void init() override;

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CRegisters_8Bit::rdwr(data);
    }

private:

    // nested class to hold calculations for a single channel:
    class LEDChannel {
    public:
        LEDChannel(uint8_t &_direct_pwm_value) :
            direct_pwm_value{_direct_pwm_value}
            { }

        void update();

        // returns a value 0-255 for LED brightness:
        uint8_t current_value() const { return direct_pwm_value; }

    private:
        uint8_t &direct_pwm_value;
    };

    // these could come in as rgb or bgr?
    LEDChannel *b;
    LEDChannel *g;
    LEDChannel *r;

    void reset_registers();

    SIM_RGBLED rgbled{"LP5562"};
};

} // namespace SITL

#endif  // AP_SIM_LP5562_ENABLED
