#include <AP_HAL/AP_HAL_Boards.h>

/*
 * To test in SITL:
 *
 * ./Tools/autotest/sim_vehicle.py -v ArduCopter --rgbled
 */

#ifndef AP_SIM_LM2755_ENABLED
#define AP_SIM_LM2755_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_SIM_LM2755_ENABLED

#include "SIM_I2CDevice.h"
#include "SIM_RGBLED.h"

namespace SITL {

class LM2755DevReg : public I2CRegEnum {
public:
    static constexpr uint8_t GENERAL_PURPOSE = 0x10;
    static constexpr uint8_t TIME_STEP = 0x20;

    static constexpr uint8_t D1_HIGH_LEVEL = 0xA9;
    static constexpr uint8_t D1_LOW_LEVEL = 0xA8;
    static constexpr uint8_t D1_DELAY = 0xA1;
    static constexpr uint8_t D1_RAMP_UP_STEP_TIME = 0xA5;
    static constexpr uint8_t D1_TIME_HIGH = 0xA3;
    static constexpr uint8_t D1_RAMP_DOWN_STEP_TIME = 0xA4;
    static constexpr uint8_t D1_TIMING = 0xA2;

    static constexpr uint8_t D2_HIGH_LEVEL = 0xB9;
    static constexpr uint8_t D2_LOW_LEVEL = 0xB8;
    static constexpr uint8_t D2_DELAY = 0xB1;
    static constexpr uint8_t D2_RAMP_UP_STEP_TIME = 0xB5;
    static constexpr uint8_t D2_TIME_HIGH = 0xB3;
    static constexpr uint8_t D2_RAMP_DOWN_STEP_TIME = 0xB4;
    static constexpr uint8_t D2_TIMING = 0xB2;

    static constexpr uint8_t D3_HIGH_LEVEL = 0xC9;
    static constexpr uint8_t D3_LOW_LEVEL = 0xC8;
    static constexpr uint8_t D3_DELAY = 0xC1;
    static constexpr uint8_t D3_RAMP_UP_STEP_TIME = 0xC5;
    static constexpr uint8_t D3_TIME_HIGH = 0xC3;
    static constexpr uint8_t D3_RAMP_DOWN_STEP_TIME = 0xC4;
    static constexpr uint8_t D3_TIMING = 0xC2;
};

class LM2755 : public I2CDevice, protected I2CRegisters_8Bit
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
        LEDChannel(uint8_t &_high_level,
                   uint8_t &_low_level,
                   uint8_t &_delay,
                   uint8_t &_ramp_up_step_time,
                   uint8_t &_time_high,
                   uint8_t &_ramp_down_step_time,
                   uint8_t &_timing) :
            high_level{_high_level},
            low_level{_low_level}
            { }

        void update();

        // returns a value 0-255 for LED brightness:
        uint8_t current_value() const { return output_value; }

    private:
        uint8_t &high_level;
        uint8_t &low_level;

        uint8_t output_value;
    };

    // these could come in as rgb or bgr?
    LEDChannel *d1;
    LEDChannel *d2;
    LEDChannel *d3;

    SIM_RGBLED rgbled{"LM2755"};
};

} // namespace SITL

#endif  // AP_SIM_LM2755_ENABLED
