#include "SIM_I2CDevice.h"

namespace SITL {

class ToshibaLEDDevReg : public I2CRegEnum {
public:
    static constexpr uint8_t PWM0 = 0x01;
    static constexpr uint8_t PWM1 = 0x02;
    static constexpr uint8_t PWM2 = 0x03;
    static constexpr uint8_t ENABLE = 0x04;
};

class ToshibaLED : public I2CDevice, protected I2CRegisters_8Bit
{
public:
    void init() override {
        add_register("PWM0", ToshibaLEDDevReg::PWM0, O_WRONLY);
        add_register("PWM1", ToshibaLEDDevReg::PWM1, O_WRONLY);
        add_register("PWM2", ToshibaLEDDevReg::PWM2, O_WRONLY);
        add_register("ENABLE", ToshibaLEDDevReg::ENABLE, O_WRONLY);
    }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CRegisters_8Bit::rdwr(data);
    }

private:
    uint8_t last_print_pwm0;
    uint8_t last_print_pwm1;
    uint8_t last_print_pwm2;
    uint8_t last_print_enable;
};

} // namespace SITL
