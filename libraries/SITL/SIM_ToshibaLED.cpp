#include "SIM_ToshibaLED.h"

#if AP_SIM_TOSHIBALED_ENABLED

#include <stdio.h>

void SITL::ToshibaLED::update(const class Aircraft &aircraft)
{
    if (last_print_pwm0 == get_register(ToshibaLEDDevReg::PWM0) &&
        last_print_pwm1 == get_register(ToshibaLEDDevReg::PWM1) &&
        last_print_pwm2 == get_register(ToshibaLEDDevReg::PWM2) &&
        last_print_enable == get_register(ToshibaLEDDevReg::ENABLE)) {
        return;
    }

    last_print_pwm0 = get_register(ToshibaLEDDevReg::PWM0);
    last_print_pwm1 = get_register(ToshibaLEDDevReg::PWM1);
    last_print_pwm2 = get_register(ToshibaLEDDevReg::PWM2);
    last_print_enable = get_register(ToshibaLEDDevReg::ENABLE);
    // gcs().send_text(MAV_SEVERITY_INFO, "SIM_ToshibaLED: PWM0=%u PWM1=%u PWM2=%u ENABLE=%u", last_print_pwm0, last_print_pwm1, last_print_pwm2, last_print_enable);

    if (get_register(ToshibaLEDDevReg::ENABLE)) {
        // here we convert from 0-15 BGR (the PWM values from the i2c bus)
        // to 0-255 RGB (what SIM_RGBLED wants):
        rgbled.set_colours(
            get_register(ToshibaLEDDevReg::PWM2) * 17,
            get_register(ToshibaLEDDevReg::PWM1) * 17,
            get_register(ToshibaLEDDevReg::PWM0) * 17
            );
    } else {
        rgbled.set_colours(0, 0, 0);
    }
}

#endif
