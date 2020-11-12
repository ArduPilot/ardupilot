#include "SIM_ToshibaLED.h"

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
}
