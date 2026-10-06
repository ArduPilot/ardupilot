#include "SIM_config.h"

#if AP_SIM_AS5600_ENABLED

#include "SIM_AS5600.h"

#include <stdio.h>

void SITL::AS5600::init()
{
    add_register("ZMCO", AS5600DevReg::ZMCO, I2CRegisters::RegMode::RDONLY);
    set_register(AS5600DevReg::ZMCO, (uint8_t)1U);

    add_register("RAH", AS5600DevReg::RAW_ANGLE_HIGH, I2CRegisters::RegMode::RDONLY);
    set_register(AS5600DevReg::RAW_ANGLE_HIGH, (uint8_t)0U);
    add_register("RAL", AS5600DevReg::RAW_ANGLE_LOW, I2CRegisters::RegMode::RDONLY);
    set_register(AS5600DevReg::RAW_ANGLE_LOW, (uint8_t)0U);
}

void SITL::AS5600::update(const class Aircraft &aircraft)
{

    // gcs().send_text(MAV_SEVERITY_INFO, "SIM_AS5600: PWM0=%u PWM1=%u PWM2=%u ENABLE=%u", last_print_pwm0, last_print_pwm1, last_print_pwm2, last_print_enable);
    Quaternion attitude;
    aircraft.get_attitude(attitude);
    const float pitch = degrees(attitude.get_euler_pitch());
    uint16_t pitch_180 = pitch + 90;

    set_register(AS5600DevReg::RAW_ANGLE_HIGH, uint8_t(pitch_180 >> 8));
    set_register(AS5600DevReg::RAW_ANGLE_LOW, uint8_t(pitch_180 & 0xff));
}

#endif
