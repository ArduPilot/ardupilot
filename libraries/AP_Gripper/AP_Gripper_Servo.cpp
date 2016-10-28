#include <AP_Gripper/AP_Gripper_Servo.h>

extern const AP_HAL::HAL& hal;

void AP_Gripper_Servo::init_gripper()
{
    // move the servo to the release position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_gripper, config.release_pwm);
}

void AP_Gripper_Servo::grab()
{
    // move the servo to the release position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_gripper, config.grab_pwm);
}

void AP_Gripper_Servo::release()
{
    // move the servo to the release position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_gripper, config.release_pwm);
}

    // type-specific periodic updates:
void AP_Gripper_Servo::update_gripper() { };
