#include <AP_Gripper/AP_Gripper_Servo.h>

extern const AP_HAL::HAL& hal;

void AP_Gripper_Servo::init_gripper()
{
    // move the servo to the release position
    release();
}

void AP_Gripper_Servo::grab()
{
    // move the servo to the grab position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_gripper, config.grab_pwm);
    action_timestamp = AP_HAL::millis();
}

void AP_Gripper_Servo::release()
{
    // move the servo to the release position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_gripper, config.release_pwm);
    action_timestamp = AP_HAL::millis();
}

bool AP_Gripper_Servo::has_state_pwm(const uint16_t pwm) const
{
    // return true if servo is in position represented by pwm
    int16_t current_pwm;
    if (!RC_Channel_aux::get_radio(RC_Channel_aux::k_gripper, current_pwm)) {
        // function not assigned to a channel, perhaps?
        return false;
    }
    if (current_pwm != pwm) {
        // last action did not set pwm to the current value
        // (e.g. last action was a grabm not a release)
        return false;
    }
    if (AP_HAL::millis() - action_timestamp < action_time) {
        // servo still moving....
        return false;
    }
    return true;
}


bool AP_Gripper_Servo::released() const
{
    return has_state_pwm(config.release_pwm);
}

bool AP_Gripper_Servo::grabbed() const
{
    return has_state_pwm(config.grab_pwm);
}

// type-specific periodic updates:
void AP_Gripper_Servo::update_gripper() { };
