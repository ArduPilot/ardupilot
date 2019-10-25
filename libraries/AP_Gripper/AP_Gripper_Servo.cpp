#include <AP_Gripper/AP_Gripper_Servo.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #include <SITL/SITL.h>
#endif

extern const AP_HAL::HAL& hal;

void AP_Gripper_Servo::init_gripper()
{
    // move the servo to the release position
    release();
}

void AP_Gripper_Servo::grab()
{
    // move the servo to the grab position
    SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, config.grab_pwm);
    action_timestamp = AP_HAL::millis();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    is_releasing = false;
    is_released = true;
#endif
    gcs().send_text(MAV_SEVERITY_INFO, "Gripper load grabbing");
    AP::logger().Write_Event(LogEvent::GRIPPER_GRAB);
}

void AP_Gripper_Servo::release()
{
    // move the servo to the release position
    SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, config.release_pwm);
    action_timestamp = AP_HAL::millis();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    is_releasing = true;
    is_released = false;
#endif
    gcs().send_text(MAV_SEVERITY_INFO, "Gripper load releasing");
    AP::logger().Write_Event(LogEvent::GRIPPER_RELEASE);
}

bool AP_Gripper_Servo::has_state_pwm(const uint16_t pwm) const
{
    // return true if servo is in position represented by pwm
    uint16_t current_pwm;
    if (!SRV_Channels::get_output_pwm(SRV_Channel::k_gripper, current_pwm)) {
        // function not assigned to a channel, perhaps?
        return false;
    }
    if (current_pwm != pwm) {
        // last action did not set pwm to the current value
        // (e.g. last action was a grab not a release)
        return false;
    }
    if (AP_HAL::millis() - action_timestamp < action_time) {
        // servo still moving....
        return false;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (is_releasing) {
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper load released");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper load grabbed");
    }
#endif
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
void AP_Gripper_Servo::update_gripper() {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (is_releasing && !is_released) {
        is_released = released();
    } else if (!is_releasing && is_released) {
        is_released = !grabbed();
    }
#endif
};

bool AP_Gripper_Servo::valid() const
{
    if (!AP_Gripper_Backend::valid()) {
        return false;
    }
    if (!SRV_Channels::function_assigned(SRV_Channel::k_gripper)) {
        return false;
    }
    return true;
}
