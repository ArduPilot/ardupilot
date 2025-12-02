#include "Rover.h"

/*
  allow for runtime change of control channel ordering
 */
void Rover::set_control_channels(void)
{
    // check change on RCMAP
    // the library guarantees that these are non-nullptr:
    channel_steer    = &rc().get_roll_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_lateral  = &rc().get_yaw_channel();

    // set rc channel ranges
    channel_steer->set_angle(SERVO_MAX);
    channel_throttle->set_angle(100);
    if (channel_lateral != nullptr) {
        channel_lateral->set_angle(100);
    }

    // walking robots rc input init
    channel_roll = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ROLL);
    channel_pitch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::PITCH);
    channel_walking_height = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WALKING_HEIGHT);
    if (channel_roll != nullptr) {
        channel_roll->set_angle(SERVO_MAX);
        channel_roll->set_default_dead_zone(30);
    }
    if (channel_pitch != nullptr) {
        channel_pitch->set_angle(SERVO_MAX);
        channel_pitch->set_default_dead_zone(30);
    }
    if (channel_walking_height != nullptr) {
        channel_walking_height->set_angle(SERVO_MAX);
        channel_walking_height->set_default_dead_zone(30);
    }    

    // sailboat rc input init
    g2.sailboat.init_rc_in();

    // Allow to reconfigure output when not armed
    if (!arming.is_armed()) {
        g2.motors.setup_servo_output();
        // For a rover safety is TRIM throttle
        g2.motors.setup_safety_output();
    }
    // setup correct scaling for ESCs like the UAVCAN ESCs which
    // take a proportion of speed. Default to 1000 to 2000 for systems without
    // a k_throttle output
    hal.rcout->set_esc_scaling(1000, 2000);
    g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
}

void Rover::init_rc_in()
{
    // set rc dead zones
    channel_steer->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    if (channel_lateral != nullptr) {
        channel_lateral->set_default_dead_zone(30);
    }
}

void Rover::read_radio()
{
    if (!rc().read_input()) {
        // check if we lost RC link
        radio_failsafe_check(channel_throttle->get_radio_in());
        return;
    }

    failsafe.last_valid_rc_ms = AP_HAL::millis();
    // check that RC value are valid
    radio_failsafe_check(channel_throttle->get_radio_in());
}

void Rover::radio_failsafe_check(uint16_t pwm)
{
    if (!g.fs_throttle_enabled) {
        // radio failsafe disabled
        AP_Notify::flags.failsafe_radio = false;
        return;
    }

    bool failed = pwm < static_cast<uint16_t>(g.fs_throttle_value);
    if (AP_HAL::millis() - failsafe.last_valid_rc_ms > rc().get_fs_timeout_ms()) {
        // we haven't had a valid RC frame for RC_FS_TIMEOUT seconds
        failed = true;
    }
    AP_Notify::flags.failsafe_radio = failed;
    failsafe_trigger(FAILSAFE_EVENT_THROTTLE, "Radio", failed);
}
