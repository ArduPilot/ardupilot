#include "Sub.h"

#include <AP_RCProtocol/AP_RCProtocol.h>

void Sub::init_rc_in()
{
    channel_roll     = &rc().get_roll_channel();
    channel_pitch    = &rc().get_pitch_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_yaw      = &rc().get_yaw_channel();
    channel_forward  = &rc().get_forward_channel();
    channel_lateral  = &rc().get_lateral_channel();

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_throttle->set_range(1000);
    channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    // set default dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
    channel_forward->set_default_dead_zone(30);
    channel_lateral->set_default_dead_zone(30);

    // initialize rc input to 1500 on control channels (rather than 0)
    uint32_t tnow = AP_HAL::millis();
    channel_roll->set_override(1500, tnow);
    channel_pitch->set_override(1500, tnow);
    channel_yaw->set_override(1500, tnow);
    channel_throttle->set_override(1500, tnow);
    channel_forward->set_override(1500, tnow);
    channel_lateral->set_override(1500, tnow);

#if HAL_MOUNT_ENABLED
    // initialize camera mount RC inputs to centered
    RC_Channel *cam_pan_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOUNT1_YAW);
    if (cam_pan_chan != nullptr) {
        cam_pan_chan->set_override(1500, tnow);
    }
    RC_Channel *cam_tilt_chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOUNT1_PITCH);
    if (cam_tilt_chan != nullptr) {
        cam_tilt_chan->set_override(1500, tnow);
    }
#endif  // HAL_MOUNT_ENABLED
}

// init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Sub::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), AP_Motors::motor_frame_type::MOTOR_FRAME_TYPE_PLUS);
    motors.convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    motors.update_throttle_range();

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();
}
#if AP_SUB_RC_ENABLED
void Sub::read_radio()
{ 
    const uint32_t tnow_ms = AP_HAL::millis();

#if AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED
    // update in case failsafe values have changed:
    if (g.failsafe_throttle == FS_THR_DISABLED) {
        // set flag values to disable filtering in AP_RCProtocol:
        AP::RC().set_throttle_channel_and_failsafe_value(UINT8_MAX, UINT16_MAX);
    } else {
        AP::RC().set_throttle_channel_and_failsafe_value(
            channel_throttle->ch(),
            g.failsafe_throttle_value
        );
    }
#endif  // AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED

    if (rc().read_input()) {
        if (rc().input_in_failsafe()) {
            // a failsafe has been identified in the input itself.  The
            // values remain available (e.g. for reporting), but are
            // not to be used to fly the vehicle:
            set_failsafe_radio(true);
            return;
        }
        if (rc().input_valid()) {
            //got valid input
            last_radio_update_ms = tnow_ms;
            failsafe.last_pilot_input_ms = tnow_ms;
            set_failsafe_radio(false);
            return;
        }
        // the input cannot be flown on; handle it as no input at all,
        // so that RC_FS_TIMEOUT applies
    }
    
   // No radio input this time
    if (failsafe.radio) {
        // already in failsafe! no further action
        return;
    }

    // trigger failsafe if no update from the RC Radio for RC_FS_TIMEOUT seconds
    const uint32_t elapsed_ms = tnow_ms - last_radio_update_ms;
    if (elapsed_ms < rc().get_fs_timeout_ms()) {
        // not timed out yet
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled
        return;
    }
    if (!rc().has_ever_seen_rc_input() && !sub.motors.armed()) {
        // we only failsafe if we are armed OR we have ever seen an RC receiver
        return;
    }
    
    // Log an error and enter failsafe.
    LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

#endif
