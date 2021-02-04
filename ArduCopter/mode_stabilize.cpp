#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    AP_Mount *mount = AP::mount();
    if ((mount != nullptr) && (mount->mount_yaw_follow_mode == AP_Mount::vehicle_yaw_follows_gimbal)) {
        float angle_deg;
        // allow the pilot to override the yaw_condition command if the commanded yaw is outside the deadband
        // the total range of target_yaw_rate is about -10000 to 10000,
        // set the deadband to 200/10000 or 2% of the total range,
        if ((target_yaw_rate > 200) || (target_yaw_rate < -200)) {
            angle_deg = target_yaw_rate/1000.0f;
        }
        else {
            angle_deg = mount->get_follow_yaw_rate();
        }
        int8_t direction = 1;
        bool relative_angle = true;
        if (angle_deg < 0.0f) {
            angle_deg = -angle_deg;
            direction = -1;
        }
        // update auto_yaw private variable _fixed_yaw
        copter.flightmode->auto_yaw.set_fixed_yaw(
            angle_deg,
            0, // use default angle change rate
            direction,
            relative_angle);
    }

    // call attitude controller
    if (auto_yaw.mode() != AUTO_YAW_FIXED) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    }
    else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, auto_yaw.yaw(), true);
    }

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
}
