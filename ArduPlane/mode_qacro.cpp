#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQAcro::_enter()
{
    quadplane.throttle_wait = false;
    quadplane.transition->force_transition_complete();
    attitude_control->relax_attitude_controllers();

    // disable yaw rate time contant to mantain old behaviour
    quadplane.disable_yaw_rate_time_constant();

    IGNORE_RETURN(plane.ahrs.get_quaternion(plane.acro_state.q));

    return true;
}

void ModeQAcro::update()
{
    // get nav_roll and nav_pitch from multicopter attitude controller
    Vector3f att_target = plane.quadplane.attitude_control->get_att_target_euler_cd();
    plane.nav_pitch_cd = att_target.y;
    plane.nav_roll_cd = att_target.x;
    return;
}


/*
  control QACRO mode
 */
void ModeQAcro::run()
{
    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
    } else {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // convert the input to the desired body frame rate
        float target_roll = 0;
        float target_pitch = plane.channel_pitch->norm_input() * quadplane.acro_pitch_rate * 100.0f;
        float target_yaw = 0;
        if (quadplane.tailsitter.enabled()) {
            // Note that the 90 degree Y rotation for copter mode swaps body-frame roll and yaw
            target_roll =  plane.channel_rudder->norm_input() * quadplane.acro_yaw_rate * 100.0f;
            target_yaw  = -plane.channel_roll->norm_input() * quadplane.acro_roll_rate * 100.0f;
        } else {
            target_roll = plane.channel_roll->norm_input() * quadplane.acro_roll_rate * 100.0f;
            target_yaw  = plane.channel_rudder->norm_input() * quadplane.acro_yaw_rate * 100.0;
        }

        float throttle_out = quadplane.get_pilot_throttle();

        // run attitude controller
        if (plane.g.acro_locking) {
            attitude_control->input_rate_bf_roll_pitch_yaw_3(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        }

        // output pilot's throttle without angle boost
        attitude_control->set_throttle_out(throttle_out, false, 10.0f);
    }
}

#endif
