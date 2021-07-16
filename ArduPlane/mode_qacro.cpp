#include "mode.h"
#include "Plane.h"

bool ModeQAcro::_enter()
{
    plane.quadplane.throttle_wait = false;
    plane.quadplane.transition_state = QuadPlane::TRANSITION_DONE;
    plane.quadplane.attitude_control->relax_attitude_controllers();

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

void ModeQAcro::run()
{
    if (plane.quadplane.throttle_wait) {
        plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        plane.quadplane.attitude_control->set_throttle_out(0, true, 0);
        plane.quadplane.relax_attitude_control();
        return;
    }

    plane.quadplane.check_attitude_relax();

    plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert the input to the desired body frame rate
    float target_roll = 0;
    float target_pitch = plane.channel_pitch->norm_input() * plane.quadplane.acro_pitch_rate * 100.0;
    float target_yaw = 0;
    if (plane.quadplane.is_tailsitter()) {
        // Note that the 90 degree Y rotation for copter mode swaps body-frame roll and yaw
        target_roll =  plane.channel_rudder->norm_input() * plane.quadplane.acro_yaw_rate * 100.0;
        target_yaw  = -plane.channel_roll->norm_input() * plane.quadplane.acro_roll_rate * 100.0;
    } else {
        target_roll = plane.channel_roll->norm_input() * plane.quadplane.acro_roll_rate * 100.0;
        target_yaw  = plane.channel_rudder->norm_input() * plane.quadplane.acro_yaw_rate * 100.0;
    }

    float throttle_out = plane.quadplane.get_pilot_throttle();

    // run attitude controller
    if (plane.g.acro_locking) {
        plane.quadplane.attitude_control->input_rate_bf_roll_pitch_yaw_3(target_roll, target_pitch, target_yaw);
    } else {
        plane.quadplane.attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
    }

    // output pilot's throttle without angle boost
    plane.quadplane.attitude_control->set_throttle_out(throttle_out, false, 10.0);
}
