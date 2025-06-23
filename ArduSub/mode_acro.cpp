#include "Sub.h"


bool ModeAcro::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_desired_U_cm(0);

    // attitude hold inputs become thrust inputs in acro mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    sub.set_neutral_controls();

    return true;
}

void ModeAcro::run()
{
    float target_roll, target_pitch, target_yaw;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), target_roll, target_pitch, target_yaw);

    // run attitude controller
    attitude_control->input_rate_bf_roll_pitch_yaw_cds(target_roll, target_pitch, target_yaw);

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range 0-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
