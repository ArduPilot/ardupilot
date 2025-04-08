#include "Sub.h"


bool ModeSurface::init(bool ignore_checks)
{
    if(!sub.control_check_barometer()) {
        return false;
    }

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_z_controller();

    return true;

}

void ModeSurface::run()
{
    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->init_z_controller();
        return;
    }

    // Already at surface, hold depth at surface
    if (sub.ap.at_surface) {
        set_mode(Mode::Number::ALT_HOLD, ModeReason::SURFACE_COMPLETE);
    }

    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // set target climb rate
    float cmb_rate = constrain_float(fabsf(sub.wp_nav.get_default_speed_up()), 1, position_control->get_max_speed_up_cms());

    // update altitude target and call position controller
    position_control->set_pos_target_z_from_climb_rate_cm(cmb_rate);
    position_control->update_z_controller();

    // pilot has control for repositioning
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
