#include "Sub.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Sub::circle_init()
{
    if (!position_ok()) {
        return false;
    }

    circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    circle_nav.init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Sub::circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // update parameters, to allow changing at runtime
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: add some initialisation of position controllers
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        circle_nav.init();
        return;
    }

    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        circle_pilot_yaw_override = true;
    }

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    failsafe_terrain_set_status(circle_nav.update());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        attitude_control.input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), circle_nav.get_yaw(), true);
    }

    // update altitude target and call position controller
    pos_control.set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    pos_control.update_z_controller();
}
