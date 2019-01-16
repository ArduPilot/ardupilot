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
    pos_control.set_max_speed_xy(wp_nav.get_speed_xy());
    pos_control.set_max_accel_xy(wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

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
    pos_control.set_max_speed_xy(wp_nav.get_speed_xy());
    pos_control.set_max_accel_xy(wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: add some initialisation of position controllers
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        pos_control.set_alt_target_to_current_alt();
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
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run circle controller
    circle_nav.update();

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

    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();
}
