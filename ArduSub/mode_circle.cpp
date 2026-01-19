#include "Sub.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    if (!sub.position_ok()) {
        return false;
    }

    sub.circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    // All limits must be positive
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->NE_set_correction_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    sub.circle_nav.init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // update parameters, to allow changing at runtime
    // All limits must be positive
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: add some initialisation of position controllers
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.circle_nav.init();
        return;
    }

    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        sub.circle_pilot_yaw_override = true;
    }

    // get pilot desired climb rate
    target_climb_rate = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    sub.failsafe_terrain_set_status(sub.circle_nav.update_cms());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.circle_pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw_cd(), true);
    }

    // update altitude target and call position controller
    position_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate);
    position_control->D_update_controller();
}
