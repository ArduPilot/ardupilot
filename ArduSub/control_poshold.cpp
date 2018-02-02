// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED

// poshold_init - initialise PosHold controller
bool Sub::poshold_init()
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok()) {
        return false;
    }

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // set target to current position
    // only init here as we can switch to PosHold in flight with a velocity <> 0 that will be used as _last_vel in PosControl and never updated again as we inhibit Reset_I
    wp_nav.init_loiter_target();

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Sub::poshold_run()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize smoothing gain
    attitude_control.set_smoothing_gain(get_smoothing_gain());

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    ///////////////////////
    // update xy outputs //
    float pilot_lateral = channel_lateral->norm_input();
    float pilot_forward = channel_forward->norm_input();

    float lateral_out = 0;
    float forward_out = 0;

    // Allow pilot to reposition the sub
    if (fabsf(pilot_lateral) > 0.1 || fabsf(pilot_forward) > 0.1) {
        lateral_out = pilot_lateral;
        forward_out = pilot_forward;
        wp_nav.init_loiter_target(); // initialize target to current position after repositioning
    } else {
        translate_wpnav_rp(lateral_out, forward_out);
    }

    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////
    // Update attitude //

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // update attitude controller targets
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
        }
    }

    ///////////////////
    // Update z axis //

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // adjust climb rate using rangefinder
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }

    // call z axis position controller
    if (ap.at_bottom) {
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover()); // clear velocity and position targets, and integrator
        pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
    } else {
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    }

    pos_control.update_z_controller();
}
#endif  // POSHOLD_ENABLED == ENABLED
