/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool Copter::sport_init(bool ignore_checks)
{
    // initialize vertical speed and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void Copter::sport_run()
{
    SportModeState sport_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform
    update_simple_mode();

    // get pilot's desired roll and pitch rates

    // calculate rate requests
    float target_roll_rate_rads = radians(channel_roll->get_control_in() * g.acro_rp_p * 0.01f);
    float target_pitch_rate_rads = radians(channel_pitch->get_control_in() * g.acro_rp_p * 0.01f);

    float roll_angle_rad = wrap_PI(ahrs.roll);
    target_roll_rate_rads -= constrain_float(roll_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    float pitch_angle_rad = wrap_PI(ahrs.pitch);
    target_pitch_rate_rads -= constrain_float(pitch_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_pitch;

    float angle_max_rad = radians(aparm.angle_max*0.01f);
    if (roll_angle_rad > angle_max_rad){
        target_roll_rate_rads -=  g.acro_rp_p*(roll_angle_rad-angle_max_rad);
    }else if (roll_angle_rad < -angle_max_rad) {
        target_roll_rate_rads -=  g.acro_rp_p*(roll_angle_rad+angle_max_rad);
    }

    if (pitch_angle_rad > angle_max_rad){
        target_pitch_rate_rads -=  g.acro_rp_p*(pitch_angle_rad-angle_max_rad);
    }else if (pitch_angle_rad < -angle_max_rad) {
        target_pitch_rate_rads -=  g.acro_rp_p*(pitch_angle_rad+angle_max_rad);
    }

    // get pilot's desired yaw rate
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rad(radians(channel_yaw->get_control_in()*0.01f));

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        sport_state = Sport_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        sport_state = Sport_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        sport_state = Sport_Landed;
    } else {
        sport_state = Sport_Flying;
    }

    // State Machine
    switch (sport_state) {

    case Sport_MotorStopped:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control.input_euler_rate_roll_pitch_yaw_rad(target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads);
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control.relax_attitude_controllers();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control.update_z_controller();
        break;

    case Sport_Takeoff:
        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // call attitude controller
        attitude_control.input_euler_rate_roll_pitch_yaw_rad(target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads);

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case Sport_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_rate_roll_pitch_yaw_rad(target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads);
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control.update_z_controller();
        break;

    case Sport_Flying:
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // call attitude controller
        attitude_control.input_euler_rate_roll_pitch_yaw_rad(target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads);

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }
}
