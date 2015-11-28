/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_brake.pde - init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool Copter::brake_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {

        // set desired acceleration to zero
        wp_nav.clear_pilot_desired_acceleration();

        // set target to current position
        wp_nav.init_brake_target(BRAKE_MODE_DECEL_RATE);

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
        pos_control.set_accel_z(BRAKE_MODE_DECEL_RATE);

        // initialise position and desired velocity
        pos_control.set_alt_target(inertial_nav.get_altitude());
        pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

        return true;
    }else{
        return false;
    }
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void Copter::brake_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        wp_nav.init_brake_target(BRAKE_MODE_DECEL_RATE);
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(0)-throttle_average);
        return;
    }

    // relax stop target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // if landed immediately disarm
    if (ap.land_complete) {
        init_disarm_motors();
    }

    // run brake controller
    wp_nav.update_brake(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), 0.0f);

    // body-frame rate controller is run directly from 100hz loop

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
    pos_control.update_z_controller();
}
