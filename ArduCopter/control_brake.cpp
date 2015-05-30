/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_brake.pde - init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
static bool brake_init(bool ignore_checks)
{
    if (position_ok() || optflow_position_ok() || ignore_checks) {

        // set desired acceleration to zero
        wp_nav.clear_pilot_desired_acceleration();

        // set target to current position
        wp_nav.init_brake_target(BRAKE_MODE_DECEL_RATE);

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(0, 0);
        pos_control.set_accel_z(BRAKE_MODE_DECEL_RATE);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        return true;
    }else{
        return false;
    }
}

// brake_run - runs the brake controller
// should be called at 100hz or more
static void brake_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        wp_nav.init_brake_target(BRAKE_MODE_DECEL_RATE);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(0)-throttle_average);
        return;
    }

    // relax stop target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // if landed immediately disarm
    if (ap.land_complete) {
        init_disarm_motors();
    }

    // run brake controller
    wp_nav.update_brake(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
    pos_control.update_z_controller();
}
