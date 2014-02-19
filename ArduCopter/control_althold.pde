/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
static bool althold_init(bool ignore_checks)
{
    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
static void althold_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset altitude target if we're somehow not landed?
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.init_targets();
        // move throttle to minimum to keep us on the ground
        attitude_control.set_throttle_out(0, false);
    }else{
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        // body-frame rate controller is run directly from 100hz loop

        // call throttle controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}
