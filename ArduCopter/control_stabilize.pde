/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// acro_init - initialise acro controller
static bool acro_init(bool ignore_checks)
{
    return true;
}

// acro_run - runs the acro controller
// should be called at 100hz or more
static void acro_run()
{
    Vector3f rate_target;          // for roll, pitch, yaw body-frame rate targets

    // convert the input to the desired body frame rate
    rate_target.x = g.rc_1.control_in * g.acro_rp_p;
    rate_target.y = g.rc_2.control_in * g.acro_rp_p;
    rate_target.z = g.rc_4.control_in * g.acro_yaw_p;

    // To-Do: handle acro trainer here?

    // To-Do: handle helicopter

    acro_level_mix = constrain_float(1-max(max(abs(g.rc_1.control_in), abs(g.rc_2.control_in)), abs(g.rc_4.control_in))/4500.0, 0, 1)*cos_pitch_x;

    // set targets for body frame rate controller
    attitude_control.rate_stab_bf_targets(rate_target);

    // convert stabilize rates to regular rates
    attitude_control.rate_stab_bf_to_rate_bf_roll();
    attitude_control.rate_stab_bf_to_rate_bf_pitch();
    attitude_control.rate_stab_bf_to_rate_bf_yaw();

    // call get_acro_level_rates() here?

    // To-Do: convert body-frame stabilized angles to earth frame angles and update controll_roll, pitch and yaw?

    // body-frame rate controller is run directly from 100hz loop
}

// stabilize_init - initialise stabilize controller
static bool stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void stabilize_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.init_targets();
    }else{
        // call attitude controller
        attitude_control.angleef_rp_rateef_y(target_roll, target_pitch, target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop
    }

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);

    // refetch angle targets for reporting
    const Vector3f angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;

    // update estimate of throttle cruise
    #if FRAME_CONFIG == HELI_FRAME
    update_throttle_cruise(motors.get_collective_out());
    #else
    update_throttle_cruise(pilot_throttle_scaled);
    #endif  //HELI_FRAME

    // update take-off complete flag
    if (!ap.takeoff_complete) {
        if (pilot_throttle_scaled > g.throttle_cruise) {
            // we must be in the air by now
            set_takeoff_complete(true);
        }
    }
}

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
        attitude_control.angleef_rp_rateef_y(target_roll, target_pitch, target_yaw_rate);
        // body-frame rate controller is run directly from 100hz loop

        // call throttle controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(target_climb_rate);
        }else{
            // if no sonar fall back stabilize rate controller
            pos_control.climb_at_rate(target_climb_rate);
        }
    }

    // refetch angle targets for reporting
    const Vector3f angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;
}

// auto_init - initialise auto controller
static bool auto_init(bool ignore_checks)
{
    return true;
}

// auto_run - runs the auto controller
// should be called at 100hz or more
static void auto_run()
{
    Vector3f angle_target;

    // copy latest output from nav controller to stabilize controller
    control_roll = wp_nav.get_desired_roll();
    control_pitch = wp_nav.get_desired_pitch();

    // copy angle targets for reporting purposes
    angle_target.x = control_roll;
    angle_target.y = control_pitch;

    // To-Do: handle pilot input for yaw and different methods to update yaw (ROI, face next wp)
    angle_target.z = control_yaw;

    // To-Do: shorten below by moving these often used steps into a single function in the AC_AttitudeControl lib

    // set earth-frame angular targets
    attitude_control.angle_ef_targets(angle_target);

    // convert earth-frame angle targets to earth-frame rate targets
    attitude_control.angle_to_rate_ef_roll();
    attitude_control.angle_to_rate_ef_pitch();
    attitude_control.angle_to_rate_ef_yaw();

    // convert earth-frame rates to body-frame rates
    attitude_control.rate_ef_targets_to_bf();

    // body-frame rate controller is run directly from 100hz loop
}

// circle_init - initialise circle controller
static bool circle_init(bool ignore_checks)
{
    return true;
}

// circle_run - runs the circle controller
// should be called at 100hz or more
static void circle_run()
{
}

// loiter_init - initialise loiter controller
static bool loiter_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        // set target to current position
        // To-Do: supply zero velocity below?
        wp_nav.init_loiter_target();
        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void loiter_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || !inertial_nav.position_ok()) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // process pilot's roll and pitch input
        // To-Do: do we need to clear out feed forward if this is not called?
        wp_nav.move_loiter_target(g.rc_1.control_in, g.rc_2.control_in, G_Dt);

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
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
    }else{
        // run loiter controller
        wp_nav.update_loiter();

        // call attitude controller
        attitude_control.angleef_rp_rateef_y(wp_nav.get_desired_roll(), wp_nav.get_desired_pitch(), target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop

        // run altitude controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(target_climb_rate);
        }else{
            // if no sonar fall back stabilize rate controller
            pos_control.climb_at_rate(target_climb_rate);
        }
    }

    // refetch angle targets for reporting
    const Vector3f angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;
}

// guided_init - initialise guided controller
static bool guided_init(bool ignore_checks)
{
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
static void guided_run()
{
}

// land_init - initialise land controller
static bool land_init(bool ignore_checks)
{
    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
static void land_run()
{
    verify_land();
}

// rtl_init - initialise rtl controller
static bool rtl_init(bool ignore_checks)
{
    return true;
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
static void rtl_run()
{
    verify_RTL();
}

// ofloiter_init - initialise ofloiter controller
static bool ofloiter_init(bool ignore_checks)
{
    return true;
}

// ofloiter_run - runs the optical flow loiter controller
// should be called at 100hz or more
static void ofloiter_run()
{
}

// drift_init - initialise drift controller
static bool drift_init(bool ignore_checks)
{
    return true;
}

// drift_run - runs the drift controller
// should be called at 100hz or more
static void drift_run()
{
}

// sport_init - initialise sport controller
static bool sport_init(bool ignore_checks)
{
    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
static void sport_run()
{
}
