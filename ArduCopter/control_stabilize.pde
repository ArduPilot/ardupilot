/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// acro_init - initialise acro controller
static bool acro_init()
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
static bool stabilize_init()
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
    Vector3f angle_target = attitude_control.angle_ef_targets();     // for roll, pitch and yaw angular targets
    Vector3f rate_stab_ef_target;   // for yaw rate target.  Note Vector3f initialises all values to zero in constructor
    int16_t target_roll, target_pitch;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
    angle_target.x = target_roll;
    angle_target.y = target_pitch;

    // get pilot's desired yaw rate
    if (!failsafe.radio && !ap.land_complete) {
        rate_stab_ef_target.z = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // set target heading to current heading while landed
    if (ap.land_complete) {
        angle_target.z = ahrs.yaw_sensor;
    }

    // set earth-frame angular targets
    attitude_control.angle_ef_targets(angle_target);

    // convert earth-frame angle targets to earth-frame rate targets
    attitude_control.angle_to_rate_ef_roll();
    attitude_control.angle_to_rate_ef_pitch();

    // set earth-frame rate stabilize target for yaw with pilot's desired yaw
    // To-Do: this is quite wasteful to update the entire target vector when only yaw is used
    attitude_control.rate_stab_ef_targets(rate_stab_ef_target);

    // convert earth-frame stabilize rate to regular rate target
    // To-Do: replace G_Dt below
    attitude_control.rate_stab_ef_to_rate_ef_yaw();

    // convert earth-frame rates to body-frame rates
    attitude_control.rate_ef_targets_to_bf();

    // refetch angle targets for reporting
    angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;

    // body-frame rate controller is run directly from 100hz loop

    // do not run throttle controllers if motors disarmed
    if( !motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.set_throttle_out(0, false);
        set_target_alt_for_reporting(0);
    }else{
        // manual throttle but with angle boost
        int16_t pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
        attitude_control.set_throttle_out(pilot_throttle_scaled, true);

        // update estimate of throttle cruise
        #if FRAME_CONFIG == HELI_FRAME
        update_throttle_cruise(motors.get_collective_out());
        #else
        update_throttle_cruise(pilot_throttle_scaled);
        #endif  //HELI_FRAME

        if (!ap.takeoff_complete && motors.armed()) {
            if (pilot_throttle_scaled > g.throttle_cruise) {
                // we must be in the air by now
                set_takeoff_complete(true);
            }
        }
    }
}

// althold_init - initialise althold controller
static bool althold_init()
{
    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
static void althold_run()
{
    Vector3f angle_target = attitude_control.angle_ef_targets();     // for roll, pitch and yaw angular targets
    Vector3f rate_stab_ef_target;   // for yaw rate target.  Note Vector3f initialises all values to zero in constructor
    int16_t target_roll, target_pitch;
    int16_t target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: set target angles to zero or current attitude?
        // To-Do: reset altitude target if we're somehow not landed?
        attitude_control.set_throttle_out(0, false);
        return;
    }
    
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
    angle_target.x = target_roll;
    angle_target.y = target_pitch;

    // get pilot's desired yaw rate
    if (!failsafe.radio && !ap.land_complete) {
        rate_stab_ef_target.z = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // set target heading to current heading while landed
    if (ap.land_complete) {
        angle_target.z = ahrs.yaw_sensor;
    }

    // set earth-frame angular targets
    attitude_control.angle_ef_targets(angle_target);

    // convert earth-frame angle targets to earth-frame rate targets
    attitude_control.angle_to_rate_ef_roll();
    attitude_control.angle_to_rate_ef_pitch();

    // set earth-frame rate stabilize target for yaw with pilot's desired yaw
    // To-Do: this is quite wasteful to update the entire target vector when only yaw is used
    attitude_control.rate_stab_ef_targets(rate_stab_ef_target);

    // convert earth-frame stabilize rate to regular rate target
    // To-Do: replace G_Dt below
    attitude_control.rate_stab_ef_to_rate_ef_yaw();

    // convert earth-frame rates to body-frame rates
    attitude_control.rate_ef_targets_to_bf();

    // refetch angle targets for reporting
    angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;

    // body-frame rate controller is run directly from 100hz loop

    // To-Do: move throttle control up so we can disable angle and rate controllers?
    // check if we are taking off
    if (ap.land_complete) {
        if (target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }else{
            // move throttle to minimum to keep us on the ground
            attitude_control.set_throttle_out(0, false);
            // To-Do: should return here because we don't want throttle out to be overwritten below
        }
    }

    // check land_complete flag again in case it was changed above
    if (!ap.land_complete) {
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(target_climb_rate);
        }else{
            // if no sonar fall back stabilize rate controller
            pos_control.climb_at_rate(target_climb_rate);
        }
    }
}

// auto_init - initialise auto controller
static bool auto_init()
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
static bool circle_init()
{
    return true;
}

// circle_run - runs the circle controller
// should be called at 100hz or more
static void circle_run()
{
}

// loiter_init - initialise loiter controller
static bool loiter_init()
{
    return true;
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void loiter_run()
{
}

// guided_init - initialise guided controller
static bool guided_init()
{
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
static void guided_run()
{
}

// land_init - initialise land controller
static bool land_init()
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
static bool rtl_init()
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
static bool ofloiter_init()
{
    return true;
}

// ofloiter_run - runs the optical flow loiter controller
// should be called at 100hz or more
static void ofloiter_run()
{
}

// drift_init - initialise drift controller
static bool drift_init()
{
    return true;
}

// drift_run - runs the drift controller
// should be called at 100hz or more
static void drift_run()
{
}

// sport_init - initialise sport controller
static bool sport_init()
{
    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
static void sport_run()
{
}
