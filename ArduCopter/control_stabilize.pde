/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize, althold, drift and sport flight modes
 */

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
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }

    // refetch angle targets for reporting
    const Vector3f angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;
}

// circle_init - initialise circle controller
static bool circle_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        return true;
    }else{
        return false;
    }
    // set yaw to point to center of circle
    // yaw_look_at_WP = circle_center;
    // initialise bearing to current heading
    //yaw_look_at_WP_bearing = ahrs.yaw_sensor;
    //yaw_initialised = true;
}

// circle_run - runs the circle controller
// should be called at 100hz or more
static void circle_run()
{
    // if we are landed reset yaw target to current heading
    //if (ap.land_complete) {
    //    control_yaw = ahrs.yaw_sensor;
    //}
    // points toward the center of the circle or does a panorama
    //get_circle_yaw();
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
    // if we have landed reset yaw target to current heading
    //if (ap.land_complete) {
    //    control_yaw = ahrs.yaw_sensor;
    //}
    //get_yaw_drift();
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
