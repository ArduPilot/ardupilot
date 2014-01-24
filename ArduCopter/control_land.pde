/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// counter to verify landings
static uint16_t land_detector;
static bool land_with_gps;

// land_init - initialise land controller
static bool land_init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    land_with_gps = GPS_ok();
    if (land_with_gps) {
        // set target to current position
        wp_nav.init_loiter_target();
    }
    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
static void land_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    float target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset altitude target if we're somehow not landed?
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // disarming detector
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
    // disarm when the landing detector says we've landed and throttle is at minimum
    if (ap.land_complete && (g.rc_3.control_in == 0 || failsafe.radio)) {
        init_disarm_motors();
        return;
    }
#else
    // disarm when the landing detector says we've landed
    if (ap.land_complete) {
        init_disarm_motors();
        return;
    }
#endif

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.init_targets();
        // move throttle to minimum to keep us on the ground
        attitude_control.set_throttle_out(0, false);
    }else{
        // call attitude controller
        attitude_control.angleef_rp_rateef_y(target_roll, target_pitch, target_yaw_rate);
        // body-frame rate controller is run directly from 100hz loop

        // call landing throttle controller
        target_climb_rate = get_throttle_land();

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

// get_throttle_land - high level landing logic
//      returns climb rate (in cm/s) which should be passed to the position controller
//      should be called at 100hz or higher
static float get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        return pos_control.get_speed_down();
    }else{
        return -abs(g.land_speed);
    }
}

// reset_land_detector - initialises land detector
static void reset_land_detector()
{
    set_land_complete(false);
    land_detector = 0;
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
static bool update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(climb_rate) < 20 && motors.limit.throttle_lower) {
        if (!ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = 0;
            }
        }
    }else{
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        if(ap.land_complete) {
            set_land_complete(false);
        }
    }

    // return current state of landing
    return ap.land_complete;
}
