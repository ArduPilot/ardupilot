/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// counter to verify landings
static uint32_t land_detector = ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE; // we assume we are landed

// land_complete_maybe - return true if we may have landed (used to reset loiter targets during landing)
static bool land_complete_maybe()
{
    return (ap.land_complete || ap.land_complete_maybe);
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
static void update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    // check that the average throttle output is near minimum (less than 12.5% hover throttle)
    bool motor_at_lower_limit = motors.limit.throttle_lower && (motors.get_throttle_low_comp() < 0.125f);
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;

    // lowpass filter on accel
    accel_ef = land_accel_ef_filter.apply(accel_ef, MAIN_LOOP_SECONDS);

    // check that the airframe is not accelerating (not falling or breaking after fast forward flight)
    bool accel_stationary = (accel_ef.length() < 1.0f);

    if ( motor_at_lower_limit && accel_stationary) {
        if (!ap.land_complete) {
            // increase counter until we hit the trigger then set land complete flag
            if( land_detector < ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE;
            }
        }
    } else {
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        // if throttle output is high then clear landing flag
        if (motors.get_throttle_out() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
    }

    // set land maybe flag
    set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER_SEC*MAIN_LOOP_RATE);
}

// update_throttle_low_comp - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
static void update_throttle_low_comp()
{
    if (mode_has_manual_throttle(control_mode)) {
        // manual throttle
        if(!motors.armed() || g.rc_3.control_in <= 0) {
            motors.set_throttle_low_comp(0.1f);
        } else {
            motors.set_throttle_low_comp(0.5f);
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests
        const Vector3f angle_target = attitude_control.angle_ef_targets();
        bool large_angle_request = (pythagorous2(angle_target.x, angle_target.y) > 1500.0f);

        // check for large external disturbance
        const Vector3f angle_error = attitude_control.angle_bf_error();
        bool large_angle_error = (pythagorous2(angle_error.x, angle_error.y) > 3000.0f);

        // check for large acceleration ( falling )
        Vector3f accel_ef = ahrs.get_accel_ef_blended();
        accel_ef.z += GRAVITY_MSS;
        bool accel_moving = (accel_ef.length() > 3.0f);

        if ( large_angle_request || large_angle_error || accel_moving) {
            // if target lean angle is over 15 degrees set high
            motors.set_throttle_low_comp(0.9f);
        } else {
            motors.set_throttle_low_comp(0.1f);
        }
    }
}

static void update_ground_effect_detector(void)
{
    static bool takeoffExpected = false;
    static uint32_t takeoff_time_ms;
    static float takeoff_alt_cm;

    if(!motors.armed()) {
        // disarmed - disable ground effect and return
        ahrs.setTakeoffExpected(false);
        ahrs.setTouchdownExpected(false);
        takeoffExpected = false;
        return;
    }

    // variable initialization
    uint32_t tnow_ms = hal.scheduler->millis();
    float xy_des_speed_cms = 0.0f;
    float xy_speed_cms = 0.0f;
    float des_climb_rate_cms = pos_control.get_desired_velocity().z;

    if(pos_control.is_active_xy()) {
        Vector3f vel_target = pos_control.get_vel_target();
        vel_target.z = 0.0f;
        xy_des_speed_cms = vel_target.length();
    }

    if(position_ok() || optflow_position_ok()) {
        Vector3f vel = inertial_nav.get_velocity();
        vel.z = 0.0f;
        xy_speed_cms = vel.length();
    }

    // takeoff logic

    // if we are armed and haven't yet taken off
    if (motors.armed() && ap.land_complete && !takeoffExpected) {
        takeoffExpected = true;
    }

    // if we aren't taking off yet, reset the takeoff timer, altitude and complete flag
    bool throttle_up = mode_has_manual_throttle(control_mode) && g.rc_3.control_in > 0;
    if (!throttle_up && ap.land_complete) {
        takeoff_time_ms = tnow_ms;
        takeoff_alt_cm = inertial_nav.get_altitude();
    }

    // if we are in takeoffExpected and we meet the conditions for having taken off
    // end the takeoffExpected state
    if (takeoffExpected && (tnow_ms-takeoff_time_ms > 5000 || inertial_nav.get_altitude()-takeoff_alt_cm > 50.0f)) {
        takeoffExpected = false;
    }

    // landing logic
    const Vector3f& angle_target = attitude_control.angle_ef_targets();
    bool small_angle_request = pythagorous2(angle_target.x, angle_target.y) < 750.0f;
    bool xy_speed_low = (position_ok() || optflow_position_ok()) && xy_speed_cms <= 125.0f;
    bool xy_speed_demand_low = pos_control.is_active_xy() && xy_des_speed_cms <= 125.0f;
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control.is_active_xy()) || (control_mode == ALT_HOLD && small_angle_request);

    bool descent_demanded = pos_control.is_active_z() && des_climb_rate_cms < 0.0f;
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;
    bool z_speed_low = abs(climb_rate) <= LAND_DETECTOR_CLIMBRATE_MAX*2.0f;
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));

    bool touchdownExpected = slow_horizontal && slow_descent;

    // Prepare the EKF for ground effect if either takeoff or touchdown is expected.
    ahrs.setTakeoffExpected(takeoffExpected);
    ahrs.setTouchdownExpected(touchdownExpected);
}
