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
    bool motor_at_lower_limit = motors.limit.throttle_lower && motors.is_throttle_mix_min();
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

// update_throttle_thr_mix - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
static void update_throttle_thr_mix()
{
    if (mode_has_manual_throttle(control_mode)) {
        // manual throttle
        if(!motors.armed() || g.rc_3.control_in <= 0) {
            motors.set_throttle_mix_min();
        } else {
            motors.set_throttle_mix_mid();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control.angle_ef_targets();
        bool large_angle_request = (pythagorous2(angle_target.x, angle_target.y) > 1500.0f);

        // check for large external disturbance - angle error over 30 degrees
        const Vector3f angle_error = attitude_control.angle_bf_error();
        bool large_angle_error = (pythagorous2(angle_error.x, angle_error.y) > 3000.0f);

        // check for large acceleration - falling or high turbulence
        Vector3f accel_ef = ahrs.get_accel_ef_blended();
        accel_ef.z += GRAVITY_MSS;
        bool accel_moving = (accel_ef.length() > 3.0f);

        // check for requested decent
        bool descent_not_demanded = pos_control.get_desired_velocity().z >= 0.0f;

        if ( large_angle_request || large_angle_error || accel_moving || descent_not_demanded) {
            motors.set_throttle_mix_max();
        } else {
            motors.set_throttle_mix_min();
        }
    }
}
