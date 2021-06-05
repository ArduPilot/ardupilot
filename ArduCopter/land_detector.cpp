#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity


// counter to verify landings
static uint32_t land_detector_count = 0;

// run land and crash detectors
// called at MAIN_LOOP_RATE
void Copter::update_land_and_crash_detectors()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, scheduler.get_loop_period_s());

    update_land_detector();

#if PARACHUTE == ENABLED
    // check parachute
    parachute_check();
#endif

    crash_check();
    thrust_loss_check();
    yaw_imbalance_check();
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
void Copter::update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    if (!motors->armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
    } else if (ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME
        // if rotor speed and collective pitch are high then clear landing flag
        if (motors->get_takeoff_collective() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
#else
        // if throttle output is high then clear landing flag
        if (motors->get_throttle() > get_non_takeoff_throttle()) {
#endif
            set_land_complete(false);
        }
    } else if (standby_active) {
        // land detector will not run in standby mode
        land_detector_count = 0;
    } else {

#if FRAME_CONFIG == HELI_FRAME
        // check that collective pitch is below mid collective (zero thrust) position
        bool motor_at_lower_limit = (motors->get_below_mid_collective() && fabsf(ahrs.get_roll()) < M_PI/2.0f);
#else
        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
#endif

        uint8_t land_detector_scalar = 1;
#if LANDING_GEAR_ENABLED == ENABLED
        if (landinggear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
            // we have a WoW sensor so lets loosen the strictness of the landing detector
            land_detector_scalar = 2;
        }
#endif

        // check that the airframe is not accelerating (not falling or braking after fast forward flight)
        bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX * land_detector_scalar);

        // check that vertical speed is within 1m/s of zero
        bool descent_rate_low = fabsf(inertial_nav.get_velocity_z()) < 100 * land_detector_scalar;

        // if we have a healthy rangefinder only allow landing detection below 2 meters
        bool rangefinder_check = (!rangefinder_alt_ok() || rangefinder_state.alt_cm_filt.get() < LAND_RANGEFINDER_MIN_ALT_CM);

        // if we have weight on wheels (WoW) or ambiguous unknown. never no WoW
#if LANDING_GEAR_ENABLED == ENABLED
        const bool WoW_check = (landinggear.get_wow_state() == AP_LandingGear::LG_WOW || landinggear.get_wow_state() == AP_LandingGear::LG_WOW_UNKNOWN);
#else
        const bool WoW_check = true;
#endif

        if (motor_at_lower_limit && accel_stationary && descent_rate_low && rangefinder_check && WoW_check) {
            // landed criteria met - increment the counter and check if we've triggered
            if( land_detector_count < ((float)LAND_DETECTOR_TRIGGER_SEC)*scheduler.get_loop_rate_hz()) {
                land_detector_count++;
            } else {
                set_land_complete(true);
            }
        } else {
            // we've sensed movement up or down so reset land_detector
            land_detector_count = 0;
        }
    }

    set_land_complete_maybe(ap.land_complete || (land_detector_count >= LAND_DETECTOR_MAYBE_TRIGGER_SEC*scheduler.get_loop_rate_hz()));
}

// set land_complete flag and disarm motors if disarm-on-land is configured
void Copter::set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    land_detector_count = 0;

    if(b){
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
    } else {
        AP::logger().Write_Event(LogEvent::NOT_LANDED);
    }
    ap.land_complete = b;

#if STATS_ENABLED == ENABLED
    g2.stats.set_flying(!b);
#endif

    // tell AHRS flying state
    set_likely_flying(!b);
    
    // trigger disarm-on-land if configured
    bool disarm_on_land_configured = (g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) != 0;
    const bool mode_disarms_on_land = flightmode->allows_arming(AP_Arming::Method::LANDING) && !flightmode->has_manual_throttle();

    if (ap.land_complete && motors->armed() && disarm_on_land_configured && mode_disarms_on_land) {
        arming.disarm(AP_Arming::Method::LANDED);
    }
}

// set land complete maybe flag
void Copter::set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE_MAYBE);
    }
    ap.land_complete_maybe = b;
}

// sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
void Copter::update_throttle_mix()
{
#if FRAME_CONFIG != HELI_FRAME
    // if disarmed or landed prioritise throttle
    if (!motors->armed() || ap.land_complete) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    if (flightmode->has_manual_throttle()) {
        // manual throttle
        if (channel_throttle->get_control_in() <= 0 || air_mode == AirMode::AIRMODE_DISABLED) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = (norm(angle_target.x, angle_target.y) > LAND_CHECK_LARGE_ANGLE_CD);

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        const bool accel_moving = (land_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_cms().z >= 0.0f;

        // check if landing
        const bool landing = flightmode->is_landing();

        if ((large_angle_request && !landing) || large_angle_error || accel_moving || descent_not_demanded) {
            attitude_control->set_throttle_mix_max(pos_control->get_vel_z_control_ratio());
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
#endif
}
