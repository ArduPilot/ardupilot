#include "Copter.h"

#include <AP_Stats/AP_Stats.h>              // statistics library

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
    Vector3f accel_ef = ahrs.get_accel_ef();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, scheduler.get_loop_period_s());

    update_land_detector();

#if HAL_PARACHUTE_ENABLED
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

#if HAL_LOGGING_ENABLED
    uint16_t logging_flags = 0;
#define SET_LOG_FLAG(condition, flag) if (condition) { logging_flags |= (uint16_t)flag; }
#else
#define SET_LOG_FLAG(condition, flag)
#endif

    if (!motors->armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
    } else if (ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME
        // if rotor speed and collective pitch are high then clear landing flag
        if (!flightmode->is_taking_off() && motors->get_takeoff_collective() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
#else
        // if throttle output is high then clear landing flag
        if (!flightmode->is_taking_off() && motors->get_throttle_out() > get_non_takeoff_throttle() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            // this should never happen because take-off should be detected at the flight mode level
            // this here to highlight there is a bug or missing take-off detection
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
#endif
            set_land_complete(false);
        }
    } else if (standby_active) {
        // land detector will not run in standby mode
        land_detector_count = 0;
    } else {

        float land_trigger_sec = LAND_DETECTOR_TRIGGER_SEC;
#if FRAME_CONFIG == HELI_FRAME
        // check for both manual collective modes and modes that use altitude hold. For manual collective (called throttle
        // because multi's use throttle), check that collective pitch is below land min collective position or throttle stick is zero.
        // Including the throttle zero check will ensure the conditions where stabilize stick zero position was not below collective min. For modes
        // that use altitude hold, check that the pilot is commanding a descent and collective is at min allowed for altitude hold modes.

        // check if landing
        const bool landing = flightmode->is_landing();
        SET_LOG_FLAG(landing, LandDetectorLoggingFlag::LANDING);
        bool motor_at_lower_limit = (flightmode->has_manual_throttle() && (motors->get_below_land_min_coll() || heli_flags.coll_stk_low) && fabsf(ahrs.get_roll()) < M_PI/2.0f)
#if MODE_AUTOROTATE_ENABLED
                                    || (flightmode->mode_number() == Mode::Number::AUTOROTATE && motors->get_below_land_min_coll())
#endif
                                    || ((!get_force_flying() || landing) && motors->limit.throttle_lower && pos_control->get_vel_desired_cms().z < 0.0f);
        bool throttle_mix_at_min = true;
#else
        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower;
        bool throttle_mix_at_min = attitude_control->is_throttle_mix_min();
        // set throttle_mix_at_min to true because throttle is never at mix min in airmode
        // increase land_trigger_sec when using airmode
        if (flightmode->has_manual_throttle() && air_mode == AirMode::AIRMODE_ENABLED) {
            land_trigger_sec = LAND_AIRMODE_DETECTOR_TRIGGER_SEC;
            throttle_mix_at_min = true;
        }
#endif
        SET_LOG_FLAG(motor_at_lower_limit, LandDetectorLoggingFlag::MOTOR_AT_LOWER_LIMIT);
        SET_LOG_FLAG(throttle_mix_at_min, LandDetectorLoggingFlag::THROTTLE_MIX_AT_MIN);

        uint8_t land_detector_scalar = 1;
#if AP_LANDINGGEAR_ENABLED
        if (landinggear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
            // we have a WoW sensor so lets loosen the strictness of the landing detector
            land_detector_scalar = 2;
        }
#endif

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD;
        SET_LOG_FLAG(large_angle_request, LandDetectorLoggingFlag::LARGE_ANGLE_REQUEST);

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);
        SET_LOG_FLAG(large_angle_error, LandDetectorLoggingFlag::LARGE_ANGLE_ERROR);

        // check that the airframe is not accelerating (not falling or braking after fast forward flight)
        bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX * land_detector_scalar);
        SET_LOG_FLAG(accel_stationary, LandDetectorLoggingFlag::ACCEL_STATIONARY);

        // check that vertical speed is within 1m/s of zero
        bool descent_rate_low = fabsf(inertial_nav.get_velocity_z_up_cms()) < 100.0 * LAND_DETECTOR_VEL_Z_MAX * land_detector_scalar;
        SET_LOG_FLAG(descent_rate_low, LandDetectorLoggingFlag::DESCENT_RATE_LOW);

        // if we have a healthy rangefinder only allow landing detection below 2 meters
        bool rangefinder_check = (!rangefinder_alt_ok() || rangefinder_state.alt_cm_filt.get() < LAND_RANGEFINDER_MIN_ALT_CM);
        SET_LOG_FLAG(rangefinder_check, LandDetectorLoggingFlag::RANGEFINDER_BELOW_2M);

        // if we have weight on wheels (WoW) or ambiguous unknown. never no WoW
#if AP_LANDINGGEAR_ENABLED
        const bool WoW_check = (landinggear.get_wow_state() == AP_LandingGear::LG_WOW || landinggear.get_wow_state() == AP_LandingGear::LG_WOW_UNKNOWN);
#else
        const bool WoW_check = true;
#endif
        SET_LOG_FLAG(WoW_check, LandDetectorLoggingFlag::WOW);

        if (motor_at_lower_limit && throttle_mix_at_min && !large_angle_request && !large_angle_error && accel_stationary && descent_rate_low && rangefinder_check && WoW_check) {
            // landed criteria met - increment the counter and check if we've triggered
            if( land_detector_count < land_trigger_sec*scheduler.get_loop_rate_hz()) {
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

#if HAL_LOGGING_ENABLED
// @LoggerMessage: LDET
// @Description: Land Detector State
// @Field: TimeUS: Time since system startup
// @Field: Flags: boolean state flags
// @FieldBitmaskEnum: Flags: Copter::LandDetectorLoggingFlag
// @Field: Count: landing_detector pass count
    SET_LOG_FLAG(ap.land_complete, LandDetectorLoggingFlag::LANDED);
    SET_LOG_FLAG(ap.land_complete_maybe, LandDetectorLoggingFlag::LANDED_MAYBE);
    SET_LOG_FLAG(standby_active, LandDetectorLoggingFlag::STANDBY_ACTIVE);
    Log_LDET(logging_flags, land_detector_count);
#undef SET_LOG_FLAG
#endif
}

#if HAL_LOGGING_ENABLED
void Copter::Log_LDET(uint16_t logging_flags, uint32_t detector_count)
{
    // do not log if no change:
    if (logging_flags == land_detector.last_logged_flags &&
        detector_count == land_detector.last_logged_count) {
        return;
    }
    // do not log more than 50Hz:
    const auto now = AP_HAL::millis();
    if (now - land_detector.last_logged_ms < 20) {
        return;
    }

    land_detector.last_logged_count = detector_count;
    land_detector.last_logged_flags = logging_flags;
    land_detector.last_logged_ms = now;

    AP::logger().WriteStreaming(
        "LDET",
        "TimeUS," "Flags," "Count",
        "s"       "-"      "-",
        "F"       "-"      "-",
        "Q"       "H"      "I",
        AP_HAL::micros64(),
        logging_flags,
        land_detector_count
    );
}
#endif

// set land_complete flag and disarm motors if disarm-on-land is configured
void Copter::set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    land_detector_count = 0;

#if HAL_LOGGING_ENABLED
    if(b){
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
    } else {
        AP::logger().Write_Event(LogEvent::NOT_LANDED);
    }
#endif
    ap.land_complete = b;

#if AP_STATS_ENABLED
    AP::stats()->set_flying(!b);
#endif

    // tell AHRS flying state
    set_likely_flying(!b);

    if (!b) {
        // not landed, no further action
        return;
    }

    // landed; trigger disarm-on-land if configured
    if ((g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) == 0) {
        // not configured to disarm on landing detection
        return;
    }

    if (!motors->armed()) {
        // we are not currently armed, so we don't need to disarm:
        // n.b. should this be checking vehicle-armed rather than motors-armed?
        return;
    }

    if (flightmode->has_manual_throttle()) {
        // we do not use the landing detector to disarm if the vehicle
        // is in e.g. STABILIZE.  The normal DISARM_DELAY logic applies.
        return;
    }

    // the flightmode may not allow disarm on landing.  Note that this
    // check returns false for the LAND flight mode - it checks the
    // landing detector (ap.land_complete) itself.
    if (!flightmode->allows_arming(AP_Arming::Method::LANDING)) {
        return;
    }

    // all checks passed, disarm the vehicle:
    arming.disarm(AP_Arming::Method::LANDED);
}

// set land complete maybe flag
void Copter::set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        LOGGER_WRITE_EVENT(LogEvent::LAND_COMPLETE_MAYBE);
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
        if (channel_throttle->get_control_in() <= 0 && air_mode != AirMode::AIRMODE_ENABLED) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD;

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        const bool accel_moving = (land_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_cms().z >= 0.0f;

        // check if landing
        const bool landing = flightmode->is_landing();

        if (((large_angle_request || get_force_flying()) && !landing) || large_angle_error || accel_moving || descent_not_demanded) {
            attitude_control->set_throttle_mix_max(pos_control->get_vel_z_control_ratio());
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
#endif
}

// helper function for force flying flag
bool Copter::get_force_flying() const
{
#if FRAME_CONFIG == HELI_FRAME
    if (attitude_control->get_inverted_flight()) {
        return true;
    }
#endif
    return force_flying;
}
