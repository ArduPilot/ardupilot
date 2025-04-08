#include "Copter.h"

/**
 *
 * Detects failures of the ekf or inertial nav system triggers an alert
 * to the pilot and helps take countermeasures
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check structure
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    bool bad_variance;          // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    bool has_ever_passed;       // true if the ekf checks have ever passed
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void Copter::ekf_check()
{
    // ensure EKF_CHECK_ITERATIONS_MAX is at least 7
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX must be at least 7");

    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if ekf check is disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold and also check
    // if we has a position estimate
    const bool over_threshold = ekf_over_threshold();
    const bool has_position = ekf_has_relative_position() || ekf_has_absolute_position();
    const bool checks_passed = !over_threshold && has_position;

    // return if ekf checks have never passed
    ekf_check_state.has_ever_passed |= checks_passed;
    if (!ekf_check_state.has_ever_passed) {
        return;
    }

    // increment or decrement counters and take action
    if (!checks_passed) {
        // if variances are not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2) && over_threshold) {
                // we are two iterations away from declaring an EKF failsafe, ask the EKF if we can reset
                // yaw to resolve the issue
                ahrs.request_yaw_reset();
            }
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-1)) {
                // we are just about to declare a EKF failsafe, ask the EKF if we can
                // change lanes to resolve the issue
                ahrs.check_lane_switch();
            }
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if variances are flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

// ekf_over_threshold - returns true if the ekf's variance are over the tolerance
bool Copter::ekf_over_threshold()
{
    // use EKF to get variance
    float position_var, vel_var, height_var, tas_variance;
    Vector3f mag_variance;
    variances_valid = ahrs.get_variances(vel_var, position_var, height_var, mag_variance, tas_variance);

    if (!variances_valid) {
        return false;
    }

    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_ekf_check_us) * 1e-6f;

    // always update filtered values as this serves the vibration check as well
    position_var = pos_variance_filt.apply(position_var, dt);
    vel_var = vel_variance_filt.apply(vel_var, dt);

    last_ekf_check_us = now_us;

    // return false if disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // return true if two of compass, velocity and position variances are over the threshold OR velocity variance is twice the threshold
    uint8_t over_thresh_count = 0;
    if (mag_max >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    bool optflow_healthy = false;
#if AP_OPTICALFLOW_ENABLED
    optflow_healthy = optflow.healthy();
#endif
    if (!optflow_healthy && (vel_var >= (2.0f * g.fs_ekf_thresh))) {
        over_thresh_count += 2;
    } else if (vel_var >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if ((position_var >= g.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}


// failsafe_ekf_event - perform ekf failsafe
void Copter::failsafe_ekf_event()
{
    // EKF failsafe event has occurred
    failsafe.ekf = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // if disarmed take no action
    if (!motors->armed()) {
        return;
    }

    // sometimes LAND *does* require GPS so ensure we are in non-GPS land
    if (flightmode->mode_number() == Mode::Number::LAND && landing_with_GPS()) {
        mode_land.do_not_use_GPS();
        return;
    }

    // does this mode require position?
    if (!copter.flightmode->requires_GPS() && (g.fs_ekf_action != FS_EKF_ACTION_LAND_EVEN_STABILIZE)) {
        return;
    }

    // take action based on fs_ekf_action parameter
    switch (g.fs_ekf_action) {
        case FS_EKF_ACTION_ALTHOLD:
            // AltHold
            if (failsafe.radio || !set_mode(Mode::Number::ALT_HOLD, ModeReason::EKF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            }
            break;
        case FS_EKF_ACTION_LAND:
        case FS_EKF_ACTION_LAND_EVEN_STABILIZE:
        default:
            set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            break;
    }

    // set true if ekf action is triggered
    AP_Notify::flags.failsafe_ekf = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe: changed to %s Mode", flightmode->name());
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
void Copter::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    failsafe.ekf = false;
    if (AP_Notify::flags.failsafe_ekf) {
        AP_Notify::flags.failsafe_ekf = false;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe Cleared");
    }
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}

// re-check if the flight mode requires GPS but EKF failsafe is active
// this should be called by flight modes that are changing their submode from one that does NOT require a position estimate to one that does
void Copter::failsafe_ekf_recheck()
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    // trigger EKF failsafe action
    failsafe_ekf_event();
}

// check for ekf yaw reset and adjust target heading, also log position reset
void Copter::check_ekf_reset()
{
    // check for yaw reset
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        LOGGER_WRITE_EVENT(LogEvent::EKF_YAW_RESET);
    }

    // check for change in primary EKF, reset attitude target and log.  AC_PosControl handles position target adjustment
    if ((ahrs.get_primary_core_index() != ekf_primary_core) && (ahrs.get_primary_core_index() != -1)) {
        attitude_control->inertial_frame_reset();
        ekf_primary_core = ahrs.get_primary_core_index();
        LOGGER_WRITE_ERROR(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(ekf_primary_core));
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF primary changed:%d", (unsigned)ekf_primary_core);
    }
}

// check for high vibrations affecting altitude control
void Copter::check_vibration()
{
    uint32_t now = AP_HAL::millis();

    // assume checks will succeed
    bool innovation_checks_valid = true;

    // check if vertical velocity and position innovations are positive (NKF3.IVD & NKF3.IPD are both positive)
    Vector3f vel_innovation;
    Vector3f pos_innovation;
    Vector3f mag_innovation;
    float tas_innovation;
    float yaw_innovation;
    if (!ahrs.get_innovations(vel_innovation, pos_innovation, mag_innovation, tas_innovation, yaw_innovation)) {
        innovation_checks_valid = false;
    }
    const bool innov_velD_posD_positive = is_positive(vel_innovation.z) && is_positive(pos_innovation.z);

    // check if vertical velocity variance is at least 1 (NK4.SV >= 1.0)
    // filtered variances are updated in ekf_check() which runs at the same rate (10Hz) as this check
    if (!variances_valid) {
        innovation_checks_valid = false;
    }
    const bool is_vibration_affected = ahrs.is_vibration_affected();
    const bool bad_vibe_detected = (innovation_checks_valid && innov_velD_posD_positive && (vel_variance_filt.get() > 1.0f)) || is_vibration_affected;
    const bool do_bad_vibe_actions = (g2.fs_vibe_enabled == 1) && bad_vibe_detected && motors->armed() && !flightmode->has_manual_throttle();

    if (!vibration_check.high_vibes) {
        // initialise timers
        if (!do_bad_vibe_actions) {
            vibration_check.start_ms = now;
        }
        // check if failure has persisted for at least 1 second
        if (now - vibration_check.start_ms > 1000) {
            // switch position controller to use resistant gains
            vibration_check.clear_ms = 0;
            vibration_check.high_vibes = true;
            pos_control->set_vibe_comp(true);
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_OCCURRED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation ON");
        }
    } else {
        // initialise timer
        if (do_bad_vibe_actions) {
            vibration_check.clear_ms = now;
        }
        // turn off vibration compensation after 15 seconds
        if (now - vibration_check.clear_ms > 15000) {
            // restore position controller gains, reset timers and update user
            vibration_check.start_ms = 0;
            vibration_check.high_vibes = false;
            pos_control->set_vibe_comp(false);
            vibration_check.clear_ms = 0;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation OFF");
        }
    }

    return;
}
