#include "Plane.h"

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
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
    bool failsafe_on;           // true when the loss of navigation failsafe is on
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void Plane::ekf_check()
{
    // ensure EKF_CHECK_ITERATIONS_MAX is at least 7
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX must be at least 7");

    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if motors are not armed, or ekf check is disabled
    bool ekf_check_disabled = !plane.arming.is_armed() || (g2.fs_ekf_thresh <= 0.0f);
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.in_vtol_posvel_mode()) {
        ekf_check_disabled = true;
    }
#endif
    if (ekf_check_disabled) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold
    if (ekf_over_threshold()) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2)) {
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
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
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

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
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
bool Plane::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g2.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // Get EKF innovations normalised wrt the innovaton test limits.
    // A value above 1.0 means the EKF has rejected that sensor data
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    if (!ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance)) {
        return false;
    };

    // The EKF rejects all magnetometer axes if any single axis exceeds limits
    // so take the maximum of all axes
    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // Assign a score to each over threshold based on severity
    uint8_t over_thresh_count = 0;
    if (mag_max >= g2.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if (vel_variance >= (2.0f * g2.fs_ekf_thresh)) {
        over_thresh_count += 2;
    } else if (vel_variance >= g2.fs_ekf_thresh) {
        over_thresh_count++;
    }

    // Position is the most important so accept a lower score from other sensors if position failed
    if ((position_variance >= g2.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}


// failsafe_ekf_event - perform ekf failsafe
void Plane::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (ekf_check_state.failsafe_on) {
        return;
    }

    // EKF failsafe event has occurred
    ekf_check_state.failsafe_on = true;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // if not in a VTOL mode requring position, then nothing needs to be done
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.in_vtol_posvel_mode()) {
        return;
    }

    if (quadplane.in_vtol_auto()) {
        // the pilot is not controlling via sticks so switch to QLAND
        plane.set_mode(mode_qland, ModeReason::EKF_FAILSAFE);
    } else {
        // the pilot is controlling via sticks so fallbacl to QHOVER
        plane.set_mode(mode_qhover, ModeReason::EKF_FAILSAFE);
    }
#endif
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
void Plane::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!ekf_check_state.failsafe_on) {
        return;
    }

    ekf_check_state.failsafe_on = false;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}
