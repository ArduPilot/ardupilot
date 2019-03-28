#include "Rover.h"

/**
 *
 * Detects failures of the ekf and triggers a failsafe
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif


// EKF_check structure
static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    uint8_t bad_variance : 1;   // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void Rover::ekf_check()
{
    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if motors are not armed, or ekf check is disabled
    if (!arming.is_armed() || (g.fs_ekf_thresh <= 0.0f)) {
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
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;

                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK,
                                         LogErrorCode::EKFCHECK_BAD_VARIANCE);
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

            // if variance is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK,
                                         LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
}

// returns true if the ekf's variance are over the tolerance
bool Rover::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // use EKF to get variance
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance, offset);

    // return true if two of compass, velocity and position variances are over the threshold
    uint8_t over_thresh_count = 0;
    if (mag_variance.length() >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    if (position_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if (over_thresh_count >= 2) {
        return true;
    }

    if (ekf_position_ok()) {
        return false;
    }

    return true;
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Rover::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // get EKF filter status
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // if disarmed we accept a predicted horizontal position
    if (!arming.is_armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// perform ekf failsafe
void Rover::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV,
                             LogErrorCode::FAILSAFE_OCCURRED);
    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF failsafe!");

    // does this mode require position?
    if (!control_mode->requires_position()) {
        return;
    }

    // take action based on fs_ekf_action parameter
    switch ((enum fs_ekf_action)g.fs_ekf_action.get()) {
        case FS_EKF_DISABLE:
            // do nothing
            break;
        case FS_EFK_HOLD:
        default:
            set_mode(mode_hold, MODE_REASON_EKF_FAILSAFE);
            break;
    }
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
void Rover::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    failsafe.ekf = false;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV,
                             LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF failsafe cleared");
}
