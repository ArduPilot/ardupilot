/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
 *
 * ekf_check.pde - detects failures of the ekf or inertial nav system
 *                 triggers an alert to the pilot and helps take countermeasures
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

// Enumerator for types of check
enum EKFCheckType {
    CHECK_NONE = 0,
    CHECK_DCM = 1,
    CHECK_EKF = 2
};

////////////////////////////////////////////////////////////////////////////////
// EKF_check strucutre
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count_compass; 		// number of iterations ekf or dcm have been out of tolerances

    uint8_t bad_compass : 1;    // true if dcm or ekf should be considered untrusted (fail_count_compass has exceeded EKF_CHECK_ITERATIONS_MAX)

    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_dcm_check - detects if ekf variances or dcm yaw errors that are out of tolerance and triggers failsafe
// should be called at 10hz
void ekf_dcm_check()
{
    EKFCheckType check_type = CHECK_NONE;

    // decide if we should check ekf or dcm
    if (ahrs.have_inertial_nav() && g.ekfcheck_thresh > 0.0f) {
        check_type = CHECK_EKF;
    } else if (g.dcmcheck_thresh > 0.0f) {
        check_type = CHECK_DCM;
    }

    // return immediately if motors are not armed, ekf check is disabled, not using ekf or usb is connected
    if (!motors.armed() || ap.usb_connected || check_type == CHECK_NONE) {
        ekf_check_state.fail_count_compass = 0;
        ekf_check_state.bad_compass = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_compass;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold
    if ((check_type == CHECK_EKF && ekf_over_threshold()) || (check_type == CHECK_DCM && dcm_over_threshold())) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_compass) {
            // increase counter
            ekf_check_state.fail_count_compass++;
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count_compass >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count_compass = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_compass = true;
                // log an error in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKFINAV_CHECK, ERROR_CODE_EKFINAV_CHECK_BAD_VARIANCE);
                // send message to gcs
                if ((hal.scheduler->millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    if (check_type == CHECK_EKF) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("EKF variance"));
                    } else {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("DCM bad heading"));
                    }
                    ekf_check_state.last_warn_time = hal.scheduler->millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count_compass > 0) {
            ekf_check_state.fail_count_compass--;

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_compass && ekf_check_state.fail_count_compass == 0) {
                ekf_check_state.bad_compass = false;
                // log recovery in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKFINAV_CHECK, ERROR_CODE_EKFINAV_CHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_compass;

    // To-Do: add ekf variances to extended status
}

// dcm_over_threshold - returns true if the dcm yaw error is over the tolerance
static bool dcm_over_threshold()
{
    // return true if yaw error is over the threshold
    return (g.dcmcheck_thresh > 0.0f && ahrs.get_error_yaw() > g.dcmcheck_thresh);
}

// ekf_over_threshold - returns true if the ekf's variance are over the tolerance
static bool ekf_over_threshold()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    // return false immediately if disabled
    if (g.ekfcheck_thresh <= 0.0f) {
        return false;
    }

    // use EKF to get variance
    float posVar, hgtVar, tasVar;
    Vector3f magVar;
    Vector2f offset;
    float compass_variance;
    float vel_variance;
    ahrs.get_NavEKF().getVariances(vel_variance, posVar, hgtVar, magVar, tasVar, offset);
    compass_variance = magVar.length();

    // return true if compass and velocity variance over the threshold
    return (compass_variance >= g.ekfcheck_thresh && vel_variance >= g.ekfcheck_thresh);
#else
    return false;
#endif
}


// failsafe_ekf_event - perform ekf failsafe
static void failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // do nothing if motors disarmed or not in flight mode that requires GPS
    if (!motors.armed() || !mode_requires_GPS(control_mode)) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode
    if (mode_requires_GPS(control_mode)) {
        set_mode_land_with_pause();
    }

    // if flight mode is LAND ensure it's not the GPS controlled LAND
    if (control_mode == LAND) {
        land_do_not_use_GPS();
    }
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
static void failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    // clear flag and log recovery
    failsafe.ekf = false;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, ERROR_CODE_FAILSAFE_RESOLVED);
}
