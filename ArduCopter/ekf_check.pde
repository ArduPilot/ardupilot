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

#ifndef EKF_CHECK_COMPASS_INAV_CONVERSION
 # define EKF_CHECK_COMPASS_INAV_CONVERSION 0.01f  // converts the inertial nav's acceleration corrections to a form that is comparable to the ekf variance
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check strucutre
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count_compass; // number of iterations ekf's compass variance has been out of tolerances

    uint8_t bad_compass : 1;    // true if compass variance is bad

    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects ekf variances that are out of tolerance
// should be called at 10hz
void ekf_check()
{
    // return immediately if motors are not armed, ekf check is disabled, no inertial-nav position yet or usb is connected
    if (!motors.armed() || g.ekfcheck_compass_thresh == 0.0f || !inertial_nav.position_ok() || ap.usb_connected) {
        ekf_check_state.fail_count_compass = 0;
        ekf_check_state.bad_compass = 0;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_compass;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // variances
    float compass_variance = 0;

#if AP_AHRS_NAVEKF_AVAILABLE
    if (ahrs.have_inertial_nav()) {
        // use EKF to get variance
        float velVar, posVar, hgtVar, tasVar;
        Vector3f magVar;
        Vector2f offset;
        ahrs.get_NavEKF().getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
        compass_variance = magVar.length();
    } else {
        // use complementary filter's acceleration corrections multiplied by conversion factor to make them general in the same range as the EKF's variances
        compass_variance = safe_sqrt(inertial_nav.accel_correction_hbf.x * inertial_nav.accel_correction_hbf.x + inertial_nav.accel_correction_hbf.y * inertial_nav.accel_correction_hbf.y) * EKF_CHECK_COMPASS_INAV_CONVERSION;
    }
#else
    // use complementary filter's acceleration corrections multiplied by conversion factor to make them general in the same range as the EKF's variances
    compass_variance = safe_sqrt(inertial_nav.accel_correction_hbf.x * inertial_nav.accel_correction_hbf.x + inertial_nav.accel_correction_hbf.y * inertial_nav.accel_correction_hbf.y) * EKF_CHECK_COMPASS_INAV_CONVERSION;
#endif

    // compare compass variance vs threshold
    if (compass_variance >= g.ekfcheck_compass_thresh) {
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
                Log_Write_Error(ERROR_SUBSYSTEM_EKF_CHECK, ERROR_CODE_EKF_CHECK_BAD_COMPASS);
                // send message to gcs
                if ((hal.scheduler->millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("EKF: compass variance"));
                    ekf_check_state.last_warn_time = hal.scheduler->millis();
                }
                // trigger failsafe
                failsafe_ekf_event();
            }
        }
    } else {
        // if compass is flagged as bad
        if (ekf_check_state.bad_compass) {
            // reduce counter
            ekf_check_state.fail_count_compass--;
            // if counter reaches zero then clear flag
            if (ekf_check_state.fail_count_compass == 0) {
                ekf_check_state.bad_compass = false;
                // log recovery in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKF_CHECK, ERROR_CODE_EKF_CHECK_BAD_COMPASS_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_compass;

    // To-Do: add check for althold when vibrations are high
    // To-Do: add ekf variances to extended status
    // To-Do: add counter measures to try and recover from bad EKF
    // To-Do: add check into GPS position_ok() to return false if ekf xy not healthy?
    // To-Do: ensure it compiles for AVR
}

// failsafe_ekf_event - perform ekf failsafe
static void failsafe_ekf_event()
{
    uint32_t last_gps_update_ms;

    // return immediately if ekf failsafe already triggered or disabled
    if (failsafe.ekf || g.ekfcheck_compass_thresh <= 0.0f) {
        return;
    }

    // do nothing if motors disarmed or not in flight mode that requires GPS
    if (!motors.armed() || !mode_requires_GPS(control_mode)) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKF, ERROR_CODE_FAILSAFE_OCCURRED);

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
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKF, ERROR_CODE_FAILSAFE_RESOLVED);
}
