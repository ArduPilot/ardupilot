#include "Copter.h"

// performs pre-arm checks. expects to be called at 1hz.
void AP_Arming_Copter::update(void)
{
    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = PREARM_DISPLAY_PERIOD/2;
    pre_arm_display_counter++;
    bool display_fail = false;
    if (pre_arm_display_counter >= PREARM_DISPLAY_PERIOD) {
        display_fail = true;
        pre_arm_display_counter = 0;
    }

    if (pre_arm_checks(display_fail)) {
        set_pre_arm_check(true);
    }
}

// performs pre-arm checks and arming checks
bool AP_Arming_Copter::all_checks_passing(bool arming_from_gcs)
{
    if (pre_arm_checks(true)) {
        set_pre_arm_check(true);
    } else {
        return false;
    }

    return copter.ap.pre_arm_check && arm_checks(true, arming_from_gcs);
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
// NOTE: this does *NOT* call AP_Arming::pre_arm_checks() yet!
bool AP_Arming_Copter::pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (copter.motors->armed()) {
        return true;
    }

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK) && copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Interlock/E-Stop Conflict");
        }
        return false;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.  This check to be repeated,
    // as state can change at any time.
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Motor Interlock Enabled");
        }
        return false;
    }

    // exit immediately if we've already successfully performed the pre-arm check
    if (copter.ap.pre_arm_check) {
        // run gps checks because results may change and affect LED colour
        // no need to display failures because arm_checks will do that if the pilot tries to arm
        pre_arm_gps_checks(false);
        return true;
    }

    // succeed if pre arm checks are disabled
    if (checks_to_perform == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return true;
    }

    return barometer_checks(display_failure)
        & rc_calibration_checks(display_failure)
        & compass_checks(display_failure)
        & gps_checks(display_failure)
        & fence_checks(display_failure)
        & ins_checks(display_failure)
        & board_voltage_checks(display_failure)
        & logging_checks(display_failure)
        & parameter_checks(display_failure)
        & motor_checks(display_failure)
        & pilot_throttle_checks(display_failure);
}

bool AP_Arming_Copter::rc_calibration_checks(bool display_failure)
{
    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks(display_failure);
    return copter.ap.pre_arm_rc_check;
}

bool AP_Arming_Copter::barometer_checks(bool display_failure)
{
    // check Baro
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_BARO)) {
        // barometer health check
        if (!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Barometer not healthy");
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = _inav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(_inav.get_altitude() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                if (display_failure) {
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Altitude disparity");
                }
                return false;
            }
        }
    }
    return true;
}

bool AP_Arming_Copter::compass_checks(bool display_failure)
{
    bool ret = AP_Arming::compass_checks(display_failure);

    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_COMPASS)) {
        // check compass offsets have been set.  AP_Arming only checks
        // this if learning is off; Copter *always* checks.
        if (!_compass.configured()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Compass not calibrated");
            }
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Copter::gps_checks(bool display_failure)
{
    // check GPS
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }
    return true;
}

bool AP_Arming_Copter::fence_checks(bool display_failure)
{
    #if AC_FENCE == ENABLED
    // check fence is initialised
    const char *fail_msg = nullptr;
    if (!copter.fence.pre_arm_check(fail_msg)) {
        if (display_failure && fail_msg != nullptr) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", fail_msg);
        }
        return false;
    }
    #endif
    return true;
}

bool AP_Arming_Copter::ins_checks(bool display_failure)
{
    bool ret = AP_Arming::ins_checks(display_failure);

    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {

        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: gyros still settling");
            }
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Copter::board_voltage_checks(bool display_failure)
{
#if HAL_HAVE_BOARD_VOLTAGE
    // check board voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (hal.analogin->board_voltage() < BOARD_VOLTAGE_MIN || hal.analogin->board_voltage() > BOARD_VOLTAGE_MAX) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check Board Voltage");
            }
            return false;
        }
    }
#endif

    Parameters &g = copter.g;

    // check battery voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (copter.failsafe.battery || (!copter.ap.usb_connected && copter.battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check Battery");
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming_Copter::parameter_checks(bool display_failure)
{
    // check various parameter values
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if (copter.check_duplicate_auxsw()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Duplicate Aux Switch Options");
            }
            return false;
        }

        // failsafe parameter checks
        if (copter.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check FS_THR_VALUE");
                }
                return false;
            }
        }

        // lean angle parameter check
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check ANGLE_MAX");
            }
            return false;
        }

        // acro balance parameter check
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: ACRO_BAL_ROLL/PITCH");
            }
            return false;
        }

        #if RANGEFINDER_ENABLED == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (copter.optflow.enabled() && !copter.rangefinder.pre_arm_check()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: check range finder");
            }
            return false;
        }
        #endif

        #if FRAME_CONFIG == HELI_FRAME
        // check helicopter parameters
        if (!copter.motors->parameter_check(display_failure)) {
            return false;
        }
        #endif // HELI_FRAME

        // check for missing terrain data
        if (!pre_arm_terrain_check(display_failure)) {
            return false;
        }

        // check adsb avoidance failsafe
        if (copter.failsafe.adsb) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: ADSB threat detected");
            }
            return false;
        }

        // check for something close to vehicle
        if (!pre_arm_proximity_check(display_failure)) {
            return false;
        }
    }

    return true;
}

// check motor setup was successful
bool AP_Arming_Copter::motor_checks(bool display_failure)
{
    // check motors initialised  correctly
    if (!copter.motors->initialised_ok()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: check firmware or FRAME_CLASS");
        }
        return false;
    }
    return true;
}

bool AP_Arming_Copter::pilot_throttle_checks(bool display_failure)
{
    // check throttle is above failsafe throttle
    // this is near the bottom to allow other failures to be displayed before checking pilot throttle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RC)) {
        if (copter.g.failsafe_throttle != FS_THR_DISABLED && copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
            if (display_failure) {
                #if FRAME_CONFIG == HELI_FRAME
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Collective below Failsafe");
                #else
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Throttle below Failsafe");
                #endif
            }
            return false;
        }
    }

    return true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
void AP_Arming_Copter::pre_arm_rc_checks(const bool display_failure)
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if (copter.ap.pre_arm_rc_check) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((checks_to_perform != ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    const RC_Channel *channels[] = {
        copter.channel_roll,
        copter.channel_pitch,
        copter.channel_throttle,
        copter.channel_yaw
    };
    const char *channel_names[] = { "Roll", "Pitch", "Throttle", "Yaw" };

    for (uint8_t i=0; i<ARRAY_SIZE(channels);i++) {
        const RC_Channel *channel = channels[i];
        const char *channel_name = channel_names[i];
        // check if radio has been calibrated
        if (!channel->min_max_configured()) {
            if (display_failure) {
                copter.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL,"PreArm: RC %s not configured", channel_name);
            }
            return;
        }
        if (channel->get_radio_min() > 1300) {
            if (display_failure) {
                copter.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL,"PreArm: %s radio min too high", channel_name);
            }
            return;
        }
        if (channel->get_radio_max() < 1700) {
            if (display_failure) {
                copter.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL,"PreArm: %s radio max too low", channel_name);
            }
            return;
        }
        if (i == 2) {
            // skip checking trim for throttle as older code did not check it
            continue;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            if (display_failure) {
                copter.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL,"PreArm: %s radio trim below min", channel_name);
            }
            return;
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            if (display_failure) {
                copter.gcs_send_text_fmt(MAV_SEVERITY_CRITICAL,"PreArm: %s radio trim above max", channel_name);
            }
            return;
        }
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Copter::pre_arm_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Nav Checks");
        }
        return false;
    }

    // check if flight mode requires GPS
    bool mode_requires_gps = copter.mode_requires_GPS(copter.control_mode);

    // check if fence requires GPS
    bool fence_requires_gps = false;
    #if AC_FENCE == ENABLED
    // if circular or polygon fence is enabled we need GPS
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
    #endif

    // return true if GPS is not required
    if (!mode_requires_gps && !fence_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
    if (!copter.position_ok()) {
        if (display_failure) {
            const char *reason = ahrs.prearm_failure_reason();
            if (reason) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
            } else {
                if (!mode_requires_gps && fence_requires_gps) {
                    // clarify to user why they need GPS in non-GPS flight mode
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Fence enabled, need 3D Fix");
                } else {
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Need 3D Fix");
                }
            }
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    _ahrs_navekf.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: EKF compass variance");
        }
        return false;
    }

    // check home and EKF origin are not too far
    if (copter.far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: EKF-home variance");
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // return true immediately if gps check is disabled
    if (!(checks_to_perform == ARMING_CHECK_ALL || checks_to_perform & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (copter.gps.get_hdop() > copter.g.gps_hdop_good) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: High GPS HDOP");
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // call parent gps checks
    if (!AP_Arming::gps_checks(display_failure)) {
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// check ekf attitude is acceptable
bool AP_Arming_Copter::pre_arm_ekf_attitude_check()
{
    // get ekf filter status
    nav_filter_status filt_status = _inav.get_filter_status();

    return filt_status.flags.attitude;
}

// check we have required terrain data
bool AP_Arming_Copter::pre_arm_terrain_check(bool display_failure)
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    // succeed if not using terrain data
    if (!copter.terrain_use()) {
        return true;
    }

    // check if terrain following is enabled, using a range finder but RTL_ALT is higher than rangefinder's max range
    // To-Do: modify RTL return path to fly at or above the RTL_ALT and remove this check

    if (copter.rangefinder_state.enabled && (copter.g.rtl_altitude > copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270))) {
        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: RTL_ALT above rangefinder max range");
        return false;
    }

    // show terrain statistics
    uint16_t terr_pending, terr_loaded;
    copter.terrain.get_statistics(terr_pending, terr_loaded);
    bool have_all_data = (terr_pending <= 0);
    if (!have_all_data && display_failure) {
        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Terrain data");
    }
    return have_all_data;
#else
    return true;
#endif
}

// check nothing is too close to vehicle
bool AP_Arming_Copter::pre_arm_proximity_check(bool display_failure)
{
#if PROXIMITY_ENABLED == ENABLED

    // return true immediately if no sensor present
    if (copter.g2.proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return true;
    }

    // return false if proximity sensor unhealthy
    if (copter.g2.proximity.get_status() < AP_Proximity::Proximity_Good) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: check proximity sensor");
        }
        return false;
    }

    // get closest object if we might use it for avoidance
#if AC_AVOID_ENABLED == ENABLED
    float angle_deg, distance;
    if (copter.avoid.proximity_avoidance_enabled() && copter.g2.proximity.get_closest_object(angle_deg, distance)) {
        // display error if something is within 60cm
        if (distance <= 0.6f) {
            if (display_failure) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Proximity %d deg, %4.2fm", (int)angle_deg, (double)distance);
            }
            return false;
        }
    }
#endif

    return true;
#else
    return true;
#endif
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool AP_Arming_Copter::arm_checks(bool display_failure, bool arming_from_gcs)
{
    #if LOGGING_ENABLED == ENABLED
    // start dataflash
    copter.start_logging();
    #endif

    // check accels and gyro are healthy
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {
        //check if accelerometers have calibrated and require reboot
        if (_ins.accel_cal_requires_reboot()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "PreArm: Accels calibrated requires reboot");
            }
            return false;
        }

        if (!_ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Accels not healthy");
            }
            return false;
        }
        if (!_ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Gyros not healthy");
            }
            return false;
        }
        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: gyros still settling");
            }
            return false;
        }
    }

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Waiting for Nav Checks");
        }
        return false;
    }

    // check compass health
    if (!_compass.healthy()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Compass not healthy");
        }
        return false;
    }

    if (_compass.is_calibrating()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Compass calibration running");
        }
        return false;
    }

    //check if compass has calibrated and requires reboot
    if (_compass.compass_cal_requires_reboot()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibrated requires reboot");
        }
        return false;
    }

    control_mode_t control_mode = copter.control_mode;

    // always check if the current mode allows arming
    if (!copter.mode_allows_arming(control_mode, arming_from_gcs)) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Mode not armable");
        }
        return false;
    }

    // always check gps
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

    // always check motors
    if (!motor_checks(display_failure)) {
        return false;
    }

    // if we are using motor interlock switch and it's enabled, fail to arm
    // skip check in Throw mode which takes control of the motor interlock
    if (copter.ap.using_interlock && copter.motors->get_interlock()) {
        gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Motor Interlock Enabled");
        return false;
    }

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    if (!copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        copter.set_motor_emergency_stop(false);
        // if we are using motor Estop switch, it must not be in Estop position
    } else if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && copter.ap.motor_emergency_stop){
        gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Motor Emergency Stopped");
        return false;
    }

    // succeed if arming checks are disabled
    if (checks_to_perform == ARMING_CHECK_NONE) {
        return true;
    }

    // baro checks
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_BARO)) {
        // baro health check
        if (!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Barometer not healthy");
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = _inav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref && (fabsf(_inav.get_altitude() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM)) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Altitude disparity");
            }
            return false;
        }
    }

    #if AC_FENCE == ENABLED
    // check vehicle is within fence
    const char *fail_msg = nullptr;
    if (!copter.fence.pre_arm_check(fail_msg)) {
        if (display_failure && fail_msg != nullptr) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Arm: %s", fail_msg);
        }
        return false;
    }
    #endif

    // check lean angle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Leaning");
            }
            return false;
        }
    }

    // check battery voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (copter.failsafe.battery || (!copter.ap.usb_connected && copter.battery.exhausted(copter.g.fs_batt_voltage, copter.g.fs_batt_mah))) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Check Battery");
            }
            return false;
        }
    }

    // check for missing terrain data
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        if (!pre_arm_terrain_check(display_failure)) {
            return false;
        }
    }

    // check adsb
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        if (copter.failsafe.adsb) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: ADSB threat detected");
            }
            return false;
        }
    }

    // check throttle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RC)) {
        // check throttle is not too low - must be above failsafe throttle
        if (copter.g.failsafe_throttle != FS_THR_DISABLED && copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
            if (display_failure) {
                #if FRAME_CONFIG == HELI_FRAME
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective below Failsafe");
                #else
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle below Failsafe");
                #endif
            }
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(arming_from_gcs && (control_mode == GUIDED || control_mode == GUIDED_NOGPS))) {
            // above top of deadband is too always high
            if (copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in()) > 0.0f) {
                if (display_failure) {
                    #if FRAME_CONFIG == HELI_FRAME
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
                    #else
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
                    #endif
                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((copter.mode_has_manual_throttle(control_mode) || control_mode == DRIFT) && copter.channel_throttle->get_control_in() > 0) {
                if (display_failure) {
                    #if FRAME_CONFIG == HELI_FRAME
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
                    #else
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
                    #endif
                }
                return false;
            }
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Safety Switch");
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

enum HomeState AP_Arming_Copter::home_status() const
{
    return copter.ap.home_state;
}

void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    if(copter.ap.pre_arm_check != b) {
        copter.ap.pre_arm_check = b;
        AP_Notify::flags.pre_arm_check = b;
    }
}

void AP_Arming_Copter::set_pre_arm_rc_check(bool b)
{
    if(copter.ap.pre_arm_rc_check != b) {
        copter.ap.pre_arm_rc_check = b;
    }
}

void AP_Arming_Copter::gcs_send_text(MAV_SEVERITY severity, const char *str)
{
    copter.gcs_send_text(severity, str);
}
