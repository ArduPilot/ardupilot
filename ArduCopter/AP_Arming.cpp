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

    set_pre_arm_check(pre_arm_checks(display_fail));
}

// performs pre-arm checks and arming checks
bool AP_Arming_Copter::all_checks_passing(bool arming_from_gcs)
{
    set_pre_arm_check(pre_arm_checks(true));

    return copter.ap.pre_arm_check && arm_checks(true, arming_from_gcs);
}

// perform pre-arm checks
//  return true if the checks pass successfully
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
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Interlock/E-Stop Conflict");
        }
        return false;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.  This check to be repeated,
    // as state can change at any time.
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Motor Interlock Enabled");
        }
    }

    // succeed if pre arm checks are disabled
    if (checks_to_perform == ARMING_CHECK_NONE) {
        return true;
    }

    return fence_checks(display_failure)
        & parameter_checks(display_failure)
        & motor_checks(display_failure)
        & pilot_throttle_checks(display_failure) &
        AP_Arming::pre_arm_checks(display_failure);
}

bool AP_Arming_Copter::barometer_checks(bool display_failure)
{
    if (!AP_Arming::barometer_checks(display_failure)) {
        return false;
    }

    bool ret = true;
    // check Baro
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_BARO)) {
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = _inav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(_inav.get_altitude() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                if (display_failure) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Altitude disparity");
                }
                ret = false;
            }
        }
    }
    return ret;
}

bool AP_Arming_Copter::compass_checks(bool display_failure)
{
    bool ret = AP_Arming::compass_checks(display_failure);

    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_COMPASS)) {
        // check compass offsets have been set.  AP_Arming only checks
        // this if learning is off; Copter *always* checks.
        if (!_compass.configured()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Compass not calibrated");
            }
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Copter::fence_checks(bool display_failure)
{
    #if AC_FENCE == ENABLED
    // check fence is initialised
    const char *fail_msg = nullptr;
    if (!copter.fence.pre_arm_check(fail_msg)) {
        if (display_failure && fail_msg != nullptr) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s", fail_msg);
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
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: gyros still settling");
            }
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Copter::board_voltage_checks(bool display_failure)
{
    if (!AP_Arming::board_voltage_checks(display_failure)) {
        return false;
    }

    Parameters &g = copter.g;

    // check battery voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (copter.failsafe.battery) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Battery failsafe");
            }
            return false;
        }

        // all following checks are skipped if USB is connected
        if (copter.ap.usb_connected) {
            return true;
        }

        // check if battery is exhausted
        if (copter.battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check Battery");
            }
            return false;
        }

        // call parent battery checks
        if (!AP_Arming::battery_checks(display_failure)) {
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
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Duplicate Aux Switch Options");
            }
            return false;
        }

        // failsafe parameter checks
        if (copter.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check FS_THR_VALUE");
                }
                return false;
            }
        }

        // lean angle parameter check
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check ANGLE_MAX");
            }
            return false;
        }

        // acro balance parameter check
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: ACRO_BAL_ROLL/PITCH");
            }
            return false;
        }

        #if RANGEFINDER_ENABLED == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (copter.optflow.enabled() && !copter.rangefinder.pre_arm_check()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: check range finder");
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
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: ADSB threat detected");
            }
            return false;
        }

        // check for something close to vehicle
        if (!pre_arm_proximity_check(display_failure)) {
            return false;
        }

        // Check for 0 value PID's - some items can / should be 0 and as such are not checked.
        // If the ATC_RAT_*_FF is non zero then the corresponding ATC_RAT_* PIDS can be 0.
        if (is_zero(copter.g.p_pos_xy.kP())) {
            parameter_checks_pid_warning_message(display_failure, "POS_XY_P");
            return false;
        } else if (is_zero(copter.g.p_alt_hold.kP())) {
            parameter_checks_pid_warning_message(display_failure, "POS_Z_P");
            return false;
        } else if (is_zero(copter.g.p_vel_z.kP())) {
            parameter_checks_pid_warning_message(display_failure, "VEL_Z_P");
            return false;
        } else if (is_zero(copter.g.pid_accel_z.kP())) {
            parameter_checks_pid_warning_message(display_failure, "ACCEL_Z_P");
            return false;
        } else if (is_zero(copter.g.pid_accel_z.kI())) {
            parameter_checks_pid_warning_message(display_failure, "ACCEL_Z_I");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_roll_pid().kP()) && is_zero(copter.attitude_control->get_rate_roll_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_RLL_P");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_roll_pid().kI()) && is_zero(copter.attitude_control->get_rate_roll_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_RLL_I");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_roll_pid().kD()) && is_zero(copter.attitude_control->get_rate_roll_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_RLL_D");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_pitch_pid().kP()) && is_zero(copter.attitude_control->get_rate_pitch_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_PIT_P");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_pitch_pid().kI()) && is_zero(copter.attitude_control->get_rate_pitch_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_PIT_I");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_pitch_pid().kD()) && is_zero(copter.attitude_control->get_rate_pitch_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_PIT_D");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_yaw_pid().kP()) && is_zero(copter.attitude_control->get_rate_yaw_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_YAW_P");
            return false;
        } else if (is_zero(copter.attitude_control->get_rate_yaw_pid().kI()) && is_zero(copter.attitude_control->get_rate_yaw_pid().ff())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_RAT_YAW_I");
            return false;
        } else if (is_zero(copter.attitude_control->get_angle_pitch_p().kP())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_ANG_PIT_P");
            return false;
        } else if (is_zero(copter.attitude_control->get_angle_roll_p().kP())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_ANG_RLL_P");
            return false;
        } else if (is_zero(copter.attitude_control->get_angle_yaw_p().kP())) {
            parameter_checks_pid_warning_message(display_failure, "ATC_ANG_YAW_P");
            return false;
        }
    }

    return true;
}

void AP_Arming_Copter::parameter_checks_pid_warning_message(bool display_failure, const char *error_msg)
{
    if (display_failure) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check PIDs - %s", error_msg);
    }
}

// check motor setup was successful
bool AP_Arming_Copter::motor_checks(bool display_failure)
{
    // check motors initialised  correctly
    if (!copter.motors->initialised_ok()) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: check firmware or FRAME_CLASS");
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
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Collective below Failsafe");
                #else
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Throttle below Failsafe");
                #endif
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming_Copter::rc_calibration_checks(bool display_failure)
{
    const RC_Channel *channels[] = {
        copter.channel_roll,
        copter.channel_pitch,
        copter.channel_throttle,
        copter.channel_yaw
    };

    copter.ap.pre_arm_rc_check = rc_checks_copter_sub(display_failure, channels)
        & AP_Arming::rc_calibration_checks(display_failure);

    return copter.ap.pre_arm_rc_check;
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Copter::gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Nav Checks");
        }
        return false;
    }

    // check if flight mode requires GPS
    bool mode_requires_gps = copter.flightmode->requires_GPS();

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
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
            } else {
                if (!mode_requires_gps && fence_requires_gps) {
                    // clarify to user why they need GPS in non-GPS flight mode
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Fence enabled, need 3D Fix");
                } else {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Need 3D Fix");
                }
            }
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check for GPS glitch (as reported by EKF)
    nav_filter_status filt_status;
    if (_ahrs_navekf.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: GPS glitching");
            }
            return false;
        }
    }

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    _ahrs_navekf.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: EKF compass variance");
        }
        return false;
    }

    // check home and EKF origin are not too far
    if (copter.far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: EKF-home variance");
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
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: High GPS HDOP");
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // call parent gps checks
    if (!AP_Arming::gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
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
        gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: RTL_ALT above rangefinder max range");
        return false;
    }

    // show terrain statistics
    uint16_t terr_pending, terr_loaded;
    copter.terrain.get_statistics(terr_pending, terr_loaded);
    bool have_all_data = (terr_pending <= 0);
    if (!have_all_data && display_failure) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Terrain data");
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
            gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: check proximity sensor");
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
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Proximity %d deg, %4.2fm", (int)angle_deg, (double)distance);
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
    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Waiting for Nav Checks");
        }
        return false;
    }

    // check compass health
    if (!_compass.healthy()) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Compass not healthy");
        }
        return false;
    }

    if (_compass.is_calibrating()) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Compass calibration running");
        }
        return false;
    }

    //check if compass has calibrated and requires reboot
    if (_compass.compass_cal_requires_reboot()) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibrated requires reboot");
        }
        return false;
    }

    control_mode_t control_mode = copter.control_mode;

    // always check if the current mode allows arming
    if (!copter.flightmode->allows_arming(arming_from_gcs)) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Mode not armable");
        }
        return false;
    }

    // always check motors
    if (!motor_checks(display_failure)) {
        return false;
    }

    // if we are using motor interlock switch and it's enabled, fail to arm
    // skip check in Throw mode which takes control of the motor interlock
    if (copter.ap.using_interlock && copter.motors->get_interlock()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Motor Interlock Enabled");
        return false;
    }

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    if (!copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        copter.set_motor_emergency_stop(false);
        // if we are using motor Estop switch, it must not be in Estop position
    } else if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && copter.ap.motor_emergency_stop){
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Motor Emergency Stopped");
        return false;
    }

    // succeed if arming checks are disabled
    if (checks_to_perform == ARMING_CHECK_NONE) {
        return true;
    }

    // check lean angle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Leaning");
            }
            return false;
        }
    }

    // check adsb
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        if (copter.failsafe.adsb) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: ADSB threat detected");
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
                gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective below Failsafe");
                #else
                gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle below Failsafe");
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
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
                    #else
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
                    #endif
                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((copter.flightmode->has_manual_throttle() || control_mode == DRIFT) && copter.channel_throttle->get_control_in() > 0) {
                if (display_failure) {
                    #if FRAME_CONFIG == HELI_FRAME
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
                    #else
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
                    #endif
                }
                return false;
            }
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (display_failure) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Arm: Safety Switch");
        }
        return false;
    }

    // superclass method should always be the last thing called; it
    // has side-effects which would need to be cleaned up if one of
    // our arm checks failed
    return AP_Arming::arm_checks(arming_from_gcs);
}

enum HomeState AP_Arming_Copter::home_status() const
{
    return copter.ap.home_state;
}

void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    copter.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}
