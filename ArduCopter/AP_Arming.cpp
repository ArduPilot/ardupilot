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

    pre_arm_checks(display_fail);
    set_pre_arm_check(all_enabled_checks_passing());
}

void AP_Arming_Copter::pre_arm_checks(bool report)
{
    // exit immediately if already armed
    if (copter.motors->armed()) {
        return;
    }
    AP_Arming::pre_arm_checks(report);
}

// perform pre-arm checks
//  return true if the checks pass successfully
void AP_Arming_Copter::_pre_arm_checks()
{
    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_INTERLOCK) &&
        rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_ESTOP)){
        check_failed(ARMING_CHECK_NONE, "Interlock/E-Stop Conflict");
        return;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.  This check to be repeated,
    // as state can change at any time.
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(ARMING_CHECK_NONE, "Motor Interlock Enabled");
    }

    fence_checks();
    parameter_checks();
    motor_checks();
    pilot_throttle_checks();

    AP_Arming::_pre_arm_checks();
}

void AP_Arming_Copter::barometer_checks()
{
    AP_Arming::barometer_checks();

    // check Baro
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = copter.inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(copter.inertial_nav.get_altitude() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(ARMING_CHECK_BARO, "Altitude disparity");
            }
        }
}

void AP_Arming_Copter::compass_checks()
{
    AP_Arming::compass_checks();

        // check compass offsets have been set.  AP_Arming only checks
        // this if learning is off; Copter *always* checks.
        if (!AP::compass().configured()) {
            check_failed(ARMING_CHECK_COMPASS, "Compass not calibrated");
        }
}

void AP_Arming_Copter::fence_checks()
{
    #if AC_FENCE == ENABLED
    // check fence is initialised
    const char *fail_msg = nullptr;
    if (!copter.fence.pre_arm_check(fail_msg)) {
        if (fail_msg == nullptr) {
            check_failed(ARMING_CHECK_NONE, "Check fence");
        } else {
            check_failed(ARMING_CHECK_NONE, "%s", fail_msg);
        }
    }
    #endif
}

void AP_Arming_Copter::ins_checks()
{
    AP_Arming::ins_checks();

    // get ekf attitude (if bad, it's usually the gyro biases)
    pre_arm_ekf_attitude_check();
}

void AP_Arming_Copter::board_voltage_checks()
{
    AP_Arming::board_voltage_checks();

        if (copter.battery.has_failsafed()) {
            check_failed(ARMING_CHECK_VOLTAGE, "Battery failsafe");
        }

        // call parent battery checks
        AP_Arming::battery_checks();
}

void AP_Arming_Copter::parameter_checks()
{
        // ensure all rc channels have different functions
        if (rc().duplicate_options_exist()) {
            check_failed(ARMING_CHECK_PARAMETERS, "Duplicate Aux Switch Options");
            return;
        }

        // failsafe parameter checks
        if (copter.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                check_failed(ARMING_CHECK_PARAMETERS, "Check FS_THR_VALUE");
                return;
            }
        }

        // lean angle parameter check
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            check_failed(ARMING_CHECK_PARAMETERS, "Check ANGLE_MAX");
            return;
        }

        // acro balance parameter check
#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            check_failed(ARMING_CHECK_PARAMETERS, "ACRO_BAL_ROLL/PITCH");
            return;
        }
#endif

        #if RANGEFINDER_ENABLED == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (copter.optflow.enabled() && !copter.rangefinder.pre_arm_check()) {
            check_failed(ARMING_CHECK_PARAMETERS, "check range finder");
            return;
        }
        #endif

        #if FRAME_CONFIG == HELI_FRAME
        // check helicopter parameters
        copter.motors->parameter_check();

        // Inverted flight feature disabled for Heli Single and Dual frames
        if (copter.g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_QUAD &&
            rc().find_channel_for_option(RC_Channel::aux_func_t::INVERTED) != nullptr) {
            check_failed(ARMING_CHECK_PARAMETERS, "Inverted flight option not supported");
            return;
        }
        #endif // HELI_FRAME

        // check for missing terrain data
        pre_arm_terrain_check();

        // check adsb avoidance failsafe
#if ADSB_ENABLED == ENABLE
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, "ADSB threat detected");
            return;
        }
#endif

        // check for something close to vehicle
        pre_arm_proximity_check();

        // Check for 0 value PID's - some items can / should be 0 and as such are not checked.
        // If the ATC_RAT_*_FF is non zero then the corresponding ATC_RAT_* PIDS can be 0.
        if (is_zero(copter.pos_control->get_pos_xy_p().kP())) {
            parameter_checks_pid_warning_message("PSC_POSXY_P");
        } else if (is_zero(copter.pos_control->get_pos_z_p().kP())) {
            parameter_checks_pid_warning_message("PSC_POSZ_P");
        } else if (is_zero(copter.pos_control->get_vel_z_p().kP())) {
            parameter_checks_pid_warning_message("PSC_VELZ_P");
        } else if (is_zero(copter.pos_control->get_accel_z_pid().kP())) {
            parameter_checks_pid_warning_message("PSC_ACCZ_P");
        } else if (is_zero(copter.pos_control->get_accel_z_pid().kI())) {
            parameter_checks_pid_warning_message("PSC_ACCZ_I");
        } else if (is_zero(copter.attitude_control->get_rate_roll_pid().kP()) && is_zero(copter.attitude_control->get_rate_roll_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_RLL_P");
        } else if (is_zero(copter.attitude_control->get_rate_roll_pid().kI()) && is_zero(copter.attitude_control->get_rate_roll_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_RLL_I");
        } else if (is_zero(copter.attitude_control->get_rate_roll_pid().kD()) && is_zero(copter.attitude_control->get_rate_roll_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_RLL_D");
        } else if (is_zero(copter.attitude_control->get_rate_pitch_pid().kP()) && is_zero(copter.attitude_control->get_rate_pitch_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_PIT_P");
        } else if (is_zero(copter.attitude_control->get_rate_pitch_pid().kI()) && is_zero(copter.attitude_control->get_rate_pitch_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_PIT_I");
        } else if (is_zero(copter.attitude_control->get_rate_pitch_pid().kD()) && is_zero(copter.attitude_control->get_rate_pitch_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_PIT_D");
        } else if (is_zero(copter.attitude_control->get_rate_yaw_pid().kP()) && is_zero(copter.attitude_control->get_rate_yaw_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_YAW_P");
        } else if (is_zero(copter.attitude_control->get_rate_yaw_pid().kI()) && is_zero(copter.attitude_control->get_rate_yaw_pid().ff())) {
            parameter_checks_pid_warning_message("ATC_RAT_YAW_I");
        } else if (is_zero(copter.attitude_control->get_angle_pitch_p().kP())) {
            parameter_checks_pid_warning_message("ATC_ANG_PIT_P");
        } else if (is_zero(copter.attitude_control->get_angle_roll_p().kP())) {
            parameter_checks_pid_warning_message("ATC_ANG_RLL_P");
        } else if (is_zero(copter.attitude_control->get_angle_yaw_p().kP())) {
            parameter_checks_pid_warning_message("ATC_ANG_YAW_P");
        }
}

void AP_Arming_Copter::parameter_checks_pid_warning_message(const char *error_msg)
{
    check_failed(ARMING_CHECK_PARAMETERS, "Check PIDs - %s", error_msg);
}

// check motor setup was successful
void AP_Arming_Copter::motor_checks()
{
    // check motors initialised  correctly
    if (!copter.motors->initialised_ok()) {
        check_failed(ARMING_CHECK_NONE, "check firmware or FRAME_CLASS");
        return;
    }
}

void AP_Arming_Copter::pilot_throttle_checks()
{
    // check throttle is above failsafe throttle
    // this is near the bottom to allow other failures to be displayed before checking pilot throttle
        if (copter.g.failsafe_throttle != FS_THR_DISABLED && copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
            #if FRAME_CONFIG == HELI_FRAME
            const char *failmsg = "Collective below Failsafe";
            #else
            const char *failmsg = "Throttle below Failsafe";
            #endif
            check_failed(ARMING_CHECK_RC, failmsg);
        }
}

void AP_Arming_Copter::rc_calibration_checks()
{
    const RC_Channel *channels[] = {
        copter.channel_roll,
        copter.channel_pitch,
        copter.channel_throttle,
        copter.channel_yaw
    };

    rc_checks_copter_sub(channels);

    AP_Arming::rc_calibration_checks();

    copter.ap.pre_arm_rc_check = !(failing_checks & ARMING_CHECK_RC);
}

// performs pre_arm gps related checks and returns true if passed
void AP_Arming_Copter::gps_checks()
{
    AP_Notify::flags.pre_arm_gps_check = false;

    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        check_failed(ARMING_CHECK_NONE, "Waiting for Nav Checks");
        return;
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
        return;
    }

    // ensure GPS is ok
    if (!copter.position_ok()) {
        const char *reason = ahrs.prearm_failure_reason();
        if (reason == nullptr) {
            if (!mode_requires_gps && fence_requires_gps) {
                // clarify to user why they need GPS in non-GPS flight mode
                reason = "Fence enabled, need 3D Fix";
            } else {
                reason = "Need 3D Fix";
            }
        }
        check_failed(ARMING_CHECK_NONE, "%s", reason);
        return;
    }

    // check for GPS glitch (as reported by EKF)
    nav_filter_status filt_status;
    if (ahrs.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            check_failed(ARMING_CHECK_NONE, "GPS glitching");
            return;
        }
    }

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
        check_failed(ARMING_CHECK_NONE, "EKF compass variance");
    }

    // check home and EKF origin are not too far
    if (copter.far_from_EKF_origin(ahrs.get_home())) {
        check_failed(ARMING_CHECK_NONE, "EKF-home variance");
        return;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (copter.gps.get_hdop() > copter.g.gps_hdop_good) {
        check_failed(ARMING_CHECK_GPS, "High GPS HDOP");
        return;
    }

    // call parent gps checks
    AP_Arming::gps_checks();

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = !(failing_checks & ARMING_CHECK_GPS);
}

// check ekf attitude is acceptable
void AP_Arming_Copter::pre_arm_ekf_attitude_check()
{
    // get ekf filter status
    nav_filter_status filt_status = copter.inertial_nav.get_filter_status();

    if (!filt_status.flags.attitude) {
        check_failed(ARMING_CHECK_INS, "EKF attitude is bad");
    }
}

// check we have required terrain data
void AP_Arming_Copter::pre_arm_terrain_check()
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    // succeed if not using terrain data
    if (!copter.terrain_use()) {
        return;
    }

    // check if terrain following is enabled, using a range finder but RTL_ALT is higher than rangefinder's max range
    // To-Do: modify RTL return path to fly at or above the RTL_ALT and remove this check

    if (copter.rangefinder_state.enabled && (copter.g.rtl_altitude > copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270))) {
        check_failed(ARMING_CHECK_PARAMETERS, "RTL_ALT above rangefinder max range");
        return;
    }

    // show terrain statistics
    uint16_t terr_pending, terr_loaded;
    copter.terrain.get_statistics(terr_pending, terr_loaded);
    bool have_all_data = (terr_pending <= 0);
    if (!have_all_data) {
        check_failed(ARMING_CHECK_PARAMETERS, "Waiting for Terrain data");
    }
#endif
}

// check nothing is too close to vehicle
void AP_Arming_Copter::pre_arm_proximity_check()
{
#if PROXIMITY_ENABLED == ENABLED

    // return true immediately if no sensor present
    if (copter.g2.proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return;
    }

    // return false if proximity sensor unhealthy
    if (copter.g2.proximity.get_status() < AP_Proximity::Proximity_Good) {
        check_failed(ARMING_CHECK_PARAMETERS, "check proximity sensor");
        return;
    }

    // get closest object if we might use it for avoidance
#if AC_AVOID_ENABLED == ENABLED
    float angle_deg, distance;
    if (copter.avoid.proximity_avoidance_enabled() && copter.g2.proximity.get_closest_object(angle_deg, distance)) {
        // display error if something is within 60cm
        if (distance <= 0.6f) {
            check_failed(ARMING_CHECK_PARAMETERS, "Proximity %d deg, %4.2fm", (int)angle_deg, (double)distance);
            return;
        }
    }
#endif
#endif
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
void AP_Arming_Copter::arm_checks(AP_Arming::ArmingMethod method)
{
    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        check_failed(ARMING_CHECK_NONE, "AHRS not healthy");
        return;
    }

    const Compass &_compass = AP::compass();
#ifndef ALLOW_ARM_NO_COMPASS
    // check compass health
    if (!_compass.healthy()) {
        check_failed(ARMING_CHECK_NONE, "Compass not healthy");
        return;
    }
#endif

    if (_compass.is_calibrating()) {
        check_failed(ARMING_CHECK_NONE, "Compass calibration running");
        return;
    }

    //check if compass has calibrated and requires reboot
    if (_compass.compass_cal_requires_reboot()) {
        check_failed(ARMING_CHECK_NONE, "Compass calibrated requires reboot");
        return;
    }

    control_mode_t control_mode = copter.control_mode;

    // always check if the current mode allows arming
    if (!copter.flightmode->allows_arming(AP_Arming::ArmingMethod::MAVLINK)) {
        check_failed(ARMING_CHECK_NONE, "Mode not armable");
        return;
    }

    // always check motors
    motor_checks();

    // if we are using motor interlock switch and it's enabled, fail to arm
    // skip check in Throw mode which takes control of the motor interlock
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(ARMING_CHECK_NONE, "Motor Interlock Enabled");
        return;
    }

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    if (!rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_ESTOP)){
        copter.set_motor_emergency_stop(false);
        // if we are using motor Estop switch, it must not be in Estop position
    } else if (rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_ESTOP) && copter.ap.motor_emergency_stop){
        check_failed(ARMING_CHECK_NONE, "Motor Emergency Stopped");
        return;
    }

    // succeed if arming checks are disabled
    if (checks_to_perform == ARMING_CHECK_NONE) {
        return;
    }

    // check lean angle
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            check_failed(ARMING_CHECK_INS, "Leaning");
            return;
        }

    // check adsb
#if ADSB_ENABLED == ENABLE
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, "ADSB threat detected");
            return;
        }
#endif

    // check throttle
         #if FRAME_CONFIG == HELI_FRAME
        const char *rc_item = "Collective";
        #else
        const char *rc_item = "Throttle";
        #endif
        // check throttle is not too low - must be above failsafe throttle
        if (copter.g.failsafe_throttle != FS_THR_DISABLED && copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
            check_failed(ARMING_CHECK_RC, "%s below failsafe", rc_item);
            return;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(method == AP_Arming::ArmingMethod::MAVLINK && (control_mode == GUIDED || control_mode == GUIDED_NOGPS))) {
            // above top of deadband is too always high
            if (copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in()) > 0.0f) {
                check_failed(ARMING_CHECK_RC, "%s too high", rc_item);
                return;
            }
            // in manual modes throttle must be at zero
            if ((copter.flightmode->has_manual_throttle() || control_mode == DRIFT) && copter.channel_throttle->get_control_in() > 0) {
                check_failed(ARMING_CHECK_RC, "%s too high", rc_item);
                return;
            }
        }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        check_failed(ARMING_CHECK_NONE, "Safety Switch");
        return;
    }

    // superclass method should always be the last thing called; it
    // has side-effects which would need to be cleaned up if one of
    // our arm checks failed
    AP_Arming::arm_checks(method);
}

void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    copter.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}
