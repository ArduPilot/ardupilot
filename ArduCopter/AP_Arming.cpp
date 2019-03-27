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
bool AP_Arming_Copter::all_checks_passing(AP_Arming::Method method)
{
    set_pre_arm_check(pre_arm_checks(true));

    return copter.ap.pre_arm_check && arm_checks(true, method);
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
    if (rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_INTERLOCK) &&
        rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_ESTOP)){
        check_failed(ARMING_CHECK_NONE, display_failure, "Interlock/E-Stop Conflict");
        return false;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.  This check to be repeated,
    // as state can change at any time.
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Motor Interlock Enabled");
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
        nav_filter_status filt_status = copter.inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(copter.inertial_nav.get_altitude() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(ARMING_CHECK_BARO, display_failure, "Altitude disparity");
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
        if (!AP::compass().configured()) {
            check_failed(ARMING_CHECK_COMPASS, display_failure, "Compass not calibrated");
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Copter::ins_checks(bool display_failure)
{
    bool ret = AP_Arming::ins_checks(display_failure);

    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {

        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            check_failed(ARMING_CHECK_INS, display_failure, "EKF attitude is bad");
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

    // check battery voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (copter.battery.has_failsafed()) {
            check_failed(ARMING_CHECK_VOLTAGE, display_failure, "Battery failsafe");
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

        // ensure all rc channels have different functions
        if (rc().duplicate_options_exist()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Duplicate Aux Switch Options");
            return false;
        }

        // failsafe parameter checks
        if (copter.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }

        // lean angle parameter check
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check ANGLE_MAX");
            return false;
        }

        // acro balance parameter check
#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "ACRO_BAL_ROLL/PITCH");
            return false;
        }
#endif

        #if RANGEFINDER_ENABLED == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (copter.optflow.enabled() && !copter.rangefinder.pre_arm_check()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "check range finder");
            return false;
        }
        #endif

        #if FRAME_CONFIG == HELI_FRAME
        // check helicopter parameters
        if (!copter.motors->parameter_check(display_failure)) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Heli motors checks failed");
            return false;
        }
        // Inverted flight feature disabled for Heli Single and Dual frames
        if (copter.g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_QUAD &&
            rc().find_channel_for_option(RC_Channel::aux_func_t::INVERTED) != nullptr) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Inverted flight option not supported");
            return false;
        }
        // Ensure an Aux Channel is configured for motor interlock
        if (rc().find_channel_for_option(RC_Channel::aux_func_t::MOTOR_INTERLOCK) == nullptr) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Motor Interlock not configured");
            return false;
        }

        #endif // HELI_FRAME

        // check for missing terrain data
        if (!pre_arm_terrain_check(display_failure)) {
            return false;
        }

        // check adsb avoidance failsafe
#if ADSB_ENABLED == ENABLE
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "ADSB threat detected");
            return false;
        }
#endif

        // check for something close to vehicle
        if (!pre_arm_proximity_check(display_failure)) {
            return false;
        }

        // ensure controllers are OK with us arming:
        char failure_msg[50];
        if (!copter.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
        if (!copter.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
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
        check_failed(ARMING_CHECK_NONE, display_failure, "check firmware or FRAME_CLASS");
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
            #if FRAME_CONFIG == HELI_FRAME
            const char *failmsg = "Collective below Failsafe";
            #else
            const char *failmsg = "Throttle below Failsafe";
            #endif
            check_failed(ARMING_CHECK_RC, display_failure, failmsg);
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
    AP_Notify::flags.pre_arm_gps_check = false;

    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "AHRS not healthy");
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
        const char *reason = ahrs.prearm_failure_reason();
        if (reason == nullptr) {
            if (!mode_requires_gps && fence_requires_gps) {
                // clarify to user why they need GPS in non-GPS flight mode
                reason = "Fence enabled, need 3D Fix";
            } else {
                reason = "Need 3D Fix";
            }
        }
        check_failed(ARMING_CHECK_NONE, display_failure, "%s", reason);
        return false;
    }

    // check for GPS glitch (as reported by EKF)
    nav_filter_status filt_status;
    if (ahrs.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            check_failed(ARMING_CHECK_NONE, display_failure, "GPS glitching");
            return false;
        }
    }

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
        check_failed(ARMING_CHECK_NONE, display_failure, "EKF compass variance");
        return false;
    }

    // check home and EKF origin are not too far
    if (copter.far_from_EKF_origin(ahrs.get_home())) {
        check_failed(ARMING_CHECK_NONE, display_failure, "EKF-home variance");
        return false;
    }

    // return true immediately if gps check is disabled
    if (!(checks_to_perform == ARMING_CHECK_ALL || checks_to_perform & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (copter.gps.get_hdop() > copter.g.gps_hdop_good) {
        check_failed(ARMING_CHECK_GPS, display_failure, "High GPS HDOP");
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
    nav_filter_status filt_status = copter.inertial_nav.get_filter_status();

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
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "RTL_ALT above rangefinder max range");
        return false;
    }

    // show terrain statistics
    uint16_t terr_pending, terr_loaded;
    copter.terrain.get_statistics(terr_pending, terr_loaded);
    bool have_all_data = (terr_pending <= 0);
    if (!have_all_data) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Waiting for Terrain data");
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
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "check proximity sensor");
        return false;
    }

    // get closest object if we might use it for avoidance
#if AC_AVOID_ENABLED == ENABLED
    float angle_deg, distance;
    if (copter.avoid.proximity_avoidance_enabled() && copter.g2.proximity.get_closest_object(angle_deg, distance)) {
        // display error if something is within 60cm
        if (distance <= 0.6f) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Proximity %d deg, %4.2fm", (int)angle_deg, (double)distance);
            return false;
        }
    }
#endif

#endif
    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool AP_Arming_Copter::arm_checks(bool display_failure, AP_Arming::Method method)
{
    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "AHRS not healthy");
        return false;
    }

    const Compass &_compass = AP::compass();
#ifndef ALLOW_ARM_NO_COMPASS
    // check compass health
    if (!_compass.healthy()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Compass not healthy");
        return false;
    }
#endif

    if (_compass.is_calibrating()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Compass calibration running");
        return false;
    }

    //check if compass has calibrated and requires reboot
    if (_compass.compass_cal_requires_reboot()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Compass calibrated requires reboot");
        return false;
    }

    control_mode_t control_mode = copter.control_mode;

    // always check if the current mode allows arming
    if (!copter.flightmode->allows_arming(method == AP_Arming::Method::MAVLINK)) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Mode not armable");
        return false;
    }

    // always check motors
    if (!motor_checks(display_failure)) {
        return false;
    }

    // if we are using motor interlock switch and it's enabled, fail to arm
    // skip check in Throw mode which takes control of the motor interlock
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Motor Interlock Enabled");
        return false;
    }

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    if (!rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_ESTOP)){
        SRV_Channels::set_emergency_stop(false);
        // if we are using motor Estop switch, it must not be in Estop position
    } else if (rc().find_channel_for_option(RC_Channel::aux_func::MOTOR_ESTOP) && SRV_Channels::get_emergency_stop()){
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
            check_failed(ARMING_CHECK_INS, display_failure, "Leaning");
            return false;
        }
    }

    // check adsb
#if ADSB_ENABLED == ENABLE
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "ADSB threat detected");
            return false;
        }
    }
#endif

    // check throttle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RC)) {
         #if FRAME_CONFIG == HELI_FRAME
        const char *rc_item = "Collective";
        #else
        const char *rc_item = "Throttle";
        #endif
        // check throttle is not too low - must be above failsafe throttle
        if (copter.g.failsafe_throttle != FS_THR_DISABLED && copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s below failsafe", rc_item);
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(method == AP_Arming::Method::MAVLINK && (control_mode == GUIDED || control_mode == GUIDED_NOGPS))) {
            // above top of deadband is too always high
            if (copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in()) > 0.0f) {
                check_failed(ARMING_CHECK_RC, display_failure, "%s too high", rc_item);
                return false;
            }
            // in manual modes throttle must be at zero
            if ((copter.flightmode->has_manual_throttle() || control_mode == DRIFT) && copter.channel_throttle->get_control_in() > 0) {
                check_failed(ARMING_CHECK_RC, display_failure, "%s too high", rc_item);
                return false;
            }
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Safety Switch");
        return false;
    }

    // superclass method should always be the last thing called; it
    // has side-effects which would need to be cleaned up if one of
    // our arm checks failed
    return AP_Arming::arm_checks(method);
}

void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    copter.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}
