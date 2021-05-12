#include "Blimp.h"


// performs pre-arm checks. expects to be called at 1hz.
void AP_Arming_Blimp::update(void)
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
}

bool AP_Arming_Blimp::pre_arm_checks(bool display_failure)
{
    const bool passed = run_pre_arm_checks(display_failure);
    set_pre_arm_check(passed);
    return passed;
}

// perform pre-arm checks
//  return true if the checks pass successfully
bool AP_Arming_Blimp::run_pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (blimp.motors->armed()) {
        return true;
    }

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) &&
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP)) {
        check_failed(display_failure, "Interlock/E-Stop Conflict");
        return false;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.  This check to be repeated,
    // as state can change at any time.
    if (blimp.ap.using_interlock && blimp.ap.motor_interlock_switch) {
        check_failed(display_failure, "Motor Interlock Enabled");
    }

    // if pre arm checks are disabled run only the mandatory checks
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }

    return fence_checks(display_failure)
           & parameter_checks(display_failure)
           & motor_checks(display_failure)
           & pilot_throttle_checks(display_failure)
           & gcs_failsafe_check(display_failure)
           & alt_checks(display_failure)
           & AP_Arming::pre_arm_checks(display_failure);
}

bool AP_Arming_Blimp::barometer_checks(bool display_failure)
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
        nav_filter_status filt_status = blimp.inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(blimp.inertial_nav.get_altitude() - blimp.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(ARMING_CHECK_BARO, display_failure, "Altitude disparity");
                ret = false;
            }
        }
    }
    return ret;
}

bool AP_Arming_Blimp::compass_checks(bool display_failure)
{
    bool ret = AP_Arming::compass_checks(display_failure);

    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_COMPASS)) {
        // check compass offsets have been set.  AP_Arming only checks
        // this if learning is off; Blimp *always* checks.
        char failure_msg[50] = {};
        if (!AP::compass().configured(failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_COMPASS, display_failure, "%s", failure_msg);
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Blimp::ins_checks(bool display_failure)
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

bool AP_Arming_Blimp::board_voltage_checks(bool display_failure)
{
    if (!AP_Arming::board_voltage_checks(display_failure)) {
        return false;
    }

    // check battery voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (blimp.battery.has_failsafed()) {
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

bool AP_Arming_Blimp::parameter_checks(bool display_failure)
{
    // check various parameter values
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {

        // failsafe parameter checks
        if (blimp.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (blimp.channel_down->get_radio_min() <= blimp.g.failsafe_throttle_value+10 || blimp.g.failsafe_throttle_value < 910) {
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }
        if (blimp.g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            // FS_GCS_ENABLE == 2 has been removed
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "FS_GCS_ENABLE=2 removed, see FS_OPTIONS");
        }

        // lean angle parameter check
        if (blimp.aparm.angle_max < 1000 || blimp.aparm.angle_max > 8000) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check ANGLE_MAX");
            return false;
        }

        // pilot-speed-up parameter check
        if (blimp.g.pilot_speed_up <= 0) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check PILOT_SPEED_UP");
            return false;
        }
    }

    return true;
}

// check motor setup was successful
bool AP_Arming_Blimp::motor_checks(bool display_failure)
{
    // check motors initialised  correctly
    if (!blimp.motors->initialised_ok()) {
        check_failed(display_failure, "Check firmware or FRAME_CLASS");
        return false;
    }

    // further checks enabled with parameters
    if (!check_enabled(ARMING_CHECK_PARAMETERS)) {
        return true;
    }

    return true;
}

bool AP_Arming_Blimp::pilot_throttle_checks(bool display_failure)
{
    // check throttle is above failsafe throttle
    // this is near the bottom to allow other failures to be displayed before checking pilot throttle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RC)) {
        if (blimp.g.failsafe_throttle != FS_THR_DISABLED && blimp.channel_down->get_radio_in() < blimp.g.failsafe_throttle_value) {
            const char *failmsg = "Throttle below Failsafe";
            check_failed(ARMING_CHECK_RC, display_failure, "%s", failmsg);
            return false;
        }
    }

    return true;
}

bool AP_Arming_Blimp::rc_calibration_checks(bool display_failure)
{
    return true;
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Blimp::gps_checks(bool display_failure)
{
    // run mandatory gps checks first
    if (!mandatory_gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check if flight mode requires GPS
    bool mode_requires_gps = blimp.flightmode->requires_GPS();


    // return true if GPS is not required
    if (!mode_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // return true immediately if gps check is disabled
    if (!(checks_to_perform == ARMING_CHECK_ALL || checks_to_perform & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (blimp.gps.get_hdop() > blimp.g.gps_hdop_good) {
        check_failed(ARMING_CHECK_GPS, display_failure, "High GPS HDOP");
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
bool AP_Arming_Blimp::pre_arm_ekf_attitude_check()
{
    // get ekf filter status
    nav_filter_status filt_status = blimp.inertial_nav.get_filter_status();

    return filt_status.flags.attitude;
}

// performs mandatory gps checks.  returns true if passed
bool AP_Arming_Blimp::mandatory_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();
    char failure_msg[50] = {};
    if (!ahrs.pre_arm_check(false, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // check if flight mode requires GPS
    bool mode_requires_gps = blimp.flightmode->requires_GPS();

    if (mode_requires_gps) {
        if (!blimp.position_ok()) {
            // vehicle level position estimate checks
            check_failed(display_failure, "Need Position Estimate");
            return false;
        }
    } else  {
        // return true if GPS is not required
        return true;
    }

    // if we got here all must be ok
    return true;
}

// Check GCS failsafe
bool AP_Arming_Blimp::gcs_failsafe_check(bool display_failure)
{
    if (blimp.failsafe.gcs) {
        check_failed(display_failure, "GCS failsafe on");
        return false;
    }
    return true;
}

// performs altitude checks.  returns true if passed
bool AP_Arming_Blimp::alt_checks(bool display_failure)
{
    // always EKF altitude estimate
    if (!blimp.flightmode->has_manual_throttle() && !blimp.ekf_alt_ok()) {
        check_failed(display_failure, "Need Alt Estimate");
        return false;
    }

    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool AP_Arming_Blimp::arm_checks(AP_Arming::Method method)
{
    return AP_Arming::arm_checks(method);
}

// mandatory checks that will be run if ARMING_CHECK is zero or arming forced
bool AP_Arming_Blimp::mandatory_checks(bool display_failure)
{
    // call mandatory gps checks and update notify status because regular gps checks will not run
    bool result = mandatory_gps_checks(display_failure);
    AP_Notify::flags.pre_arm_gps_check = result;

    // call mandatory alt check
    if (!alt_checks(display_failure)) {
        result = false;
    }

    return result;
}

void AP_Arming_Blimp::set_pre_arm_check(bool b)
{
    blimp.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}

bool AP_Arming_Blimp::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (blimp.motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Arming motors"); //MIR kept in - usually only in SITL

    AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    blimp.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        AP::logger().Write_Event(LogEvent::EKF_ALT_RESET);

        // we have reset height, so arming height is zero
        blimp.arming_altitude_m = 0;
    } else if (!ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!blimp.set_home_to_current_location(false)) {
            // ignore failure
        }

        // remember the height when we armed
        blimp.arming_altitude_m = blimp.inertial_nav.get_altitude() * 0.01;
    }

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

    // finally actually arm the motors
    blimp.motors->armed(true);

    // log flight mode in case it was changed while vehicle was disarmed
    AP::logger().Write_Mode((uint8_t)blimp.control_mode, blimp.control_mode_reason);

    // perf monitor ignores delay due to arming
    AP::scheduler().perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    blimp.arm_time_ms = millis();

    // Start the arming delay
    blimp.ap.in_arming_delay = true;

    // return success
    return true;
}

// arming.disarm - disarm motors
bool AP_Arming_Blimp::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // return immediately if we are already disarmed
    if (!blimp.motors->armed()) {
        return true;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors"); //MIR keeping in - usually only in SITL


    AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    // save compass offsets learned by the EKF if enabled
    Compass &compass = AP::compass();
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // send disarm command to motors
    blimp.motors->armed(false);

    AP::logger().set_vehicle_armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    blimp.ap.in_arming_delay = false;

    return true;
}
