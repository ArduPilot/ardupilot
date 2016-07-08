// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for copter
 */
#include "arming_checks.h"
#include "Copter.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Arming_Copter::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    AP_GROUPEND
};

// performs pre-arm checks. expects to be called at 1hz.
void AP_Arming_Copter::update_arming_checks()
{
    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = PREARM_DISPLAY_PERIOD/2;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= PREARM_DISPLAY_PERIOD) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }
}

// performs pre-arm checks and arming checks
bool AP_Arming_Copter::all_arming_checks_passing(bool arming_from_gcs)
{
  
    return (pre_arm_checks(true) && arm_checks(true, arming_from_gcs));
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
bool AP_Arming_Copter::pre_arm_checks(bool report)
{
    // exit immediately if already armed
    if (copter.motors.armed()) {
        AP_Notify::flags.pre_arm_check = true;
        return true;
    }

    // set notify LEDs based on gps checks
    uint64_t enabled_checks = 0;
    uint64_t passed_checks = 0;
    AP_Notify::flags.pre_arm_gps_check = gps_checks(false, enabled_checks, passed_checks);

    // call parent class checks
    if (!AP_Arming::pre_arm_checks(report)) {
        AP_Notify::flags.pre_arm_check = false;
        return false;
    }

    AP_Notify::flags.pre_arm_check = true;
    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool AP_Arming_Copter::arm_checks(bool report, bool arming_from_gcs)
{
#if LOGGING_ENABLED == ENABLED
    // start dataflash
    copter.start_logging();
#endif

    // always check if the current mode allows arming
    if (!copter.mode_allows_arming(copter.control_mode, arming_from_gcs)) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Arm: Mode not armable");
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::barometer_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::barometer_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = true;

    // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
    // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
    // that may differ from the baro height due to baro drift.
    nav_filter_status filt_status;
    filt_status = copter.inertial_nav.get_filter_status();
    bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
    if (using_baro_ref) {
        enabled_checks |= ARMING_CHECK_COPTER_BARO;
        passed_checks |= ARMING_CHECK_COPTER_BARO;
        group_is_disabled = false;
        if (fabsf(copter.inertial_nav.get_altitude() - barometer.get_altitude()) > COPTER_ARMING_CHECK_ALT_DISPARITY_MAX_CM) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Altitude disparity");
            }
            passed_checks &= ~ARMING_CHECK_COPTER_BARO;
            group_contains_failure = true;
        }
    }
    
    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }
    
    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::ins_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::ins_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false;

    // check ekf attitude, if bad it's usually the gyro biases have not settled
    enabled_checks |= ARMING_CHECK_COPTER_INS;
    passed_checks |= ARMING_CHECK_COPTER_INS;
    if (!copter.inertial_nav.get_filter_status().flags.attitude) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: gyros still settling");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_INS;
        group_contains_failure = true;
    }

    // check lean angle
    enabled_checks |= ARMING_CHECK_COPTER_INS_2;
    passed_checks |= ARMING_CHECK_COPTER_INS_2;
    if (degrees(acosf(copter.ahrs.cos_roll()*copter.ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Leaning");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_INS_2;
        group_contains_failure = true;
    }

    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }
    
    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::parameter_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::parameter_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false;  // some checks will be made either way here

    // radio failsafe parameter checks
    if (copter.g.failsafe_throttle) {
        enabled_checks |= ARMING_CHECK_COPTER_PARAMETERS;
        passed_checks |= ARMING_CHECK_COPTER_PARAMETERS;
        // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
        if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Check FS_THR_VALUE");
            }
            passed_checks &= ~ARMING_CHECK_COPTER_PARAMETERS;
            group_contains_failure = true;
        }
    }

    // lean angle parameter check
    enabled_checks |= ARMING_CHECK_COPTER_PARAMETERS_2;
    passed_checks |= ARMING_CHECK_COPTER_PARAMETERS_2;
    if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Check ANGLE_MAX");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_PARAMETERS_2;
        group_contains_failure = true;
    }

    // acro balance parameter check
    enabled_checks |= ARMING_CHECK_COPTER_PARAMETERS_3;
    passed_checks |= ARMING_CHECK_COPTER_PARAMETERS_3;
    if ((copter.g.acro_balance_roll > copter.attitude_control.get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control.get_angle_pitch_p().kP())) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Check ACRO_BAL_ROLL/PITCH");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_PARAMETERS_3;
        group_contains_failure = true;
    }
#if FRAME_CONFIG == HELI_FRAME
    // check helicopter parameters
    enabled_checks |= ARMING_CHECK_HELI_PARAMETERS;
    passed_checks |= ARMING_CHECK_HELI_PARAMETERS;
    if (!copter.motors.parameter_check(report)) {
        passed_checks &= ~ARMING_CHECK_HELI_PARAMETERS;
        group_contains_failure = true;
    }
#endif // HELI_FRAME
    
    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::compass_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::compass_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false; // there is always at least one check being run

    // compass is required for copter
    enabled_checks |= ARMING_CHECK_COPTER_COMPASS;
    passed_checks |= ARMING_CHECK_COPTER_COMPASS;
    if (!copter.compass.use_for_yaw()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Compass disabled");
        }
        passed_checks &= ARMING_CHECK_COPTER_COMPASS;
        group_contains_failure = true;
    }

    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::gps_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // check if flight mode requires GPS
    bool gps_required = copter.mode_requires_GPS(copter.control_mode);

#if AC_FENCE == ENABLED
    // if circular fence is enabled we need GPS
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) {
        gps_required = true;
    }
#endif

    // return true if GPS is not required
    if (!gps_required) {
        return ARMING_CHECK_PASSED;
    }

    bool group_contains_failure = false;
    bool group_is_disabled = false; // some checks will for sure be passed

    // ensure position is ok
    enabled_checks |= ARMING_CHECK_COPTER_GPS;
    passed_checks |= ARMING_CHECK_COPTER_GPS;
    if (!copter.position_ok()) {
        if (report) {
            const char *reason = copter.ahrs.prearm_failure_reason();
            if (reason) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
            } else {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Need 3D Fix");
            }
        }
        passed_checks &= ~ARMING_CHECK_COPTER_GPS;
        group_contains_failure = true;
    }

    // check EKF compass variance is below failsafe threshold
    enabled_checks |= ARMING_CHECK_COPTER_GPS_2;
    passed_checks |= ARMING_CHECK_COPTER_GPS_2;
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    copter.ahrs.get_NavEKF().getVariances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: EKF compass variance");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_GPS_2;
        group_contains_failure = true;
    }

    // check home and EKF origin are not too far
    enabled_checks |= ARMING_CHECK_COPTER_GPS_3;
    passed_checks |= ARMING_CHECK_COPTER_GPS_3;
    if (copter.far_from_EKF_origin(copter.ahrs.get_home())) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: EKF-home variance");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_GPS_3;
        group_contains_failure = true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    enabled_checks |= ARMING_CHECK_COPTER_GPS_3;
    passed_checks |= ARMING_CHECK_COPTER_GPS_3;
    if (copter.gps.get_hdop() > copter.g.gps_hdop_good) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: High GPS HDOP");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_GPS_3;
        group_contains_failure = true;
    }

#if AC_FENCE == ENABLED
    // check fence has not been breached
    enabled_checks |= ARMING_CHECK_COPTER_GPS_4;
    passed_checks |= ARMING_CHECK_COPTER_GPS_4;
    if(!copter.fence.pre_arm_check()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: check fence");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_GPS_4;
        group_contains_failure = true;
    }
#endif

    if (group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    if (group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::manual_transmitter_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::manual_transmitter_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false; // some checks will always be run
    
    // vehicle specific checks

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    enabled_checks |= ARMING_CHECK_COPTER_RC;
    passed_checks |= ARMING_CHECK_COPTER_RC;
    if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK) && copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Interlock/E-Stop Conflict");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_RC;
        group_contains_failure = true;
    }

    // if motor interlock aux switch is in use, switch needs to be in disabled position to arm
    enabled_checks |= ARMING_CHECK_COPTER_RC_2;
    passed_checks |= ARMING_CHECK_COPTER_RC_2;
    if (copter.ap.using_interlock && copter.motors.get_interlock()){
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Motor Interlock Enabled");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_RC_2;
        group_contains_failure = true;
    }

    // if we are using Motor Emergency Stop aux switch, check it is not enabled
    enabled_checks |= ARMING_CHECK_COPTER_RC_3;
    passed_checks |= ARMING_CHECK_COPTER_RC_3;
    if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && copter.ap.motor_emergency_stop){
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Motor Emergency Stopped");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_RC_3;
        group_contains_failure = true;
    }

    // To-Do: allow checks below this point to be disabled

    // check throttle is above failsafe throttle
    enabled_checks |= ARMING_CHECK_COPTER_RC_4;
    passed_checks |= ARMING_CHECK_COPTER_RC_4;
    if ((copter.g.failsafe_throttle != FS_THR_DISABLED) && (copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value)) {
        if (report) {
#if FRAME_CONFIG == HELI_FRAME
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Collective below Failsafe");
#else
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Throttle below Failsafe");
#endif
        }
        passed_checks &= ~ARMING_CHECK_COPTER_RC_4;
        group_contains_failure = true;
    }

    // check throttle is not too high - skips checks if arming from GCS in Guided
    enabled_checks |= ARMING_CHECK_COPTER_RC_5;
    passed_checks |= ARMING_CHECK_COPTER_RC_5;
    if (copter.control_mode != GUIDED) {
        // above top of deadband is too always high
        bool thr_too_high = copter.channel_throttle->get_control_in() > copter.get_takeoff_trigger_throttle();
        // in manual modes and Drift throttle must be at zero
        if ((copter.mode_has_manual_throttle(copter.control_mode) || copter.control_mode == DRIFT) && (copter.channel_throttle->get_control_in() > 0)) {
            thr_too_high = true;
        }
        if (thr_too_high) {
            if (report) {
#if FRAME_CONFIG == HELI_FRAME
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
#else
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
#endif
            }
            passed_checks &= ~ARMING_CHECK_COPTER_RC_5;
            group_contains_failure = true;
        }
    }

    enabled_checks |= ARMING_CHECK_COPTER_RC_6;
    passed_checks |= ARMING_CHECK_COPTER_RC_6;
    // check if radio has been calibrated
    if (!copter.channel_throttle->get_radio_min() || !copter.channel_throttle->get_radio_max()) {
        passed_checks &= ~ARMING_CHECK_COPTER_RC_6;
        group_contains_failure = true;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (copter.channel_roll->get_radio_min() > 1300 || copter.channel_roll->get_radio_max() < 1700 || copter.channel_pitch->get_radio_min() > 1300 || copter.channel_pitch->get_radio_max() < 1700) {
        passed_checks &= ~ARMING_CHECK_COPTER_RC_6;
        group_contains_failure = true;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (copter.channel_throttle->get_radio_min() > 1300 || copter.channel_throttle->get_radio_max() < 1700 || copter.channel_yaw->get_radio_min() > 1300 || copter.channel_yaw->get_radio_max() < 1700) {
        passed_checks &= ~ARMING_CHECK_COPTER_RC_6;
        group_contains_failure = true;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (copter.channel_roll->get_radio_trim() < 1300 || copter.channel_roll->get_radio_trim() > 1700 || copter.channel_pitch->get_radio_trim() < 1300 || copter.channel_pitch->get_radio_trim() > 1700) {
        passed_checks &= ~ARMING_CHECK_COPTER_RC_6;
        group_contains_failure = true;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (copter.channel_yaw->get_radio_trim() < 1300 || copter.channel_yaw->get_radio_trim() > 1700) {
        passed_checks &= ~ARMING_CHECK_COPTER_RC_6;
        group_contains_failure = true;
    }

    // display failure
    if ((passed_checks&ARMING_CHECK_COPTER_RC_6)==0 && report) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: RC not calibrated");
    }
    
    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::battery_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::battery_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false;

    // check battery voltage
    enabled_checks |= ARMING_CHECK_COPTER_BATTERY;
    passed_checks |= ARMING_CHECK_COPTER_BATTERY;
    if (copter.failsafe.battery || (!copter.ap.usb_connected && copter.battery.exhausted(copter.g.fs_batt_voltage, copter.g.fs_batt_mah))) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Arm: Check Battery");
        }
        passed_checks &= ARMING_CHECK_COPTER_BATTERY;
        group_contains_failure = true;
    }

    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::rangefinder_optflow_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{  
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RANGEFINDER_OPTFLOW)) {
        enabled_checks &= ~ARMING_CHECK_COPTER_RANGEFINDER_OPTFLOW;
        return ARMING_CHECK_DISABLED;
    }

#if RANGEFINDER_ENABLED == ENABLED && OPTFLOW == ENABLED
    // check range finder if optflow enabled
    enabled_checks |= ARMING_CHECK_COPTER_RANGEFINDER_OPTFLOW;
    if (copter.optflow.enabled() && !copter.rangefinder.pre_arm_check()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: check range finder");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_RANGEFINDER_OPTFLOW;
        return ARMING_CHECK_FAILED;
    }
    passed_checks |= ARMING_CHECK_COPTER_RANGEFINDER_OPTFLOW;
#endif

    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::terrain_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::terrain_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false; // some checks are always run
  
    // check if terrain following is enabled, using a range finder but RTL_ALT is higher than rangefinder's max range
    // To-Do: modify RTL return path to fly at or above the RTL_ALT and remove this check
    enabled_checks |= ARMING_CHECK_COPTER_TERRAIN;
    passed_checks |= ARMING_CHECK_COPTER_TERRAIN;
    if ((copter.rangefinder.num_sensors() > 0) && (copter.g.rtl_altitude > copter.rangefinder.max_distance_cm())) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: RTL_ALT above rangefinder max range");
        }
        passed_checks &= ~ARMING_CHECK_COPTER_TERRAIN;
        group_contains_failure = true;
    }
  
    if (copter.terrain_use()) {
        enabled_checks |= ARMING_CHECK_COPTER_TERRAIN_2;
        passed_checks |= ARMING_CHECK_COPTER_TERRAIN_2;
        uint16_t terr_pending, terr_loaded;
        copter.terrain.get_statistics(terr_pending, terr_loaded);
        if (terr_pending > 0) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Terrain data");
            }
            passed_checks &= ~ARMING_CHECK_COPTER_TERRAIN;
            group_contains_failure = true;
        }
    }
    
    if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }
#endif

    // if we got here checks have passed
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming_Copter::rallypoint_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
#if AC_RALLY == ENABLED && AC_FENCE == ENABLED
    // call parent class checks
    ArmingCheckResult ret = AP_Arming::rallypoint_checks(report, enabled_checks, passed_checks);
    bool group_contains_failure = false;
    bool group_is_disabled = false; // some checks are always run

    // check rally points are within fences
    enabled_checks |= ARMING_CHECK_COPTER_RALLYPOINT;
    passed_checks |= ARMING_CHECK_COPTER_RALLYPOINT;
    for (uint8_t i=0; i<copter.rally.get_rally_total(); i++) {
         RallyLocation rally_loc;
         if (copter.rally.get_rally_point_with_index(i, rally_loc)) {
             Location_Class rally_point(copter.rally.rally_location_to_location(rally_loc));
             if (!copter.fence.check_destination_within_fence(rally_point)) {
                 if (report) {
                     GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: rallypoints outside fence");
                 }
                 passed_checks &= ~ARMING_CHECK_COPTER_RALLYPOINT;
                 group_contains_failure = true;
                 break;
             }
         }
     }
     
     if (ret==ARMING_CHECK_DISABLED && group_is_disabled) {
         return ARMING_CHECK_DISABLED;
     }

     if (ret==ARMING_CHECK_FAILED || group_contains_failure) {
         return ARMING_CHECK_FAILED;
     }
 #endif
    
    return ARMING_CHECK_PASSED;
}