// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for copter
 */
#include "arming_checks.h"
#include "Copter.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Arming_Copter::var_info[] PROGMEM = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    AP_GROUPEND
};

/*
  additional arming checks for copter
 */
bool AP_Arming_Copter::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    return ret;
}

bool AP_Arming_Copter::barometer_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::barometer_checks(report)) {
        return false;
    }

    // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
    // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
    // that may differ from the baro height due to baro drift.
    nav_filter_status filt_status;
    filt_status = copter.inertial_nav.get_filter_status();
    bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
    if (using_baro_ref) {
        if (fabsf(copter.inertial_nav.get_altitude() - barometer.get_altitude()) > COPTER_ARMING_CHECK_ALT_DISPARITY_MAX_CM) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Altitude disparity"));
            }
            return false;
        }
    }

    // if we got here checks have passed
    return true;
}

bool AP_Arming_Copter::ins_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(report)) {
        return false;
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {
        // check ekf attitude, if bad it's usually the gyro biases have not settled
        if (!copter.inertial_nav.get_filter_status().flags.attitude) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: gyros still settling"));
            }
            return false;
        }

        // check lean angle
        if (degrees(acosf(copter.ahrs.cos_roll()*copter.ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Leaning"));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming_Copter::parameter_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::parameter_checks(report)) {
        return false;
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        // radio failsafe parameter checks
        if (copter.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (copter.channel_throttle->radio_min <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                if (report) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return false;
            }
        }

        // lean angle parameter check
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return false;
        }

        // acro balance parameter check
        if ((copter.g.acro_balance_roll > copter.g.p_stabilize_roll.kP()) || (copter.g.acro_balance_pitch > copter.g.p_stabilize_pitch.kP())) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Check ACRO_BAL_ROLL/PITCH"));
            }
            return false;
        }
#if FRAME_CONFIG == HELI_FRAME
        // check helicopter parameters
        if (!copter.motors.parameter_check(report)) {
            return false;
        }
#endif // HELI_FRAME
    }

    return true;
}

bool AP_Arming_Copter::compass_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::compass_checks(report)) {
        return false;
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_COMPASS)) {
        // compass is required for copter
        if (!copter.compass.use_for_yaw()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, PSTR("PreArm: Compass disabled"));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming_Copter::gps_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::gps_checks(report)) {
        return false;
    }

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
        return true;
    }

    // ensure position is ok
    if (!copter.position_ok()) {
        if (report) {
            const char *reason = copter.ahrs.prearm_failure_reason();
            if (reason) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, PSTR("PreArm: %s"), reason);
            } else {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Need 3D Fix"));
            }
        }
        return false;
    }

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    copter.ahrs.get_NavEKF().getVariances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: EKF compass variance"));
        }
        return false;
    }

    // check home and EKF origin are not too far
    if (copter.far_from_EKF_origin(copter.ahrs.get_home())) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: EKF-home variance"));
        }
        return false;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (copter.gps.get_hdop() > copter.g.gps_hdop_good) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: High GPS HDOP"));
        }
        return false;
    }

#if AC_FENCE == ENABLED
    // check fence has not been breached
    if(!copter.fence.pre_arm_check()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: check fence"));
        }
        return false;
    }
#endif

    return true;
}

bool AP_Arming_Copter::manual_transmitter_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::manual_transmitter_checks(report)) {
        return false;
    }

    // vehicle specific checks

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK) && copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Interlock/E-Stop Conflict"));
        }
        return false;
    }

    // if motor interlock aux switch is in use, switch needs to be in disabled position to arm
    if (copter.ap.using_interlock && copter.motors.get_interlock()){
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Motor Interlock Enabled"));
        }
        return false;
    }

    // if we are using Motor Emergency Stop aux switch, check it is not enabled
    if (copter.check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && copter.ap.motor_emergency_stop){
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Motor Emergency Stopped"));
        }
        return false;
    }

    // check throttle is above failsafe throttle
    if ((copter.g.failsafe_throttle != FS_THR_DISABLED) && (copter.channel_throttle->radio_in < copter.g.failsafe_throttle_value)) {
        if (report) {
#if FRAME_CONFIG == HELI_FRAME
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Collective below Failsafe"));
#else
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Throttle below Failsafe"));
#endif
        }
        return false;
    }

    // check throttle is not too high - skips checks if arming from GCS in Guided
    if (copter.control_mode != GUIDED) {
        // above top of deadband is too always high
        bool thr_too_high = copter.channel_throttle->control_in > copter.get_takeoff_trigger_throttle();
        // in manual modes and Drift throttle must be at zero
        if ((copter.mode_has_manual_throttle(copter.control_mode) || copter.control_mode == DRIFT) && (copter.channel_throttle->control_in > 0)) {
            thr_too_high = true;
        }
        if (thr_too_high) {
            if (report) {
#if FRAME_CONFIG == HELI_FRAME
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("Arm: Collective too high"));
#else
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("Arm: Throttle too high"));
#endif
            }
            return false;
        }
    }

    bool ret = true;

    // check if radio has been calibrated
    if (!copter.channel_throttle->radio_min.load() && !copter.channel_throttle->radio_max.load()) {
        ret = false;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (copter.channel_roll->radio_min > 1300 || copter.channel_roll->radio_max < 1700 || copter.channel_pitch->radio_min > 1300 || copter.channel_pitch->radio_max < 1700) {
        ret = false;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (copter.channel_throttle->radio_min > 1300 || copter.channel_throttle->radio_max < 1700 || copter.channel_yaw->radio_min > 1300 || copter.channel_yaw->radio_max < 1700) {
        ret = false;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (copter.channel_roll->radio_trim < 1300 || copter.channel_roll->radio_trim > 1700 || copter.channel_pitch->radio_trim < 1300 || copter.channel_pitch->radio_trim > 1700) {
        ret = false;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (copter.channel_yaw->radio_trim < 1300 || copter.channel_yaw->radio_trim > 1700) {
        ret = false;
    }

    // display failure
    if (!ret && report) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: RC not calibrated"));
    }

    return ret;
}

bool AP_Arming_Copter::battery_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::battery_checks(report)) {
        return false;
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_BATTERY)) {
        // check battery voltage
        if (copter.failsafe.battery || (!copter.ap.usb_connected && copter.battery.exhausted(copter.g.fs_batt_voltage, copter.g.fs_batt_mah))) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("Arm: Check Battery"));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming_Copter::rangefinder_optflow_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RANGEFINDER_OPTFLOW)) {
#if CONFIG_SONAR == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (copter.optflow.enabled() && !copter.sonar.pre_arm_check()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: check range finder"));
            }
            return false;
        }
#endif
    }

    return true;
}
