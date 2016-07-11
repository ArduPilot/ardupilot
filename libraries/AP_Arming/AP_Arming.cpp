/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Arming.h"
#include "Arming_Failure.h"
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>

#define AP_ARMING_COMPASS_OFFSETS_MAX   600
#define AP_ARMING_COMPASS_MAGFIELD_MIN  185     // 0.35 * 530 milligauss
#define AP_ARMING_COMPASS_MAGFIELD_MAX  875     // 1.65 * 530 milligauss
#define AP_ARMING_BOARD_VOLTAGE_MIN     4.3f
#define AP_ARMING_BOARD_VOLTAGE_MAX     5.8f
#define AP_ARMING_ACCEL_ERROR_THRESHOLD 0.75f

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Arming::var_info[] = {
    // @Param: REQUIRE
    // @DisplayName: Require Arming Motors 
    // @Description: Arming disabled until some requirements are met. If 0, there are no requirements (arm immediately).  If 1, require rudder stick or GCS arming before arming motors and send THR_MIN PWM to throttle channel when disarmed.  If 2, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed. See the ARMING_CHECK_* parameters to see what checks are done before arming. Note, if setting this parameter to 0 a reboot is required to arm the plane.  Also note, even with this parameter at 0, if ARMING_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre-arm check failure.
    // @Values: 0:Disabled,1:THR_MIN PWM when disarmed,2:0 PWM when disarmed
    // @User: Advanced
    AP_GROUPINFO_FLAGS("REQUIRE",     0,      AP_Arming,  require,                 1, AP_PARAM_NO_SHIFT),

    // @Param: CHECK
    // @DisplayName: Arm Checks to Peform (bitmask)
    // @Description: Checks prior to arming motor. This is a bitmask of checks that will be performed befor allowing arming. The default is no checks, allowing arming at any time. You can select whatever checks you prefer by adding together the values of each check type to set this parameter. For example, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING_CHECK to 72. For most users it is recommended that you set this to 1 to enable all checks.
    // @Values: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Failsafe,128:Board voltage,256:Battery Level,512:Airspeed,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration
    // @Bitmask: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC,7:Board voltage,8:Battery Level,9:Airspeed,10:Logging Available,11:Hardware safety switch,12:GPS Configuration
    // @User: Standard
    AP_GROUPINFO("CHECK",        2,     AP_Arming,  checks_to_perform,       ARMING_CHECK_ALL),

    // @Param: ACCTHRESH
    // @DisplayName: Accelerometer error threshold
    // @Description: Accelerometer error threshold used to determine inconsistent accelerometers. Compares this error range to other accelerometers to detect a hardware or calibration error. Lower value means tighter check and harder to pass arming check. Not all accelerometers are created equal.
    // @Units: m/s/s
    // @Range: 0.25 3.0
    // @User: Advanced
    AP_GROUPINFO("ACCTHRESH",    3,     AP_Arming,  accel_error_threshold,  AP_ARMING_ACCEL_ERROR_THRESHOLD),

    // @Param: MIN_VOLT
    // @DisplayName: Minimum arming voltage on the first battery
    // @Description: The minimum voltage on the first battery to arm, 0 disabes the check
    // @Units: Volts
    // @Increment: 0.1 
    // @User: Standard
    AP_GROUPINFO("MIN_VOLT",      4,     AP_Arming,  _min_voltage[0],  0),

    // @Param: MIN_VOLT2
    // @DisplayName: Minimum arming voltage on the second battery
    // @Description: The minimum voltage on the first battery to arm, 0 disabes the check
    // @Units: Volts
    // @Increment: 0.1 
    // @User: Standard
    AP_GROUPINFO("MIN_VOLT2",     5,     AP_Arming,  _min_voltage[1],  0),

    AP_GROUPEND
};

//The function point is particularly hacky, hacky, tacky
//but I don't want to reimplement messaging to GCS at the moment:
AP_Arming::AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                     const AP_BattMonitor &battery, const enum HomeState &home_set) :
    ahrs(ahrs_ref),
    barometer(baro),
    _compass(compass),
    _battery(battery),
    home_is_set(home_set),
    armed(false),
    logging_available(false),
    arming_method(NONE)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(last_accel_pass_ms, 0, sizeof(last_accel_pass_ms));
    memset(last_gyro_pass_ms, 0, sizeof(last_gyro_pass_ms));
}

bool AP_Arming::is_armed()
{ 
    return require == NONE || armed; 
}

uint16_t AP_Arming::get_enabled_checks()
{
    return checks_to_perform;
}

void AP_Arming::set_enabled_checks(uint16_t ap)
{
    checks_to_perform = ap;
}

AP_Arming::ArmingCheckResult AP_Arming::barometer_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_BARO)) {
        enabled_checks &= ~ARMING_CHECK_BARO;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_BARO;

    if (!barometer.all_healthy()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Barometer not healthy");
        }
        passed_checks &= ~ARMING_CHECK_BARO;
        return ARMING_CHECK_FAILED;
    }

    passed_checks |= ARMING_CHECK_BARO;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::airspeed_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_AIRSPEED)) {
        enabled_checks &= ~ARMING_CHECK_AIRSPEED;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_BARO;

    const AP_Airspeed *airspeed = ahrs.get_airspeed();
    if (airspeed == NULL) {
        // not an airspeed capable vehicle
        passed_checks |= ARMING_CHECK_BARO;
        return ARMING_CHECK_PASSED;
    }
    if (airspeed->enabled() && airspeed->use() && !airspeed->healthy()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: airspeed not healthy");
        }
        passed_checks &= ~ARMING_CHECK_BARO;
        return ARMING_CHECK_FAILED;;
    }

    passed_checks |= ARMING_CHECK_BARO;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::logging_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_LOGGING)) {
        enabled_checks &= ~ARMING_CHECK_LOGGING;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_LOGGING;
    
    if (!logging_available) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: logging not available");
        }
        passed_checks &= ~ARMING_CHECK_LOGGING;
        return ARMING_CHECK_FAILED;
    }
    
    passed_checks |= ARMING_CHECK_LOGGING;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::ins_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    const AP_InertialSensor &ins = ahrs.get_ins();
    
    bool group_contains_failure = false;
    bool group_is_disabled = true;
    
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {
        enabled_checks |= ARMING_CHECK_INS;
        passed_checks |= ARMING_CHECK_INS;
        group_is_disabled = false;
        if (!ins.get_gyro_health_all()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: gyros not healthy");
            }
            passed_checks &= ~ARMING_CHECK_INS;
            group_contains_failure = true;
        }
    }
    
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS_2)) {
        enabled_checks |= ARMING_CHECK_INS_2;
        passed_checks |= ARMING_CHECK_INS_2;
        group_is_disabled = false;
        if (!ins.gyro_calibrated_ok_all()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: gyros not calibrated");
            }
            passed_checks &= ~ARMING_CHECK_INS_2;
            group_contains_failure = true;
        }
    }
    
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS_3)) {
        enabled_checks |= ARMING_CHECK_INS_3;
        passed_checks |= ARMING_CHECK_INS_3;
        group_is_disabled = false;
        if (!ins.get_accel_health_all()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: accels not healthy");
            }
            passed_checks &= ~ARMING_CHECK_INS_3;
            group_contains_failure = true;
        }
    }
    
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS_4)) {
        enabled_checks |= ARMING_CHECK_INS_4;
        passed_checks |= ARMING_CHECK_INS_4;
        group_is_disabled = false;
        if (!ins.accel_calibrated_ok_all()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: 3D accel cal needed");
            }
            passed_checks &= ~ARMING_CHECK_INS_4;
            group_contains_failure = true;
        }
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS_5)) {
        enabled_checks |= ARMING_CHECK_INS_5;
        passed_checks |= ARMING_CHECK_INS_5;
        group_is_disabled = false;
        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                // allow for up to 0.75 m/s/s difference. Has to pass
                // in last 10 seconds
                float threshold = 0.75f;
                if (i >= 2) {
                    /*
                      we allow for a higher threshold for IMU3 as it
                      runs at a different temperature to IMU1/IMU2,
                      and is not used for accel data in the EKF
                     */
                    threshold *= 3;
                }
                if (vec_diff.length() <= threshold) {
                    last_accel_pass_ms[i] = AP_HAL::millis();
                }
                if (AP_HAL::millis() - last_accel_pass_ms[i] > 10000) {
                    if (report) {
                        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: inconsistent Accelerometers");
                    }
                    passed_checks &= ~ARMING_CHECK_INS_5;
                    group_contains_failure = true;
                }
            }
        }
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS_6)) {
        enabled_checks |= ARMING_CHECK_INS_6;
        passed_checks |= ARMING_CHECK_INS_6;
        group_is_disabled = false;
        // check all gyros are giving consistent readings
        if (ins.get_gyro_count() > 1) {
            const Vector3f &prime_gyro_vec = ins.get_gyro();
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get next gyro vector
                const Vector3f &gyro_vec = ins.get_gyro(i);
                Vector3f vec_diff = gyro_vec - prime_gyro_vec;
                // allow for up to 5 degrees/s difference. Pass if its
                // been OK in last 10 seconds
                if (vec_diff.length() <= radians(5)) {
                    last_gyro_pass_ms[i] = AP_HAL::millis();
                }
                if (AP_HAL::millis() - last_gyro_pass_ms[i] > 10000) {
                    if (report) {
                        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: inconsistent gyros");
                    }
                    passed_checks &= ~ARMING_CHECK_INS_6;
                    group_contains_failure = true;
                }
            }
        }
    } 
    
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS_7)) {
        enabled_checks |= ARMING_CHECK_INS_7;
        passed_checks |= ARMING_CHECK_INS_7;
        group_is_disabled = false;
        if (ins.accel_cal_requires_reboot()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Accelerometers calibrated requires reboot");
            }
            passed_checks &= ARMING_CHECK_INS_7;
            group_contains_failure = true;
        }
    }

    // if some enabled checks failed, fail the group
    if (group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }
    
    // if all the checks were disabled, notify it is disabled
    if (group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }
  
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::parameter_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        enabled_checks &= !ARMING_CHECK_PARAMETERS;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_PARAMETERS;

    passed_checks |= ARMING_CHECK_PARAMETERS;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::compass_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
  
    bool group_contains_failure = false;
    bool group_is_disabled = true;

    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS)) {
        enabled_checks |= ARMING_CHECK_COMPASS;
        passed_checks |= ARMING_CHECK_COMPASS;
        group_is_disabled = false;
        if (!_compass.healthy()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Compass not healthy");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS;
            group_contains_failure = true;
        }  
    }
  
    // check compass learning is on or offsets have been set
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS_2)) {
        enabled_checks |= ARMING_CHECK_COMPASS_2;
        passed_checks |= ARMING_CHECK_COMPASS_2;
        group_is_disabled = false;
        if (!_compass.learn_offsets_enabled() && !_compass.configured()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Compass not calibrated");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS_2;
            group_contains_failure = true;
        }
    }

    //check if compass is calibrating
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS_3)) {
        enabled_checks |= ARMING_CHECK_COMPASS_3;
        passed_checks |= ARMING_CHECK_COMPASS_3;
        group_is_disabled = false;
        if (_compass.is_calibrating()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Arm: Compass calibration running");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS_3;
            group_contains_failure = true;
        }
    }

    //check if compass has calibrated and requires reboot
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS_4)) {
        enabled_checks |= ARMING_CHECK_COMPASS_4;
        passed_checks |= ARMING_CHECK_COMPASS_4;
        group_is_disabled = false;
        if (_compass.compass_cal_requires_reboot()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibrated requires reboot");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS_4;
            group_contains_failure = true;
        }
    }

    // check for unreasonable compass offsets
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS_5)) {
        enabled_checks |= ARMING_CHECK_COMPASS_5;
        passed_checks |= ARMING_CHECK_COMPASS_5;
        group_is_disabled = false;
        Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > AP_ARMING_COMPASS_OFFSETS_MAX) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Compass offsets too high");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS_5;
            group_contains_failure = true;
        }
    }

    // check for unreasonable mag field length
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS_6)) {
        enabled_checks |= ARMING_CHECK_COMPASS_6;
        passed_checks |= ARMING_CHECK_COMPASS_6;
        group_is_disabled = false;
        float mag_field = _compass.get_field().length();
        if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Check mag field");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS_6;
            group_contains_failure = true;
        }
    }

    // check all compasses point in roughly same direction
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_COMPASS_7)) {
        enabled_checks |= ARMING_CHECK_COMPASS_7;
        passed_checks |= ARMING_CHECK_COMPASS_7;
        group_is_disabled = false;
        if (!_compass.consistent()) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: inconsistent compasses");
            }
            passed_checks &= ~ARMING_CHECK_COMPASS_7;
            group_contains_failure = true;
        }
    }

    if (!_compass.use_for_yaw()) {
        // compass use is disabled
        return ARMING_CHECK_PASSED;
    }

    if (group_contains_failure) {
        return ARMING_CHECK_FAILED;
    }
    
    if (group_is_disabled) {
        return ARMING_CHECK_DISABLED;
    }

    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::gps_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{  
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_GPS)) {
        enabled_checks &= ~ARMING_CHECK_GPS;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_GPS;

    const AP_GPS &gps = ahrs.get_gps();

    //GPS OK?
    if (home_is_set == HOME_UNSET ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Bad GPS Position");
        }
        passed_checks &= ~ARMING_CHECK_GPS;
        return ARMING_CHECK_FAILED;
    }

    passed_checks |= ARMING_CHECK_GPS;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::rangefinder_optflow_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RANGEFINDER_OPTFLOW)) {
        enabled_checks &= ~ARMING_CHECK_RANGEFINDER_OPTFLOW;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_RANGEFINDER_OPTFLOW;
    
    passed_checks |= ARMING_CHECK_RANGEFINDER_OPTFLOW;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::battery_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_BATTERY)) {
        enabled_checks &= ~ARMING_CHECK_BATTERY;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_BATTERY;

    if (AP_Notify::flags.failsafe_battery) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Battery failsafe on");
        }
        passed_checks &= ~ARMING_CHECK_BATTERY;
        return ARMING_CHECK_FAILED;
    }

    passed_checks |= ARMING_CHECK_BATTERY;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::hardware_safety_check(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
  
    // report that the safety switch check is enabled
    enabled_checks |= ARMING_CHECK_SWITCH;
    
    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Hardware Safety Switch");
        }
        passed_checks &= ~ARMING_CHECK_SWITCH;
        return ARMING_CHECK_FAILED;
    }

    passed_checks |= ARMING_CHECK_SWITCH;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::manual_transmitter_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RC)) {
        enabled_checks &= ~ARMING_CHECK_RC;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_RC;

    if (AP_Notify::flags.failsafe_radio) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Radio failsafe on");
        }
        passed_checks &= ~ARMING_CHECK_RC;
        return ARMING_CHECK_FAILED;
    }

    //TODO verify radio calibration
    //Requires access to Parameters ... which are implemented a little
    //differently for Rover, Plane, and Copter.

    passed_checks |= ARMING_CHECK_RC;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::board_voltage_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
    // check board voltage
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        enabled_checks &= ~ARMING_CHECK_VOLTAGE;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_VOLTAGE;
    
    if(!is_zero(hal.analogin->board_voltage()) && 
    ((hal.analogin->board_voltage() < AP_ARMING_BOARD_VOLTAGE_MIN) || (hal.analogin->board_voltage() > AP_ARMING_BOARD_VOLTAGE_MAX))) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PreArm: Check board voltage");
        }
        passed_checks &= ~ARMING_CHECK_VOLTAGE;
        return ARMING_CHECK_FAILED;
    }

    passed_checks |= ARMING_CHECK_VOLTAGE;
    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::terrain_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_TERRAIN)) {
        enabled_checks &= ~ARMING_CHECK_TERRAIN;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_TERRAIN;
    
    passed_checks |= ARMING_CHECK_TERRAIN;
#endif

    return ARMING_CHECK_PASSED;
}

AP_Arming::ArmingCheckResult AP_Arming::rallypoint_checks(bool report, uint64_t &enabled_checks, uint64_t &passed_checks)
{
#if AC_RALLY == ENABLED && AC_FENCE == ENABLED
    if (!(checks_to_perform & ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RALLYPOINT)) {
        enabled_checks &= ~ARMING_CHECK_RALLYPOINT;
        return ARMING_CHECK_DISABLED;
    }
    enabled_checks |= ARMING_CHECK_RALLYPOINT;
    
    passed_checks |= ARMING_CHECK_RALLYPOINT;
#endif
  
    return ARMING_CHECK_PASSED;
}

bool AP_Arming::pre_arm_checks(bool report)
{
    if (armed || require == NONE) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }

    uint64_t enabled_checks = 0;
    uint64_t passed_checks = 0;

    hardware_safety_check(report, enabled_checks, passed_checks);
    barometer_checks(report, enabled_checks, passed_checks);
    ins_checks(report, enabled_checks, passed_checks);
    parameter_checks(report, enabled_checks, passed_checks);
    compass_checks(report, enabled_checks, passed_checks);
    gps_checks(report, enabled_checks, passed_checks);
    battery_checks(report, enabled_checks, passed_checks);
    logging_checks(report, enabled_checks, passed_checks);
    manual_transmitter_checks(report, enabled_checks, passed_checks);
    board_voltage_checks(report, enabled_checks, passed_checks);
    rangefinder_optflow_checks(report, enabled_checks, passed_checks);
    terrain_checks(report, enabled_checks, passed_checks);

    GCS_MAVLINK::send_prearm_check_report_all(enabled_checks, passed_checks);

    // total results
    bool success = (enabled_checks == (enabled_checks & passed_checks));
    return success;
}

//returns true if arming occurred successfully
bool AP_Arming::arm(uint8_t method)
{
    if (armed) { //already armed
        return false;
    }

    //are arming checks disabled?
    if (checks_to_perform == ARMING_CHECK_NONE) {
        armed = true;
        arming_method = NONE;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Throttle armed");
        return true;
    }

    if (pre_arm_checks(true)) {
        armed = true;
        arming_method = method;

        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Throttle armed");

        //TODO: Log motor arming to the dataflash
        //Can't do this from this class until there is a unified logging library

    } else {
        armed = false;
        arming_method = NONE;
    }

    return armed;
}

//returns true if disarming occurred successfully
bool AP_Arming::disarm() 
{
    if (!armed) { // already disarmed
        return false;
    }
    armed = false;

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Throttle disarmed");

    //TODO: Log motor disarming to the dataflash
    //Can't do this from this class until there is a unified logging library.

    return true;
}

AP_Arming::ArmingRequired AP_Arming::arming_required() 
{
    return (AP_Arming::ArmingRequired)require.get();
}
