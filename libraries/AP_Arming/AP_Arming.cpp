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
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>

#define AP_ARMING_COMPASS_MAGFIELD_EXPECTED 530
#define AP_ARMING_COMPASS_MAGFIELD_MIN  185     // 0.35 * 530 milligauss
#define AP_ARMING_COMPASS_MAGFIELD_MAX  875     // 1.65 * 530 milligauss
#define AP_ARMING_BOARD_VOLTAGE_MIN     4.3f
#define AP_ARMING_BOARD_VOLTAGE_MAX     5.8f
#define AP_ARMING_ACCEL_ERROR_THRESHOLD 0.75f

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Arming::var_info[] = {

    // @Param: REQUIRE
    // @DisplayName: Require Arming Motors 
    // @Description: Arming disabled until some requirements are met. If 0, there are no requirements (arm immediately).  If 1, require rudder stick or GCS arming before arming motors and sends the minimum throttle PWM value to the throttle channel when disarmed.  If 2, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed. See the ARMING_CHECK_* parameters to see what checks are done before arming. Note, if setting this parameter to 0 a reboot is required to arm the plane.  Also note, even with this parameter at 0, if ARMING_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre-arm check failure. This parameter is relevant for ArduPlane only.
    // @Values: 0:Disabled,1:THR_MIN PWM when disarmed,2:0 PWM when disarmed
    // @User: Advanced
    AP_GROUPINFO_FLAGS_FRAME("REQUIRE",     0,      AP_Arming,  require,                 1,
                             AP_PARAM_NO_SHIFT,
                             AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_ROVER),

    // @Param: CHECK
    // @DisplayName: Arm Checks to Peform (bitmask)
    // @Description: Checks prior to arming motor. This is a bitmask of checks that will be performed before allowing arming. The default is no checks, allowing arming at any time. You can select whatever checks you prefer by adding together the values of each check type to set this parameter. For example, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING_CHECK to 72. For most users it is recommended that you set this to 1 to enable all checks.
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

#if !APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // @Param: MIN_VOLT
    // @DisplayName: Minimum arming voltage on the first battery
    // @Description: The minimum voltage on the first battery to arm, 0 disables the check.  This parameter is relevant for ArduPlane only.
    // @Units: V
    // @Increment: 0.1 
    // @User: Standard
    AP_GROUPINFO("MIN_VOLT",      4,     AP_Arming,  _min_voltage[0],  0),

    // @Param: MIN_VOLT2
    // @DisplayName: Minimum arming voltage on the second battery
    // @Description: The minimum voltage on the first battery to arm, 0 disables the check. This parameter is relevant for ArduPlane only.
    // @Units: V
    // @Increment: 0.1 
    // @User: Standard
    AP_GROUPINFO("MIN_VOLT2",     5,     AP_Arming,  _min_voltage[1],  0),
#endif

    AP_GROUPEND
};

//The function point is particularly hacky, hacky, tacky
//but I don't want to reimplement messaging to GCS at the moment:
AP_Arming::AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                     const AP_BattMonitor &battery) :
    ahrs(ahrs_ref),
    barometer(baro),
    _compass(compass),
    _battery(battery),
    armed(false),
    arming_method(NONE)
{
    AP_Param::setup_object_defaults(this, var_info);

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // default REQUIRE parameter to 1 (needed for Copter which is missing the parameter declaration above)
    require.set_default(YES_MIN_PWM);
#endif

    memset(last_accel_pass_ms, 0, sizeof(last_accel_pass_ms));
    memset(last_gyro_pass_ms, 0, sizeof(last_gyro_pass_ms));
}

uint16_t AP_Arming::compass_magfield_expected() const
{
    return AP_ARMING_COMPASS_MAGFIELD_EXPECTED;
}

bool AP_Arming::is_armed()
{
    return require == NONE || armed;
}

uint16_t AP_Arming::get_enabled_checks()
{
    return checks_to_perform;
}

bool AP_Arming::check_enabled(const enum AP_Arming::ArmingChecks check) const
{
    return ((checks_to_perform & ARMING_CHECK_ALL) ||
            (checks_to_perform & check));
}

bool AP_Arming::check(bool success, bool report, AP_Arming::ArmingChecks check, const char *failmsg, ...) const
{
    if (!check_enabled(check)) {
        return true;
    }
    if (success) {
        return true;
    }
    if (report) {
        va_list args;
        va_start(args, failmsg);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Arming: %s", failmsg, args);
        va_end(args);
    }
    return false;
}

bool AP_Arming::barometer_checks(bool report)
{
    return check(barometer.all_healthy(),
                 report,
                 ARMING_CHECK_BARO,
                 "Barometer not healthy");
}

bool AP_Arming::airspeed_checks(bool report)
{
    const AP_Airspeed *airspeed = ahrs.get_airspeed();
    if (airspeed == nullptr) {
        // not an airspeed capable vehicle
        return true;
    }
    return check((airspeed->enabled() && airspeed->use() && !airspeed->healthy()),
                 report,
                 ARMING_CHECK_AIRSPEED,
                 "Airspeed not healthy");
}

bool AP_Arming::logging_checks(bool report)
{
    return (
        check(!DataFlash_Class::instance()->logging_failed(),
              report,
              ARMING_CHECK_LOGGING,
              "Logging failed") &
        check(DataFlash_Class::instance()->CardInserted(),
              report,
              ARMING_CHECK_LOGGING,
              "No SD Card")
        );
}

bool AP_Arming::ins_accels_consistent(const AP_InertialSensor &ins)
{
    if (ins.get_accel_count() <= 1) {
        return true;
    }

    const Vector3f &prime_accel_vec = ins.get_accel();
    for(uint8_t i=0; i<ins.get_accel_count(); i++) {
        if (!ins.use_accel(i)) {
            continue;
        }
        // get next accel vector
        const Vector3f &accel_vec = ins.get_accel(i);
        Vector3f vec_diff = accel_vec - prime_accel_vec;
        // allow for user-defined difference, typically 0.75 m/s/s. Has to pass in last 10 seconds
        float threshold = accel_error_threshold;
        if (i >= 2) {
            /*
              we allow for a higher threshold for IMU3 as it
              runs at a different temperature to IMU1/IMU2,
              and is not used for accel data in the EKF
            */
            threshold *= 3;
        }

        // EKF is less sensitive to Z-axis error
        vec_diff.z *= 0.5f;

        if (vec_diff.length() <= threshold) {
            last_accel_pass_ms[i] = AP_HAL::millis();
        }
        if (AP_HAL::millis() - last_accel_pass_ms[i] > 10000) {
            return false;
        }
    }

    return true;
}

bool AP_Arming::ins_gyros_consistent(const AP_InertialSensor &ins)
{
    if (ins.get_gyro_count() <= 1) {
        return true;
    }

    const Vector3f &prime_gyro_vec = ins.get_gyro();
    for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
        if (!ins.use_gyro(i)) {
            continue;
        }
        // get next gyro vector
        const Vector3f &gyro_vec = ins.get_gyro(i);
        Vector3f vec_diff = gyro_vec - prime_gyro_vec;
        // allow for up to 5 degrees/s difference. Pass if it has
        // been OK in last 10 seconds
        if (vec_diff.length() <= radians(5)) {
            last_gyro_pass_ms[i] = AP_HAL::millis();
        }
        if (AP_HAL::millis() - last_gyro_pass_ms[i] > 10000) {
            return false;
        }
    }

    return true;
}


bool AP_Arming::ins_checks(bool report)
{
    const AP_InertialSensor &ins = ahrs.get_ins();
    bool ret = true;
    ret &= (check(ins.gyro_calibrated_ok_all(),
                  report,
                  ARMING_CHECK_INS,
                  "Gyros not calibrated") &&
            check(ins.get_gyro_health_all(),
                  report,
                  ARMING_CHECK_INS,
                  "Gyros not healthy") &&
            check(ins_gyros_consistent(ins),
                  report,
                  ARMING_CHECK_INS,
                  "Gyros inconsistent")
        );
    ret &= (check(!ins.accel_cal_requires_reboot(),
                  report,
                  ARMING_CHECK_INS,
                  "Accels calibrated requires reboot") &&
            check(ins.accel_calibrated_ok_all(),
                  report,
                  ARMING_CHECK_INS,
                  "3D Accel calibration needed") &&
            check(ins.get_accel_health_all(),
                  report,
                  ARMING_CHECK_INS,
                  "Accels not healthy") &&
            check(ins_accels_consistent(ins),
                  report,
                  ARMING_CHECK_INS,
                  "Accels inconsistent")
        );

    return ret;
}

bool AP_Arming::compass_checks(bool report)
{
    if (!_compass.use_for_yaw()) {
        // compass use is disabled
        return true;
    }

    bool ret = true;

    ret &= check(_compass.healthy(),
                 report,
                 ARMING_CHECK_COMPASS,
                 "Compass not healthy");

    ret &= check(!_compass.is_calibrating(),
                 report,
                 ARMING_CHECK_COMPASS,
                 "Compass calibration running") &&
        check(!_compass.compass_cal_requires_reboot(),
              report,
              ARMING_CHECK_COMPASS,
              "Compass calibration requires reboot") &&
        // check compass learning is on or offsets have been set
        check(_compass.learn_offsets_enabled() || _compass.configured(),
              report,
              ARMING_CHECK_COMPASS,
              "Compass not calibrated") &&

        (check(_compass.get_offsets().length() <= _compass.get_offsets_max(),
               report,
               ARMING_CHECK_COMPASS,
               "Compass offsets too high") &
         check(_compass.get_field().length() <= AP_ARMING_COMPASS_MAGFIELD_MAX,
               report,
               ARMING_CHECK_COMPASS,
               "Magnetic field too weak") &
         check(_compass.get_field().length() >= AP_ARMING_COMPASS_MAGFIELD_MIN,
               report,
               ARMING_CHECK_COMPASS,
               "Magnetic field too strong") ) &&

        check(_compass.consistent(),
              report,
              ARMING_CHECK_COMPASS,
              "Compasses inconsistent");

    return ret;
}

bool AP_Arming::gps_checks(bool report)
{
    const AP_GPS &gps = ahrs.get_gps();
    bool ret = true;
    ret &= check( (home_status() != HOME_UNSET && gps.status() >= AP_GPS::GPS_OK_FIX_3D),
                 report,
                 ARMING_CHECK_COMPASS,
                 "Bad GPS Position");
    uint8_t first_unconfigured = gps.first_unconfigured_gps();
    bool pass = (first_unconfigured == AP_GPS::GPS_ALL_CONFIGURED);
    ret &= check(pass,
                 report,
                 ARMING_CHECK_COMPASS,
                 "GPS %d failing configuration checks",
                 first_unconfigured +1);
    if (!pass) {
        gps.broadcast_first_configuration_failure_reason();
    }

    float distance_m;
    ret &= check(gps.all_consistent(distance_m),
                 report,
                 ARMING_CHECK_COMPASS,
                 "GPS positions differ by %4.1fm",
                 (double)distance_m);
    ret &= check(gps.blend_health_check(),
                 report,
                 ARMING_CHECK_COMPASS,
                 "GPS blending unhealthy");

    return ret;
}

bool AP_Arming::battery_checks(bool report)
{
    bool ret = true;
    ret &= check(!AP_Notify::flags.failsafe_battery,
                 report,
                 ARMING_CHECK_BATTERY,
                 "Battery failsafe on");

    for (int i = 0; i < _battery.num_instances(); i++) {
        ret &= check( (_min_voltage[i] <= 0.0f) || (_battery.voltage(i) > _min_voltage[i]),
                      report,
                      ARMING_CHECK_BATTERY,
                      "Battery %d voltage %.1f below minimum %.1f",
                      i+1,
                      (double)_battery.voltage(i),
                      (double)_min_voltage[i]);
    }

    return true;
}

bool AP_Arming::hardware_safety_check(bool report) 
{
    return check(hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED,
                 report,
                 ARMING_CHECK_SWITCH,
                 "Hardware safety switch");
}

bool AP_Arming::manual_transmitter_checks(bool report)
{
    return check(!AP_Notify::flags.failsafe_radio,
                 report,
                 ARMING_CHECK_RC,
                 "Radio failsafe on");
    //TODO verify radio calibration
    //Requires access to Parameters ... which are implemented a little
    //differently for Rover, Plane, and Copter.
}

bool AP_Arming::board_voltage_checks(bool report)
{
#if HAL_HAVE_BOARD_VOLTAGE
    return check( ((hal.analogin->board_voltage() >= AP_ARMING_BOARD_VOLTAGE_MIN) && (hal.analogin->board_voltage() < AP_ARMING_BOARD_VOLTAGE_MAX)),
                  report,
                  ARMING_CHECK_VOLTAGE,
                  "Check board voltage");
#endif
    return true;
}

bool AP_Arming::pre_arm_checks(bool report)
{
#if !APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    if (armed || require == NONE) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }
#endif

    return hardware_safety_check(report)
        &  barometer_checks(report)
        &  ins_checks(report)
        &  compass_checks(report)
        &  gps_checks(report)
        &  battery_checks(report)
        &  logging_checks(report)
        &  manual_transmitter_checks(report)
        &  board_voltage_checks(report);
}

bool AP_Arming::arm_checks(uint8_t method)
{
    // note that this will prepare DataFlash to start logging
    // so should be the last check to be done before arming
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_LOGGING)) {
        DataFlash_Class *df = DataFlash_Class::instance();
        df->PrepForArming();
        if (!df->logging_started()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Arm: Logging not started");
            return false;
        }
    }
    return true;
}

//returns true if arming occurred successfully
bool AP_Arming::arm(uint8_t method)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // Copter should never use this function
    return false;
#else
    if (armed) { //already armed
        return false;
    }

    //are arming checks disabled?
    if (checks_to_perform == ARMING_CHECK_NONE) {
        armed = true;
        arming_method = NONE;
        gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");
        return true;
    }

    if (pre_arm_checks(true) && arm_checks(method)) {
        armed = true;
        arming_method = method;

        gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");

        //TODO: Log motor arming to the dataflash
        //Can't do this from this class until there is a unified logging library

    } else {
        armed = false;
        arming_method = NONE;
    }

    return armed;
#endif
}

//returns true if disarming occurred successfully
bool AP_Arming::disarm() 
{
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // Copter should never use this function
    return false;
#else
    if (!armed) { // already disarmed
        return false;
    }
    armed = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle disarmed");

    //TODO: Log motor disarming to the dataflash
    //Can't do this from this class until there is a unified logging library.

    return true;
#endif
}

AP_Arming::ArmingRequired AP_Arming::arming_required() 
{
    return (AP_Arming::ArmingRequired)require.get();
}
