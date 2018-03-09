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
#define AP_ARMING_AHRS_GPS_ERROR_MAX    10      // accept up to 10m difference between AHRS and GPS

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
    // @Values: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Failsafe,128:Board voltage,256:Battery Level,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration
    // @Values{Plane}: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Failsafe,128:Board voltage,256:Battery Level,512:Airspeed,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration
    // @Bitmask: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC,7:Board voltage,8:Battery Level,10:Logging Available,11:Hardware safety switch,12:GPS Configuration
    // @Bitmask{Plane}: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC,7:Board voltage,8:Battery Level,9:Airspeed,10:Logging Available,11:Hardware safety switch,12:GPS Configuration
    // @User: Standard
    AP_GROUPINFO("CHECK",        2,     AP_Arming,  checks_to_perform,       ARMING_CHECK_ALL),

    // @Param: ACCTHRESH
    // @DisplayName: Accelerometer error threshold
    // @Description: Accelerometer error threshold used to determine inconsistent accelerometers. Compares this error range to other accelerometers to detect a hardware or calibration error. Lower value means tighter check and harder to pass arming check. Not all accelerometers are created equal.
    // @Units: m/s/s
    // @Range: 0.25 3.0
    // @User: Advanced
    AP_GROUPINFO("ACCTHRESH",    3,     AP_Arming,  accel_error_threshold,  AP_ARMING_ACCEL_ERROR_THRESHOLD),

    // @Param: VOLT_MIN
    // @DisplayName: Arming voltage minimum on the first battery
    // @Description: The minimum voltage of the first battery required to arm, 0 disables the check
    // @Units: V
    // @Increment: 0.1 
    // @User: Standard
    AP_GROUPINFO("VOLT_MIN",      4,     AP_Arming,  _min_voltage[0],  0),

    // @Param: VOLT2_MIN
    // @DisplayName: Arming voltage minimum on the second battery
    // @Description: The minimum voltage of the second battery required to arm, 0 disables the check
    // @Units: V
    // @Increment: 0.1 
    // @User: Standard
    AP_GROUPINFO("VOLT2_MIN",     5,     AP_Arming,  _min_voltage[1],  0),

    AP_GROUPEND
};

//The function point is particularly hacky, hacky, tacky
//but I don't want to reimplement messaging to GCS at the moment:
AP_Arming::AP_Arming(const AP_AHRS &ahrs_ref, Compass &compass,
                     const AP_BattMonitor &battery) :
    ahrs(ahrs_ref),
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

bool AP_Arming::barometer_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_BARO)) {
        if (!AP::baro().all_healthy()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Barometer not healthy");
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::airspeed_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_AIRSPEED)) {
        const AP_Airspeed *airspeed = ahrs.get_airspeed();
        if (airspeed == nullptr) {
            // not an airspeed capable vehicle
            return true;
        }
        for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
            if (airspeed->enabled(i) && airspeed->use(i) && !airspeed->healthy(i)) {
                if (report) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Airspeed[%u] not healthy", i);
                }
                return false;
            }
        }
    }

    return true;
}

bool AP_Arming::logging_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_LOGGING)) {
        if (DataFlash_Class::instance()->logging_failed()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Logging failed");
            }
            return false;
        }
        if (!DataFlash_Class::instance()->CardInserted()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: No SD card");
            }
            return false;
        }
    }
    return true;
}

bool AP_Arming::ins_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        const AP_InertialSensor &ins = ahrs.get_ins();
        if (!ins.get_gyro_health_all()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Gyros not healthy");
            }
            return false;
        }
        if (!ins.gyro_calibrated_ok_all()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Gyros not calibrated");
            }
            return false;
        }
        if (!ins.get_accel_health_all()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Accels not healthy");
            }
            return false;
        }
        if (!ins.accel_calibrated_ok_all()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: 3D Accel calibration needed");
            }
            return false;
        }
        
        //check if accelerometers have calibrated and require reboot
        if (ins.accel_cal_requires_reboot()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Accels calibrated requires reboot");
            }
            return false;
        }

        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
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
                    if (report) {
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Accels inconsistent");
                    }
                    return false;
                }
            }
        }

        // check all gyros are giving consistent readings
        if (ins.get_gyro_count() > 1) {
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
                    if (report) {
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Gyros inconsistent");
                    }
                    return false;
                }
            }
        }
    }

    return true;
}

bool AP_Arming::compass_checks(bool report)
{
    if ((checks_to_perform) & ARMING_CHECK_ALL ||
        (checks_to_perform) & ARMING_CHECK_COMPASS) {

        if (!_compass.use_for_yaw()) {
            // compass use is disabled
            return true;
        }

        if (!_compass.healthy()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass not healthy");
            }
            return false;
        }
        // check compass learning is on or offsets have been set
        if (!_compass.learn_offsets_enabled() && !_compass.configured()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass not calibrated");
            }
            return false;
        }

        //check if compass is calibrating
        if (_compass.is_calibrating()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibration running");
            }
            return false;
        }

        //check if compass has calibrated and requires reboot
        if (_compass.compass_cal_requires_reboot()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibrated requires reboot");
            }
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > _compass.get_offsets_max()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass offsets too high");
            }
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = _compass.get_field().length();
        if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Check mag field");
            }
            return false;
        }

        // check all compasses point in roughly same direction
        if (!_compass.consistent()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Compasses inconsistent");
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::gps_checks(bool report)
{
    const AP_GPS &gps = AP::gps();
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_GPS)) {

        //GPS OK?
        if (home_status() == HOME_UNSET ||
            gps.status() < AP_GPS::GPS_OK_FIX_3D) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Bad GPS Position");
            }
            return false;
        }

        //GPS update rate acceptable
        if (!gps.is_healthy()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: GPS is not healthy");
            }
            return false;
        }

        // check GPSs are within 50m of each other and that blending is healthy
        float distance_m;
        if (!gps.all_consistent(distance_m)) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,
                                                 "PreArm: GPS positions differ by %4.1fm",
                                                 (double)distance_m);
            }
            return false;
        }
        if (!gps.blend_health_check()) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: GPS blending unhealthy");
            }
            return false;
        }

        // check AHRS and GPS are within 10m of each other
        Location gps_loc = gps.location();
        Location ahrs_loc;
        if (ahrs.get_position(ahrs_loc)) {
            float distance = location_3d_diff_NED(gps_loc, ahrs_loc).length();
            if (distance > AP_ARMING_AHRS_GPS_ERROR_MAX) {
                if (report) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: GPS and AHRS differ by %4.1fm", (double)distance);
                }
                return false;
            }
        }
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_GPS_CONFIG)) {
        uint8_t first_unconfigured = gps.first_unconfigured_gps();
        if (first_unconfigured != AP_GPS::GPS_ALL_CONFIGURED) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,
                                                 "PreArm: GPS %d failing configuration checks",
                                                  first_unconfigured + 1);
                gps.broadcast_first_configuration_failure_reason();
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::battery_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_BATTERY)) {

        if (AP_Notify::flags.failsafe_battery) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Battery failsafe on");
            }
            return false;
        }

        for (uint8_t i = 0; i < _battery.num_instances(); i++) {
            if ((_min_voltage[i] > 0.0f) && (_battery.voltage(i) < _min_voltage[i])) {
                if (report) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Battery %d voltage %.1f below minimum %.1f",
                            i+1,
                            (double)_battery.voltage(i),
                            (double)_min_voltage[i]);
                }
                return false;
            }
        }
     }
    return true;
}

bool AP_Arming::hardware_safety_check(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_SWITCH)) {

      // check if safety switch has been pushed
      if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
          if (report) {
              gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Hardware safety switch");
          }
          return false;
      }
    }

    return true;
}

bool AP_Arming::rc_calibration_checks(bool report)
{
    return true;
}

bool AP_Arming::manual_transmitter_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_RC)) {

        if (AP_Notify::flags.failsafe_radio) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Radio failsafe on");
            }
            return false;
        }

        if (!rc_calibration_checks(report)) {
            return false;
        }
    }

    return true;
}

bool AP_Arming::board_voltage_checks(bool report)
{
#if HAL_HAVE_BOARD_VOLTAGE
    // check board voltage
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if(((hal.analogin->board_voltage() < AP_ARMING_BOARD_VOLTAGE_MIN) || (hal.analogin->board_voltage() > AP_ARMING_BOARD_VOLTAGE_MAX))) {
            if (report) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check board voltage");
            }
            return false;
        }
    }
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
    // ensure the GPS drivers are ready on any final changes
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_GPS_CONFIG)) {
        if (!AP::gps().prepare_for_arming()) {
            return false;
        }
    }

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

// Copter and sub share the same RC input limits
// Copter checks that min and max have been configured by default, Sub does not
bool AP_Arming::rc_checks_copter_sub(const bool display_failure, const RC_Channel *channels[4], const bool check_min_max) const
{
    // set rc-checks to success if RC checks are disabled
    if ((checks_to_perform != ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_RC)) {
        return true;
    }

    bool ret = true;

    const char *channel_names[] = { "Roll", "Pitch", "Throttle", "Yaw" };

    for (uint8_t i=0; i<ARRAY_SIZE(channel_names);i++) {
        const RC_Channel *channel = channels[i];
        const char *channel_name = channel_names[i];
        // check if radio has been calibrated
        if (check_min_max && !channel->min_max_configured()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: RC %s not configured", channel_name);
            }
            ret = false;
        }
        if (channel->get_radio_min() > 1300) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: %s radio min too high", channel_name);
            }
            ret = false;
        }
        if (channel->get_radio_max() < 1700) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: %s radio max too low", channel_name);
            }
            ret = false;
        }
        bool fail = true;
        if (i == 2) {
            // skip checking trim for throttle as older code did not check it
            fail = false;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: %s radio trim below min", channel_name);
            }
            if (fail) {
                ret = false;
            }
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            if (display_failure) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"PreArm: %s radio trim above max", channel_name);
            }
            if (fail) {
                ret = false;
            }
        }
    }
    return ret;
}
