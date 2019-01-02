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
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define AP_ARMING_COMPASS_MAGFIELD_EXPECTED 530
#define AP_ARMING_COMPASS_MAGFIELD_MIN  185     // 0.35 * 530 milligauss
#define AP_ARMING_COMPASS_MAGFIELD_MAX  875     // 1.65 * 530 milligauss
#define AP_ARMING_BOARD_VOLTAGE_MAX     5.8f
#define AP_ARMING_ACCEL_ERROR_THRESHOLD 0.75f
#define AP_ARMING_AHRS_GPS_ERROR_MAX    10      // accept up to 10m difference between AHRS and GPS

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define ARMING_RUDDER_DEFAULT         ARMING_RUDDER_ARMONLY
#else
  #define ARMING_RUDDER_DEFAULT         ARMING_RUDDER_ARMDISARM
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Arming::var_info[] = {

    // @Param: REQUIRE
    // @DisplayName: Require Arming Motors 
    // @Description: Arming disabled until some requirements are met. If 0, there are no requirements (arm immediately).  If 1, require rudder stick or GCS arming before arming motors and sends the minimum throttle PWM value to the throttle channel when disarmed.  If 2, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed. See the ARMING_CHECK_* parameters to see what checks are done before arming. Note, if setting this parameter to 0 a reboot is required to arm the plane.  Also note, even with this parameter at 0, if ARMING_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre-arm check failure.
    // @Values: 0:Disabled,1:THR_MIN PWM when disarmed,2:0 PWM when disarmed
    // @User: Advanced
    AP_GROUPINFO_FLAGS_FRAME("REQUIRE",     0,      AP_Arming,  require,                 1,
                             AP_PARAM_NO_SHIFT,
                             AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_ROVER),

    // @Param: CHECK
    // @DisplayName: Arm Checks to Peform (bitmask)
    // @Description: Checks prior to arming motor. This is a bitmask of checks that will be performed before allowing arming. The default is no checks, allowing arming at any time. You can select whatever checks you prefer by adding together the values of each check type to set this parameter. For example, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING_CHECK to 72. For most users it is recommended that you set this to 1 to enable all checks.
    // @Values: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Channels,128:Board voltage,256:Battery Level,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration,8192:System
    // @Values{Plane}: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Channels,128:Board voltage,256:Battery Level,512:Airspeed,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration,8192:System
    // @Bitmask: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System
    // @Bitmask{Plane}: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,9:Airspeed,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System
    // @User: Standard
    AP_GROUPINFO("CHECK",        2,     AP_Arming,  checks_to_perform,       ARMING_CHECK_ALL),

    // @Param: ACCTHRESH
    // @DisplayName: Accelerometer error threshold
    // @Description: Accelerometer error threshold used to determine inconsistent accelerometers. Compares this error range to other accelerometers to detect a hardware or calibration error. Lower value means tighter check and harder to pass arming check. Not all accelerometers are created equal.
    // @Units: m/s/s
    // @Range: 0.25 3.0
    // @User: Advanced
    AP_GROUPINFO("ACCTHRESH",    3,     AP_Arming,  accel_error_threshold,  AP_ARMING_ACCEL_ERROR_THRESHOLD),

    // index 4 was VOLT_MIN, moved to AP_BattMonitor
    // index 5 was VOLT2_MIN, moved to AP_BattMonitor

    // @Param: RUDDER
    // @DisplayName: Arming with Rudder enable/disable
    // @Description: Allow arm/disarm by rudder input. When enabled arming can be done with right rudder, disarming with left rudder. Rudder arming only works in manual throttle modes with throttle at zero +- deadzone (RCx_DZ)
    // @Values: 0:Disabled,1:ArmingOnly,2:ArmOrDisarm
    // @User: Advanced
    AP_GROUPINFO_FRAME("RUDDER",  6,     AP_Arming, _rudder_arming, ARMING_RUDDER_DEFAULT, AP_PARAM_FRAME_PLANE |
                                                                                           AP_PARAM_FRAME_ROVER |
                                                                                           AP_PARAM_FRAME_COPTER |
                                                                                           AP_PARAM_FRAME_TRICOPTER |
                                                                                           AP_PARAM_FRAME_HELI),
    AP_GROUPEND
};

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

AP_Arming::AP_Arming()
{
    AP_Param::setup_object_defaults(this, var_info);
}

uint16_t AP_Arming::compass_magfield_expected() const
{
    return AP_ARMING_COMPASS_MAGFIELD_EXPECTED;
}

bool AP_Arming::is_armed()
{
    return (ArmingRequired)require.get() == NO || armed;
}

uint16_t AP_Arming::get_enabled_checks()
{
    return checks_to_perform;
}

bool AP_Arming::check_enabled(const enum AP_Arming::ArmingChecks check) const
{
    if (checks_to_perform & ARMING_CHECK_ALL) {
        return true;
    }
    if (checks_to_perform & ARMING_CHECK_NONE) {
        return false;
    }
    return (checks_to_perform & check);
}

MAV_SEVERITY AP_Arming::check_severity(const enum AP_Arming::ArmingChecks check) const
{
    // A check value of ARMING_CHECK_NONE means that the check is always run
    if (check_enabled(check) || check == ARMING_CHECK_NONE) {
        return MAV_SEVERITY_CRITICAL;
    }
    return MAV_SEVERITY_DEBUG; // technically should be NOTICE, but will annoy users at that level
}

void AP_Arming::check_failed(const enum AP_Arming::ArmingChecks check, bool report, const char *fmt, ...) const
{
    if (!report) {
        return;
    }
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), "PreArm: %s", fmt);
    MAV_SEVERITY severity = check_severity(check);
    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(severity, taggedfmt, arg_list);
    va_end(arg_list);
}

bool AP_Arming::barometer_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_BARO)) {
        if (!AP::baro().all_healthy()) {
            check_failed(ARMING_CHECK_BARO, report, "Barometer not healthy");
            return false;
        }
    }

    return true;
}

bool AP_Arming::airspeed_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_AIRSPEED)) {
        const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
        if (airspeed == nullptr) {
            // not an airspeed capable vehicle
            return true;
        }
        for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
            if (airspeed->enabled(i) && airspeed->use(i) && !airspeed->healthy(i)) {
                check_failed(ARMING_CHECK_AIRSPEED, report, "Airspeed %d not healthy", i + 1);
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
            check_failed(ARMING_CHECK_LOGGING, report, "Logging failed");
            return false;
        }
        if (!DataFlash_Class::instance()->CardInserted()) {
            check_failed(ARMING_CHECK_LOGGING, report, "No SD card");
            return false;
        }
    }
    return true;
}

bool AP_Arming::ins_accels_consistent(const AP_InertialSensor &ins)
{
    const uint8_t accel_count = ins.get_accel_count();
    if (accel_count <= 1) {
        return true;
    }

    const Vector3f &prime_accel_vec = ins.get_accel();
    const uint32_t now = AP_HAL::millis();
    for(uint8_t i=0; i<accel_count; i++) {
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
            last_accel_pass_ms[i] = now;
        }
        if (now - last_accel_pass_ms[i] > 10000) {
            return false;
        }
    }

    return true;
}

bool AP_Arming::ins_gyros_consistent(const AP_InertialSensor &ins)
{
    const uint8_t gyro_count = ins.get_gyro_count();
    if (gyro_count <= 1) {
        return true;
    }

    const Vector3f &prime_gyro_vec = ins.get_gyro();
    const uint32_t now = AP_HAL::millis();
    for(uint8_t i=0; i<gyro_count; i++) {
        if (!ins.use_gyro(i)) {
            continue;
        }
        // get next gyro vector
        const Vector3f &gyro_vec = ins.get_gyro(i);
        Vector3f vec_diff = gyro_vec - prime_gyro_vec;
        // allow for up to 5 degrees/s difference. Pass if it has
        // been OK in last 10 seconds
        if (vec_diff.length() <= radians(5)) {
            last_gyro_pass_ms[i] = now;
        }
        if (now - last_gyro_pass_ms[i] > 10000) {
            return false;
        }
    }

    return true;
}

bool AP_Arming::ins_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        const AP_InertialSensor &ins = AP::ins();
        if (!ins.get_gyro_health_all()) {
            check_failed(ARMING_CHECK_INS, report, "Gyros not healthy");
            return false;
        }
        if (!ins.gyro_calibrated_ok_all()) {
            check_failed(ARMING_CHECK_INS, report, "Gyros not calibrated");
            return false;
        }
        if (!ins.get_accel_health_all()) {
            check_failed(ARMING_CHECK_INS, report, "Accels not healthy");
            return false;
        }
        if (!ins.accel_calibrated_ok_all()) {
            check_failed(ARMING_CHECK_INS, report, "3D Accel calibration needed");
            return false;
        }
        
        //check if accelerometers have calibrated and require reboot
        if (ins.accel_cal_requires_reboot()) {
            check_failed(ARMING_CHECK_INS, report, "Accels calibrated requires reboot");
            return false;
        }

        // check all accelerometers point in roughly same direction
        if (!ins_accels_consistent(ins)) {
            check_failed(ARMING_CHECK_INS, report, "Accels inconsistent");
            return false;
        }

        // check all gyros are giving consistent readings
        if (!ins_gyros_consistent(ins)) {
            check_failed(ARMING_CHECK_INS, report, "Gyros inconsistent");
            return false;
        }
    }

    return true;
}

bool AP_Arming::compass_checks(bool report)
{
    if ((checks_to_perform) & ARMING_CHECK_ALL ||
        (checks_to_perform) & ARMING_CHECK_COMPASS) {

        Compass &_compass = AP::compass();

        // avoid Compass::use_for_yaw(void) as it implicitly calls healthy() which can
        // incorrectly skip the remaining checks, pass the primary instance directly
        if (!_compass.use_for_yaw(_compass.get_primary())) {
            // compass use is disabled
            return true;
        }

        if (!_compass.healthy()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass not healthy");
            return false;
        }
        // check compass learning is on or offsets have been set
        if (!_compass.learn_offsets_enabled() && !_compass.configured()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass not calibrated");
            return false;
        }

        //check if compass is calibrating
        if (_compass.is_calibrating()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass calibration running");
            return false;
        }

        //check if compass has calibrated and requires reboot
        if (_compass.compass_cal_requires_reboot()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass calibrated requires reboot");
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > _compass.get_offsets_max()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass offsets too high");
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = _compass.get_field().length();
        if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN) {
            check_failed(ARMING_CHECK_COMPASS, report, "Check mag field");
            return false;
        }

        // check all compasses point in roughly same direction
        if (!_compass.consistent()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compasses inconsistent");
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
        if (!AP::ahrs().home_is_set() ||
            gps.status() < AP_GPS::GPS_OK_FIX_3D) {
            check_failed(ARMING_CHECK_GPS, report, "Bad GPS Position");
            return false;
        }

        //GPS update rate acceptable
        if (!gps.is_healthy()) {
            check_failed(ARMING_CHECK_GPS, report, "GPS is not healthy");
            return false;
        }

        // check GPSs are within 50m of each other and that blending is healthy
        float distance_m;
        if (!gps.all_consistent(distance_m)) {
            check_failed(ARMING_CHECK_GPS, report, "GPS positions differ by %4.1fm",
                         (double)distance_m);
            return false;
        }
        if (!gps.blend_health_check()) {
            check_failed(ARMING_CHECK_GPS, report, "GPS blending unhealthy");
            return false;
        }

        // check AHRS and GPS are within 10m of each other
        Location gps_loc = gps.location();
        Location ahrs_loc;
        if (AP::ahrs().get_position(ahrs_loc)) {
            const float distance = location_diff(gps_loc, ahrs_loc).length();
            if (distance > AP_ARMING_AHRS_GPS_ERROR_MAX) {
                check_failed(ARMING_CHECK_GPS, report, "GPS and AHRS differ by %4.1fm", (double)distance);
                return false;
            }
        }
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_GPS_CONFIG)) {
        uint8_t first_unconfigured = gps.first_unconfigured_gps();
        if (first_unconfigured != AP_GPS::GPS_ALL_CONFIGURED) {
            check_failed(ARMING_CHECK_GPS_CONFIG,
                         report,
                         "GPS %d failing configuration checks",
                         first_unconfigured + 1);
            if (report) {
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

        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
        if (!AP::battery().arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_BATTERY, report, buffer);
            return false;
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
          check_failed(ARMING_CHECK_SWITCH, report, "Hardware safety switch");
          return false;
      }
    }

    return true;
}

bool AP_Arming::rc_calibration_checks(bool report)
{
    bool check_passed = true;
    const uint8_t num_channels = RC_Channels::get_valid_channel_count();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        const RC_Channel *c = rc().channel(i);
        if (c == nullptr) {
            continue;
        }
        if (i >= num_channels && !(c->has_override())) {
            continue;
        }
        const uint16_t trim = c->get_radio_trim();
        if (c->get_radio_min() > trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d minimum is greater than trim", i + 1);
            check_passed = false;
        }
        if (c->get_radio_max() < trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d maximum is less than trim", i + 1);
            check_passed = false;
        }
    }

    return check_passed;
}

bool AP_Arming::manual_transmitter_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_RC)) {

        if (AP_Notify::flags.failsafe_radio) {
            check_failed(ARMING_CHECK_RC, report, "Radio failsafe on");
            return false;
        }

        if (!rc_calibration_checks(report)) {
            return false;
        }
    }

    return true;
}

bool AP_Arming::servo_checks(bool report) const
{
    bool check_passed = true;
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr || c->get_function() == SRV_Channel::k_none) {
            continue;
        }

        const uint16_t trim = c->get_trim();
        if (c->get_output_min() > trim) {
            check_failed(ARMING_CHECK_NONE, report, "SERVO%d minimum is greater than trim", i + 1);
            check_passed = false;
        }
        if (c->get_output_max() < trim) {
            check_failed(ARMING_CHECK_NONE, report, "SERVO%d maximum is less than trim", i + 1);
            check_passed = false;
        }
    }

#if HAL_WITH_IO_MCU
    if (!iomcu.healthy()) {
        check_failed(ARMING_CHECK_NONE, report, "IOMCU is unhealthy");
        check_passed = false;
    }
#endif
    
    return check_passed;
}

bool AP_Arming::board_voltage_checks(bool report)
{
    // check board voltage
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
#if HAL_HAVE_BOARD_VOLTAGE
        const float bus_voltage =  hal.analogin->board_voltage();
        const float vbus_min = AP_BoardConfig::get_minimum_board_voltage();
        if(((bus_voltage < vbus_min) || (bus_voltage > AP_ARMING_BOARD_VOLTAGE_MAX))) {
            check_failed(ARMING_CHECK_VOLTAGE, report, "Board (%1.1fv) out of range %1.1f-%1.1fv", (double)bus_voltage, (double)vbus_min, (double)AP_ARMING_BOARD_VOLTAGE_MAX);
            return false;
        }
#endif // HAL_HAVE_BOARD_VOLTAGE

#if HAL_HAVE_SERVO_VOLTAGE
       const float vservo_min = AP_BoardConfig::get_minimum_servo_voltage();
        if (is_positive(vservo_min)) {
            const float servo_voltage =  hal.analogin->servorail_voltage();
            if (servo_voltage < vservo_min) {
                check_failed(ARMING_CHECK_VOLTAGE, report, "Servo voltage to low (%1.2fv < %1.2fv)", (double)servo_voltage, (double)vservo_min);
                return false;
            }
        }
#endif // HAL_HAVE_SERVO_VOLTAGE
    }

    return true;
}

/*
  check base system operations
 */
bool AP_Arming::system_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_SYSTEM)) {
        if (!hal.storage->healthy()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Param storage failed");
            return false;
        }
    }
    return true;
}

bool AP_Arming::pre_arm_checks(bool report)
{
#if !APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    if (armed || require == NO) {
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
        &  servo_checks(report)
        &  board_voltage_checks(report)
        &  system_checks(report);
}

bool AP_Arming::arm_checks(ArmingMethod method)
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
            check_failed(ARMING_CHECK_LOGGING, true, "Logging not started");
            return false;
        }
    }
    return true;
}

//returns true if arming occurred successfully
bool AP_Arming::arm(AP_Arming::ArmingMethod method, const bool do_arming_checks)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // Copter should never use this function
    return false;
#else
    if (armed) { //already armed
        return false;
    }

    //are arming checks disabled?
    if (!do_arming_checks || checks_to_perform == ARMING_CHECK_NONE) {
        armed = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");
        return true;
    }

    if (pre_arm_checks(true) && arm_checks(method)) {
        armed = true;

        gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");

        //TODO: Log motor arming to the dataflash
        //Can't do this from this class until there is a unified logging library

    } else {
        armed = false;
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
        if (channel->get_radio_min() > 1300) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio min too high", channel_name);
            ret = false;
        }
        if (channel->get_radio_max() < 1700) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio max too low", channel_name);
            ret = false;
        }
        bool fail = true;
        if (i == 2) {
            // skip checking trim for throttle as older code did not check it
            fail = false;
        }
        if (channel->get_radio_trim() < channel->get_radio_min()) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio trim below min", channel_name);
            if (fail) {
                ret = false;
            }
        }
        if (channel->get_radio_trim() > channel->get_radio_max()) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio trim above max", channel_name);
            if (fail) {
                ret = false;
            }
        }
    }
    return ret;
}
