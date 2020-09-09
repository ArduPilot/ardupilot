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
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Rally/AP_Rally.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Generator/AP_Generator_RichenPower.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_OSD/AP_OSD.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
  #include <AP_CANManager/AP_CANManager.h>
  #include <AP_Common/AP_Common.h>
  #include <AP_Vehicle/AP_Vehicle.h>

  #include <AP_PiccoloCAN/AP_PiccoloCAN.h>

  // To be replaced with macro saying if KDECAN library is included
  #if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
    #include <AP_KDECAN/AP_KDECAN.h>
  #endif
  #include <AP_UAVCAN/AP_UAVCAN.h>
#endif

#include <AP_Logger/AP_Logger.h>

#define AP_ARMING_COMPASS_MAGFIELD_EXPECTED 530
#define AP_ARMING_COMPASS_MAGFIELD_MIN  185     // 0.35 * 530 milligauss
#define AP_ARMING_COMPASS_MAGFIELD_MAX  875     // 1.65 * 530 milligauss
#define AP_ARMING_BOARD_VOLTAGE_MAX     5.8f
#define AP_ARMING_ACCEL_ERROR_THRESHOLD 0.75f
#define AP_ARMING_AHRS_GPS_ERROR_MAX    10      // accept up to 10m difference between AHRS and GPS

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define ARMING_RUDDER_DEFAULT         (uint8_t)RudderArming::ARMONLY
#else
  #define ARMING_RUDDER_DEFAULT         (uint8_t)RudderArming::ARMDISARM
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Arming::var_info[] = {

    // @Param{Plane, Rover}: REQUIRE
    // @DisplayName: Require Arming Motors 
    // @Description: Arming disabled until some requirements are met. If 0, there are no requirements (arm immediately).  If 1, require rudder stick or GCS arming before arming motors and sends the minimum throttle PWM value to the throttle channel when disarmed.  If 2, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed. See the ARMING_CHECK_* parameters to see what checks are done before arming. Note, if setting this parameter to 0 a reboot is required to arm the plane.  Also note, even with this parameter at 0, if ARMING_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre-arm check failure.
    // @Values: 0:Disabled,1:THR_MIN PWM when disarmed,2:0 PWM when disarmed
    // @User: Advanced
    AP_GROUPINFO_FLAGS_FRAME("REQUIRE",     0,      AP_Arming,  require,                 1,
                             AP_PARAM_NO_SHIFT,
                             AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_ROVER),

    // 2 was the CHECK paramter stored in a AP_Int16

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

    // @Param: MIS_ITEMS
    // @DisplayName: Required mission items
    // @Description: Bitmask of mission items that are required to be planned in order to arm the aircraft
    // @Bitmask: 0:Land,1:VTOL Land,2:DO_LAND_START,3:Takeoff,4:VTOL Takeoff,5:Rallypoint
    // @User: Advanced
    AP_GROUPINFO("MIS_ITEMS",    7,     AP_Arming, _required_mission_items, 0),

    // @Param: CHECK
    // @DisplayName: Arm Checks to Perform (bitmask)
    // @Description: Checks prior to arming motor. This is a bitmask of checks that will be performed before allowing arming. The default is no checks, allowing arming at any time. You can select whatever checks you prefer by adding together the values of each check type to set this parameter. For example, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING_CHECK to 72. For most users it is recommended that you set this to 1 to enable all checks.
    // @Values: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Channels,128:Board voltage,256:Battery Level,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration,8192:System,16384:Mission,32768:RangeFinder,65536:Camera,131072:AuxAuth,524288:FFT
    // @Values{Plane}: 0:None,1:All,2:Barometer,4:Compass,8:GPS Lock,16:INS(INertial Sensors - accels & gyros),32:Parameters(unused),64:RC Channels,128:Board voltage,256:Battery Level,512:Airspeed,1024:LoggingAvailable,2048:Hardware safety switch,4096:GPS configuration,8192:System,16384:Mission,32768:RangeFinder,65536:Camera,131072:AuxAuth,524288:FFT
    // @Bitmask: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System,14:Mission,15:Rangefinder,16:Camera,17:AuxAuth,18:VisualOdometry,19:FFT
    // @Bitmask{Plane}: 0:All,1:Barometer,2:Compass,3:GPS lock,4:INS,5:Parameters,6:RC Channels,7:Board voltage,8:Battery Level,9:Airspeed,10:Logging Available,11:Hardware safety switch,12:GPS Configuration,13:System,14:Mission,15:Rangefinder,16:Camera,17:AuxAuth,19:FFT
    // @User: Standard
    AP_GROUPINFO("CHECK",        8,     AP_Arming,  checks_to_perform,       ARMING_CHECK_ALL),

    AP_GROUPEND
};

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

AP_Arming::AP_Arming()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_Arming instances");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

uint16_t AP_Arming::compass_magfield_expected() const
{
    return AP_ARMING_COMPASS_MAGFIELD_EXPECTED;
}

bool AP_Arming::is_armed()
{
    return (Required)require.get() == Required::NO || armed;
}

uint32_t AP_Arming::get_enabled_checks() const
{
    return checks_to_perform;
}

bool AP_Arming::check_enabled(const enum AP_Arming::ArmingChecks check) const
{
    if (checks_to_perform & ARMING_CHECK_ALL) {
        return true;
    }
    return (checks_to_perform & check);
}

MAV_SEVERITY AP_Arming::check_severity(const enum AP_Arming::ArmingChecks check) const
{
    if (check_enabled(check)) {
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

void AP_Arming::check_failed(bool report, const char *fmt, ...) const
{
    if (!report) {
        return;
    }
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), "PreArm: %s", fmt);
    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(MAV_SEVERITY_CRITICAL, taggedfmt, arg_list);
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
        if (!AP::logger().logging_present()) {
            // Logging is disabled, so nothing to check.
            return true;
        }
        if (AP::logger().logging_failed()) {
            check_failed(ARMING_CHECK_LOGGING, report, "Logging failed");
            return false;
        }
        if (!AP::logger().CardInserted()) {
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
        const Vector3f vec_diff = gyro_vec - prime_gyro_vec;
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

        // check AHRS attitudes are consistent
        char failure_msg[50] = {};
        if (!AP::ahrs().attitudes_consistent(failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_INS, report, "%s", failure_msg);
            return false;
        }

#if HAL_GYROFFT_ENABLED
        // gyros are healthy so check the FFT
        if ((checks_to_perform & ARMING_CHECK_ALL) ||
            (checks_to_perform & ARMING_CHECK_FFT)) {
            // Check that the noise analyser works
            AP_GyroFFT *fft = AP::fft();

            char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
            if (fft != nullptr && !fft->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                check_failed(ARMING_CHECK_INS, report, "%s", fail_msg);
                return false;
            }
        }
#endif
    }

    return true;
}

bool AP_Arming::compass_checks(bool report)
{
    Compass &_compass = AP::compass();

    // check if compass is calibrating
    if (_compass.is_calibrating()) {
        check_failed(report, "Compass calibration running");
        return false;
    }

    // check if compass has calibrated and requires reboot
    if (_compass.compass_cal_requires_reboot()) {
        check_failed(report, "Compass calibrated requires reboot");
        return false;
    }

    if ((checks_to_perform) & ARMING_CHECK_ALL ||
        (checks_to_perform) & ARMING_CHECK_COMPASS) {

        // avoid Compass::use_for_yaw(void) as it implicitly calls healthy() which can
        // incorrectly skip the remaining checks, pass the primary instance directly
        if (!_compass.use_for_yaw(0)) {
            // compass use is disabled
            return true;
        }

        if (!_compass.healthy()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass not healthy");
            return false;
        }
        // check compass learning is on or offsets have been set
        if (!_compass.learn_offsets_enabled()) {
            char failure_msg[50] = {};
            if (!_compass.configured(failure_msg, ARRAY_SIZE(failure_msg))) {
                check_failed(ARMING_CHECK_COMPASS, report, "%s", failure_msg);
                return false;
            }
        }

        // check for unreasonable compass offsets
        const Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > _compass.get_offsets_max()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass offsets too high");
            return false;
        }

        // check for unreasonable mag field length
        const float mag_field = _compass.get_field().length();
        if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN) {
            check_failed(ARMING_CHECK_COMPASS, report, "Check mag field: %4.0f, max %d, min %d", (double)mag_field, AP_ARMING_COMPASS_MAGFIELD_MAX, AP_ARMING_COMPASS_MAGFIELD_MIN);
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
        const Location gps_loc = gps.location();
        Location ahrs_loc;
        if (AP::ahrs().get_position(ahrs_loc)) {
            const float distance = gps_loc.get_distance(ahrs_loc);
            if (distance > AP_ARMING_AHRS_GPS_ERROR_MAX) {
                check_failed(ARMING_CHECK_GPS, report, "GPS and AHRS differ by %4.1fm", (double)distance);
                return false;
            }
        }
    }

    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_GPS_CONFIG)) {
        uint8_t first_unconfigured;
        if (gps.first_unconfigured_gps(first_unconfigured)) {
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
            check_failed(ARMING_CHECK_BATTERY, report, "%s", buffer);
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

bool AP_Arming::rc_arm_checks(AP_Arming::Method method)
{
    // don't check the trims if we are in a failsafe
    if (!rc().has_valid_input()) {
        return true;
    }

    // only check if we've received some form of input within the last second
    // this is a protection against a vehicle having never enabled an input
    uint32_t last_input_ms = rc().last_input_ms();
    if ((last_input_ms == 0) || ((AP_HAL::millis() - last_input_ms) > 1000)) {
        return true;
    }

    bool check_passed = true;
    // ensure all rc channels have different functions
    if (rc().duplicate_options_exist()) {
        check_failed(ARMING_CHECK_PARAMETERS, true, "Duplicate Aux Switch Options");
        check_passed = false;
    }
    const RCMapper * rcmap = AP::rcmap();
    if (rcmap != nullptr) {
        if (!rc().arming_skip_checks_rpy()) {
            const char *names[3] = {"Roll", "Pitch", "Yaw"};
            const uint8_t channels[3] = {rcmap->roll(), rcmap->pitch(), rcmap->yaw()};
            for (uint8_t i = 0; i < ARRAY_SIZE(channels); i++) {
                const RC_Channel *c = rc().channel(channels[i] - 1);
                if (c == nullptr) {
                    continue;
                }
                if (c->get_control_in() != 0) {
                    if ((method != Method::RUDDER) || (c != rc().get_arming_channel())) { // ignore the yaw input channel if rudder arming
                        check_failed(ARMING_CHECK_RC, true, "%s (RC%d) is not neutral", names[i], channels[i]);
                        check_passed = false;
                    }
                }
            }
        }

        // if throttle check is enabled, require zero input
        if (rc().arming_check_throttle()) {
            RC_Channel *c = rc().channel(rcmap->throttle() - 1);
            if (c != nullptr) {
                if (c->get_control_in() != 0) {
                    check_failed(ARMING_CHECK_RC, true, "Throttle (RC%d) is not neutral", rcmap->throttle());
                    check_passed = false;
                }
            }
            c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FWD_THR);
            if (c != nullptr) {
                uint8_t fwd_thr = c->percent_input();
                // require channel input within 2% of minimum
                if (fwd_thr > 2) {
                    check_failed(ARMING_CHECK_RC, true, "VTOL Fwd Throttle is not zero");
                    check_passed = false;
                }
            }
        }
    }
    return check_passed;
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
            check_failed(ARMING_CHECK_RC, report, "RC%d_MIN is greater than RC%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
        if (c->get_radio_max() < trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d_MAX is less than RC%d_TRIM", i + 1, i + 1);
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

bool AP_Arming::mission_checks(bool report)
{
    if (((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_MISSION)) &&
        _required_mission_items) {
        AP_Mission *mission = AP::mission();
        if (mission == nullptr) {
            check_failed(ARMING_CHECK_MISSION, report, "No mission library present");
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                AP_HAL::panic("Mission checks requested, but no mission was allocated");
            #endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
            return false;
        }
        AP_Rally *rally = AP::rally();
        if (rally == nullptr) {
            check_failed(ARMING_CHECK_MISSION, report, "No rally library present");
            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                AP_HAL::panic("Mission checks requested, but no rally was allocated");
            #endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
            return false;
        }

        const struct MisItemTable {
          MIS_ITEM_CHECK check;
          MAV_CMD mis_item_type;
          const char *type;
        } misChecks[5] = {
          {MIS_ITEM_CHECK_LAND,          MAV_CMD_NAV_LAND,           "land"},
          {MIS_ITEM_CHECK_VTOL_LAND,     MAV_CMD_NAV_VTOL_LAND,      "vtol land"},
          {MIS_ITEM_CHECK_DO_LAND_START, MAV_CMD_DO_LAND_START,      "do land start"},
          {MIS_ITEM_CHECK_TAKEOFF,       MAV_CMD_NAV_TAKEOFF,        "takeoff"},
          {MIS_ITEM_CHECK_VTOL_TAKEOFF,  MAV_CMD_NAV_VTOL_TAKEOFF,   "vtol takeoff"},
        };
        for (uint8_t i = 0; i < ARRAY_SIZE(misChecks); i++) {
            if (_required_mission_items & misChecks[i].check) {
                if (!mission->contains_item(misChecks[i].mis_item_type)) {
                    check_failed(ARMING_CHECK_MISSION, report, "Missing mission item: %s", misChecks[i].type);
                    return false;
                }
            }
        }
        if (_required_mission_items & MIS_ITEM_CHECK_RALLY) {
            Location ahrs_loc;
            if (!AP::ahrs().get_position(ahrs_loc)) {
                check_failed(ARMING_CHECK_MISSION, report, "Can't check rally without position");
                return false;
            }
            RallyLocation rally_loc = {};
            if (!rally->find_nearest_rally_point(ahrs_loc, rally_loc)) {
                check_failed(ARMING_CHECK_MISSION, report, "No sufficently close rally point located");
                return false;
            }
          }
    }

    return true;
}

bool AP_Arming::rangefinder_checks(bool report)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RANGEFINDER)) {
        RangeFinder *range = RangeFinder::get_singleton();
        if (range == nullptr) {
            return true;
        }

        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!range->prearm_healthy(buffer, ARRAY_SIZE(buffer))) {
            check_failed(ARMING_CHECK_RANGEFINDER, report, "%s", buffer);
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
            check_failed(report, "SERVO%d_MIN is greater than SERVO%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
        if (c->get_output_max() < trim) {
            check_failed(report, "SERVO%d_MAX is less than SERVO%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
    }

#if HAL_WITH_IO_MCU
    if (!iomcu.healthy()) {
        check_failed(report, "IOMCU is unhealthy");
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
#if AP_TERRAIN_AVAILABLE
        const AP_Terrain *terrain = AP_Terrain::get_singleton();
        if ((terrain != nullptr) && terrain->init_failed()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Terrain out of memory");
            return false;
        }
#endif
#ifdef ENABLE_SCRIPTING
        const AP_Scripting *scripting = AP_Scripting::get_singleton();
        if ((scripting != nullptr) && scripting->enabled() && scripting->init_failed()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Scripting out of memory");
            return false;
        }
#endif
    }
    if (AP::internalerror().errors() != 0) {
        uint8_t buffer[32];
        AP::internalerror().errors_as_string(buffer, ARRAY_SIZE(buffer));
        check_failed(report, "Internal errors 0x%x l:%u %s", (unsigned int)AP::internalerror().errors(), AP::internalerror().last_error_line(), buffer);
        return false;
    }

    return true;
}


// check nothing is too close to vehicle
bool AP_Arming::proximity_checks(bool report) const
{
    const AP_Proximity *proximity = AP::proximity();
    // return true immediately if no sensor present
    if (proximity == nullptr) {
        return true;
    }
    if (proximity->get_status() == AP_Proximity::Status::NotConnected) {
        return true;
    }

    // return false if proximity sensor unhealthy
    if (proximity->get_status() < AP_Proximity::Status::Good) {
        check_failed(report, "check proximity sensor");
        return false;
    }

    return true;
}

bool AP_Arming::can_checks(bool report)
{
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    if (check_enabled(ARMING_CHECK_SYSTEM)) {
        char fail_msg[50] = {};
        uint8_t num_drivers = AP::can().get_num_drivers();

        for (uint8_t i = 0; i < num_drivers; i++) {
            switch (AP::can().get_driver_type(i)) {
                case AP_CANManager::Driver_Type_KDECAN: {
// To be replaced with macro saying if KDECAN library is included
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
                    AP_KDECAN *ap_kdecan = AP_KDECAN::get_kdecan(i);
                    if (ap_kdecan != nullptr && !ap_kdecan->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "KDECAN: %s", fail_msg);
                        return false;
                    }
#endif
                    break;
                }
                case AP_CANManager::Driver_Type_PiccoloCAN: {
#if HAL_PICCOLO_CAN_ENABLE
                    AP_PiccoloCAN *ap_pcan = AP_PiccoloCAN::get_pcan(i);

                    if (ap_pcan != nullptr && !ap_pcan->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "PiccoloCAN: %s", fail_msg);
                        return false;
                    }

#else
                    check_failed(ARMING_CHECK_SYSTEM, report, "PiccoloCAN not enabled");
                    return false;
#endif
                    break;
                }
                case AP_CANManager::Driver_Type_UAVCAN:
                {
                    if (!AP::uavcan_dna_server().prearm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "UAVCAN: %s", fail_msg);
                        return false;
                    }
                    break;
                }
                case AP_CANManager::Driver_Type_ToshibaCAN:
                {
                    // toshibacan doesn't currently have any prearm
                    // checks.  Theres scope for adding a "not
                    // initialised" prearm check.
                    break;
                }
                case AP_CANManager::Driver_Type_CANTester:
                {
                    check_failed(ARMING_CHECK_SYSTEM, report, "TestCAN: No Arming with TestCAN enabled");
                    break;
                }
                case AP_CANManager::Driver_Type_None:
                    break;
            }
        }
    }
#endif
    return true;
}


bool AP_Arming::fence_checks(bool display_failure)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return true;
    }

    // check fence is ready
    const char *fail_msg = nullptr;
    if (fence->pre_arm_check(fail_msg)) {
        return true;
    }

    if (fail_msg == nullptr) {
        check_failed(display_failure, "Check fence");
    } else {
        check_failed(display_failure, "%s", fail_msg);
    }

    return false;
}

bool AP_Arming::camera_checks(bool display_failure)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_CAMERA)) {
#if HAL_RUNCAM_ENABLED
        AP_RunCam *runcam = AP::runcam();
        if (runcam == nullptr) {
            return true;
        }

        // check camera is ready
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!runcam->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_CAMERA, display_failure, "%s", fail_msg);
            return false;
        }
#endif
    }
    return true;
}

bool AP_Arming::osd_checks(bool display_failure) const
{
#if OSD_ENABLED
    if ((checks_to_perform & ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_CAMERA)) {
        const AP_OSD *osd = AP::osd();
        if (osd == nullptr) {
            return true;
        }

        // check camera is ready
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!osd->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_CAMERA, display_failure, "%s", fail_msg);
            return false;
        }
    }
#endif
    return true;
}

// request an auxiliary authorisation id.  This id should be used in subsequent calls to set_aux_auth_passed/failed
// returns true on success
bool AP_Arming::get_aux_auth_id(uint8_t& auth_id)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // check we have enough room to allocate another id
    if (aux_auth_count >= aux_auth_count_max) {
        aux_auth_error = true;
        return false;
    }

    // allocate buffer for failure message
    if (aux_auth_fail_msg == nullptr) {
        aux_auth_fail_msg = (char *)calloc(aux_auth_str_len, sizeof(char));
        if (aux_auth_fail_msg == nullptr) {
            aux_auth_error = true;
            return false;
        }
    }
    auth_id = aux_auth_count;
    aux_auth_count++;
    return true;
}

// set auxiliary authorisation passed
void AP_Arming::set_aux_auth_passed(uint8_t auth_id)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // sanity check auth_id
    if (auth_id >= aux_auth_count) {
        return;
    }

    aux_auth_state[auth_id] = AuxAuthStates::AUTH_PASSED;
}

// set auxiliary authorisation failed and provide failure message
void AP_Arming::set_aux_auth_failed(uint8_t auth_id, const char* fail_msg)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // sanity check auth_id
    if (auth_id >= aux_auth_count) {
        return;
    }

    // update state
    aux_auth_state[auth_id] = AuxAuthStates::AUTH_FAILED;

    // store failure message if this authoriser has the lowest auth_id
    for (uint8_t i = 0; i < auth_id; i++) {
        if (aux_auth_state[i] == AuxAuthStates::AUTH_FAILED) {
            return;
        }
    }
    if (aux_auth_fail_msg != nullptr) {
        if (fail_msg == nullptr) {
            strncpy(aux_auth_fail_msg, "Auxiliary authorisation refused", aux_auth_str_len);
        } else {
            strncpy(aux_auth_fail_msg, fail_msg, aux_auth_str_len);
        }
        aux_auth_fail_msg_source = auth_id;
    }
}

bool AP_Arming::aux_auth_checks(bool display_failure)
{
    // handle error cases
    if (aux_auth_error) {
        if (aux_auth_fail_msg == nullptr) {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "memory low for auxiliary authorisation");
        } else {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Too many auxiliary authorisers");
        }
        return false;
    }

    WITH_SEMAPHORE(aux_auth_sem);

    // check results for each auxiliary authorisation id
    bool some_failures = false;
    bool failure_msg_sent = false;
    bool waiting_for_responses = false;
    for (uint8_t i = 0; i < aux_auth_count; i++) {
        switch (aux_auth_state[i]) {
        case AuxAuthStates::NO_RESPONSE:
            waiting_for_responses = true;
            break;
        case AuxAuthStates::AUTH_FAILED:
            some_failures = true;
            if (i == aux_auth_fail_msg_source) {
                check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "%s", aux_auth_fail_msg);
                failure_msg_sent = true;
            }
            break;
        case AuxAuthStates::AUTH_PASSED:
            break;
        }
    }

    // send failure or waiting message
    if (some_failures && !failure_msg_sent) {
        check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Auxiliary authorisation refused");
        return false;
    } else if (waiting_for_responses) {
        check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Waiting for auxiliary authorisation");
        return false;
    }

    // if we got this far all auxiliary checks must have passed
    return true;
}

bool AP_Arming::generator_checks(bool display_failure) const
{
#if GENERATOR_ENABLED
    const AP_Generator_RichenPower *generator = AP::generator();
    if (generator == nullptr) {
        return true;
    }
    char failure_msg[50] = {};
    if (!generator->pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "Generator: %s", failure_msg);
        return false;
    }
#endif
    return true;
}

bool AP_Arming::pre_arm_checks(bool report)
{
#if !APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    if (armed || require == (uint8_t)Required::NO) {
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
        &  mission_checks(report)
        &  rangefinder_checks(report)
        &  servo_checks(report)
        &  board_voltage_checks(report)
        &  system_checks(report)
        &  can_checks(report)
        &  generator_checks(report)
        &  proximity_checks(report)
        &  camera_checks(report)
        &  osd_checks(report)
        &  visodom_checks(report)
        &  aux_auth_checks(report)
        &  disarm_switch_checks(report);
}

bool AP_Arming::arm_checks(AP_Arming::Method method)
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_RC)) {
        if (!rc_arm_checks(method)) {
            return false;
        }
    }

#if HAL_GYROFFT_ENABLED
    // make sure the FFT subsystem is enabled if arming checks have been disabled
    AP_GyroFFT *fft = AP::fft();
    if (fft != nullptr) {
        fft->prepare_for_arming();
    }
#endif
    // ensure the GPS drivers are ready on any final changes
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_GPS_CONFIG)) {
        if (!AP::gps().prepare_for_arming()) {
            return false;
        }
    }
    
    // note that this will prepare AP_Logger to start logging
    // so should be the last check to be done before arming

    // Note also that we need to PrepForArming() regardless of whether
    // the arming check flag is set - disabling the arming check
    // should not stop logging from working.

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger->logging_present()) {
        // If we're configured to log, prep it
        logger->PrepForArming();
        if (!logger->logging_started() &&
            ((checks_to_perform & ARMING_CHECK_ALL) ||
             (checks_to_perform & ARMING_CHECK_LOGGING))) {
            check_failed(ARMING_CHECK_LOGGING, true, "Logging not started");
            return false;
        }
    }
    return true;
}

//returns true if arming occurred successfully
bool AP_Arming::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    if (armed) { //already armed
        return false;
    }

    if ((!do_arming_checks && mandatory_checks(true)) || (pre_arm_checks(true) && arm_checks(method))) {
        armed = true;

        Log_Write_Arm(!do_arming_checks, method); // note Log_Write_Armed takes forced not do_arming_checks

    } else {
        AP::logger().arming_failure();
        armed = false;
    }

    return armed;
}

//returns true if disarming occurred successfully
bool AP_Arming::disarm(const AP_Arming::Method method)
{
    if (!armed) { // already disarmed
        return false;
    }
    armed = false;
    _last_disarm_method = method;

    Log_Write_Disarm(method); // should be able to pass through force here?

#if HAL_HAVE_SAFETY_SWITCH
    AP_BoardConfig *board_cfg = AP_BoardConfig::get_singleton();
    if ((board_cfg != nullptr) &&
        (board_cfg->get_safety_button_options() & AP_BoardConfig::BOARD_SAFETY_OPTION_SAFETY_ON_DISARM)) {
        hal.rcout->force_safety_on();
    }
#endif // HAL_HAVE_SAFETY_SWITCH

#if HAL_GYROFFT_ENABLED
    AP_GyroFFT *fft = AP::fft();
    if (fft != nullptr) {
        fft->save_params_on_disarm();
    }
#endif

    return true;
}

AP_Arming::Required AP_Arming::arming_required() 
{
    return (AP_Arming::Required)require.get();
}

// Copter and sub share the same RC input limits
// Copter checks that min and max have been configured by default, Sub does not
bool AP_Arming::rc_checks_copter_sub(const bool display_failure, const RC_Channel *channels[4]) const
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

// check visual odometry is working
bool AP_Arming::visodom_checks(bool display_failure) const
{
    if ((checks_to_perform != ARMING_CHECK_ALL) && !(checks_to_perform & ARMING_CHECK_VISION)) {
        return true;
    }

#if HAL_VISUALODOM_ENABLED
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom != nullptr) {
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        if (!visual_odom->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_VISION, display_failure, "VisOdom: %s", fail_msg);
            return false;
        }
    }
#endif

    return true;
}

// check disarm switch is asserted
bool AP_Arming::disarm_switch_checks(bool display_failure) const
{
    const RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM);
    if (chan != nullptr &&
        chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH &&
        (checks_to_perform & ARMING_CHECK_ALL)) {
        check_failed(display_failure, "Disarm Switch on");
        return false;
    }

    return true;
}

void AP_Arming::Log_Write_Arm(const bool forced, const AP_Arming::Method method)
{
    const struct log_Arm_Disarm pkt {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : is_armed(),
        arm_checks              : get_enabled_checks(),
        forced                  : forced,
        method                  : (uint8_t)method,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
    AP::logger().Write_Event(LogEvent::ARMED);
}

void AP_Arming::Log_Write_Disarm(const AP_Arming::Method method)
{
    const struct log_Arm_Disarm pkt {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : is_armed(),
        arm_checks              : 0,
        forced                  : 0,
        method                  : (uint8_t)method
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
    AP::logger().Write_Event(LogEvent::DISARMED);
}

AP_Arming *AP_Arming::_singleton = nullptr;

/*
 * Get the AP_InertialSensor singleton
 */
AP_Arming *AP_Arming::get_singleton()
{
    return AP_Arming::_singleton;
}

namespace AP {

AP_Arming &arming()
{
    return *AP_Arming::get_singleton();
}

};
