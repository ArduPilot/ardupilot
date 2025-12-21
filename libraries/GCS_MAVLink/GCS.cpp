#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "MissionItemProtocol_Waypoints.h"
#include "MissionItemProtocol_Rally.h"
#include "MissionItemProtocol_Fence.h"

extern const AP_HAL::HAL& hal;

#ifndef MAV_SYSID_DEFAULT
#if APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
#define MAV_SYSID_DEFAULT 2
#else
#define MAV_SYSID_DEFAULT 1
#endif  // APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
#endif  // defined(MAV_SYSID_DEFAULT)

const AP_Param::GroupInfo GCS::var_info[] {
    // @Param: _SYSID
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network.
    // @Range: 1 255
    // @User: Advanced
    AP_GROUPINFO("_SYSID",    1,     GCS,  sysid,  MAV_SYSID_DEFAULT),

    // @Param: _GCS_SYSID
    // @DisplayName: My ground station number
    // @Description: This sets what MAVLink source system IDs are accepted for GCS failsafe handling, RC overrides and manual control. When MAV_GCS_SYSID_HI is less than MAV_GCS_SYSID then only this value is considered to be a GCS. When MAV_GCS_SYSID_HI is greater than or equal to MAV_GCS_SYSID then the range of values between MAV_GCS_SYSID and MAV_GCS_SYSID_HI (inclusive) are all treated as valid GCS MAVLink system IDs
    // @Range: 1 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_GCS_SYSID",    2,      GCS, mav_gcs_sysid, 255),

    // @Param: _GCS_SYSID_HI
    // @DisplayName: ground station system ID, maximum
    // @Description: Upper limit of MAVLink source system IDs considered to be from the GCS. When this is less than MAV_GCS_SYSID then only MAV_GCS_SYSID is used as GCS ID. When this is greater than or equal to MAV_GCS_SYSID then the range of values from MAV_GCS_SYSID to MAV_GCS_SYSID_HI (inclusive) is treated as a GCS ID.
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_GCS_SYSID_HI",    5,   GCS, mav_gcs_sysid_high, 0),
    
    // @Param: _OPTIONS
    // @DisplayName: MAVLink Options
    // @Description: Alters various behaviour of the MAVLink interface
    // @Bitmask: 0:Accept MAVLink only from system IDs given by MAV_SYSID_GCS and MAV_SYSID_GCS_HI
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS",    3,      GCS, mav_options, 0),

    // @Param: _TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Advanced
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    AP_GROUPINFO("_TELEM_DELAY",    4,      GCS, mav_telem_delay, 0),

#if MAVLINK_COMM_NUM_BUFFERS > 0
    // @Group: 1
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[0], "1", 11, GCS, _chan_var_info[0]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 1
    // @Group: 2
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[1], "2", 12, GCS, _chan_var_info[1]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Group: 3
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[2], "3", 13, GCS, _chan_var_info[2]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 3
    // @Group: 4
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[3], "4", 14, GCS, _chan_var_info[3]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 4
    // @Group: 5
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[4], "5", 15, GCS, _chan_var_info[4]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 5
    // @Group: 6
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[5], "6", 16, GCS, _chan_var_info[5]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 6
    // @Group: 7
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[6], "7", 17, GCS, _chan_var_info[6]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 7
    // @Group: 8
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[7], "8", 18, GCS, _chan_var_info[7]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 8
    // @Group: 9
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[8], "9", 19, GCS, _chan_var_info[8]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 9
    // @Group: 10
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[9], "10", 20, GCS, _chan_var_info[9]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 10
    // @Group: 11
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[10], "11", 21, GCS, _chan_var_info[10]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 11
    // @Group: 12
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[11], "12", 22, GCS, _chan_var_info[11]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 12
    // @Group: 13
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[12], "13", 23, GCS, _chan_var_info[12]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 13
    // @Group: 14
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[13], "14", 24, GCS, _chan_var_info[13]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 14
    // @Group: 15
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[14], "15", 25, GCS, _chan_var_info[14]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 15
    // @Group: 16
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[15], "16", 26, GCS, _chan_var_info[15]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 16
    // @Group: 17
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[16], "17", 27, GCS, _chan_var_info[16]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 17
    // @Group: 18
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[17], "18", 28, GCS, _chan_var_info[17]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 18
    // @Group: 19
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[18], "19", 29, GCS, _chan_var_info[18]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 19
    // @Group: 20
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[19], "20", 30, GCS, _chan_var_info[19]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 20
    // @Group: 21
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[20], "21", 31, GCS, _chan_var_info[20]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 21
    // @Group: 22
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[21], "22", 32, GCS, _chan_var_info[21]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 22
    // @Group: 23
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[22], "23", 33, GCS, _chan_var_info[22]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 23
    // @Group: 24
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[23], "24", 34, GCS, _chan_var_info[23]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 24
    // @Group: 25
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[24], "25", 35, GCS, _chan_var_info[24]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 25
    // @Group: 26
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[25], "26", 36, GCS, _chan_var_info[25]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 26
    // @Group: 27
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[26], "27", 37, GCS, _chan_var_info[26]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 27
    // @Group: 28
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[27], "28", 38, GCS, _chan_var_info[27]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 28
    // @Group: 29
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[28], "29", 39, GCS, _chan_var_info[28]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 29
    // @Group: 30
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[29], "30", 40, GCS, _chan_var_info[29]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 30
    // @Group: 31
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[30], "31", 41, GCS, _chan_var_info[30]),
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 31
    // @Group: 32
    // @Path: GCS_MAVLink_Parameters.cpp
    AP_SUBGROUPVARPTR(_chan[31], "32", 42, GCS, _chan_var_info[31]),
#endif

    AP_GROUPEND
};

void GCS::get_sensor_status_flags(uint32_t &present,
                                  uint32_t &enabled,
                                  uint32_t &health)
{
// if this assert fails then fix it and the comment in GCS.h where
// _statustext_queue is declared
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
ASSERT_STORAGE_SIZE(GCS::statustext_t, 60);
#endif

    WITH_SEMAPHORE(control_sensors_sem);

    update_sensor_status_flags();

    present = control_sensors_present;
    enabled = control_sensors_enabled;
    health = control_sensors_health;
}

MissionItemProtocol *GCS::missionitemprotocols[3];

void GCS::init()
{
    mavlink_system.sysid = sysid_this_mav();
}

/*
 * returns a mask of channels that statustexts should be sent to
 */
mavlink_channel_mask_t GCS::statustext_send_channel_mask() const
{
    mavlink_channel_mask_t ret = 0;
    ret |= GCS_MAVLINK::active_channel_mask();
    ret |= GCS_MAVLINK::streaming_channel_mask();
    ret &= ~GCS_MAVLINK::private_channel_mask();
    return ret;
}

/*
  send a text message to all GCS
 */
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    mavlink_channel_mask_t mask = statustext_send_channel_mask();
    if (!update_send_has_been_called) {
        // we have not yet initialised the streaming-channel-mask,
        // which is done as part of the update() call.  So just send
        // it to all channels:
        mask = (1<<_num_gcs)-1;
    }
    send_textv(severity, fmt, arg_list, mask);
}

void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list);
    va_end(arg_list);
}

void GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) {
        return;
    }
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        if (c.is_private()) {
            continue;
        }
        if (!c.is_active()) {
            continue;
        }
#if HAL_HIGH_LATENCY2_ENABLED
        if (c.is_high_latency_link) {
            continue;
        }
#endif
        // size checks done by this method:
        c.send_message(pkt, entry);
    }
}

void GCS::send_named_float(const char *name, float value) const
{

    mavlink_named_value_float_t packet {};
    packet.time_boot_ms = AP_HAL::millis();
    packet.value = value;
    memcpy(packet.name, name, MIN(strlen(name), (uint8_t)MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN));

    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
                                  (const char *)&packet);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: NVF
// @Description: Named Value Float messages; messages sent to GCS via NAMED_VALUE_FLOAT
// @Field: TimeUS: Time since system startup
// @Field: Name: Name of float
// @Field: Value: Value of float
    AP::logger().WriteStreaming(
        "NVF",
        "TimeUS," "Name," "Value",
        "s"       "#"     "-",
        "F"       "-"     "-",
        "Q"       "N"     "f",
        AP_HAL::micros64(),
        name,
        value
    );
#endif  // HAL_LOGGING_ENABLED
}

void GCS::send_named_string(const char *name, const char *value) const
{
    mavlink_named_value_string_t packet {};
    packet.time_boot_ms = AP_HAL::millis();
    strncpy_noterm(packet.name, name, ARRAY_SIZE(packet.name));
    strncpy_noterm(packet.value, value, ARRAY_SIZE(packet.value));

    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_STRING,
                                  (const char *)&packet);

#if HAL_LOGGING_ENABLED
// NVS is also emitted in GCS_Common.cpp
// @LoggerMessage: NVS
// @Description: Named Value String messages; messages sent to GCS via NAMED_VALUE_STRING
// @Field: TimeUS: Time since system startup
// @Field: Name: Name of string
// @Field: Value: Value of string
    AP::logger().WriteStreaming(
        "NVS",
        "TimeUS," "Name," "Value",
        "s"       "#"     "-",
        "F"       "-"     "-",
        "Q"       "N"     "Z",
        AP_HAL::micros64(),
        name,
        value
    );
#endif  // HAL_LOGGING_ENABLED
}


#if HAL_HIGH_LATENCY2_ENABLED
void GCS::enable_high_latency_connections(bool enabled)
{
    high_latency_link_enabled = enabled;
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "High Latency %s", enabled ? "enabled" : "disabled");
}

bool GCS::get_high_latency_status()
{
    return high_latency_link_enabled;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

/*
  install an alternative protocol handler. This allows another
  protocol to take over the link if MAVLink goes idle. It is used to
  allow for the AP_BLHeli pass-thru protocols to run on hal.serial(0)
 */
bool GCS::install_alternative_protocol(mavlink_channel_t c, GCS_MAVLINK::protocol_handler_fn_t handler)
{
    GCS_MAVLINK *link = chan(c);
    if (link == nullptr) {
        return false;
    }
    if (link->alternative.handler && handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    link->alternative.handler = handler;
    return true;
}

// note that control_sensors_present and friends are protected by
// control_sensors_sem.  There is currently only one caller to this
// method, and it does the protection for us.
void GCS::update_sensor_status_flags()
{
    control_sensors_present = 0;
    control_sensors_enabled = 0;
    control_sensors_health = 0;

#if AP_INERTIALSENSOR_ENABLED
    const AP_InertialSensor &ins = AP::ins();
#endif

#if AP_AHRS_ENABLED && AP_INERTIALSENSOR_ENABLED
    AP_AHRS &ahrs = AP::ahrs();

    control_sensors_present |= MAV_SYS_STATUS_AHRS;
    if (ahrs.initialised()) {
        control_sensors_enabled |= MAV_SYS_STATUS_AHRS;
        if (ahrs.healthy()) {
            if (!ahrs.have_inertial_nav() || ins.accel_calibrated_ok_all()) {
                control_sensors_health |= MAV_SYS_STATUS_AHRS;
            }
        }
    }
#endif

#if AP_COMPASS_ENABLED
    const Compass &compass = AP::compass();
    if (AP::compass().available()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (compass.available() && compass.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
#endif

#if AP_BARO_ENABLED
    const AP_Baro &barometer = AP::baro();
    if (barometer.num_instances() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        if (barometer.all_healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        }
    }
#endif

#if AP_GPS_ENABLED
    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (gps.is_healthy() && gps.status() >= min_status_for_gps_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#endif

#if AP_BATTERY_ENABLED
    const AP_BattMonitor &battery = AP::battery();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
    if (battery.healthy() && !battery.has_failsafed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#endif

#if AP_INERTIALSENSOR_ENABLED
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    if (!ins.calibrating()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        if (ins.get_accel_health_all()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        }
        if (ins.get_gyro_health_all() && ins.gyro_calibrated_ok_all()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        }
    }
#endif

#if HAL_LOGGING_ENABLED
    const AP_Logger &logger = AP::logger();
    bool logging_present = logger.logging_present();
    bool logging_enabled = logger.logging_enabled();
    bool logging_healthy = !logger.logging_failed();
#if AP_GPS_ENABLED
    // some GPS units do logging, so they have to be healthy too:
    logging_present |= gps.logging_present();
    logging_enabled |= gps.logging_enabled();
    logging_healthy &= !gps.logging_failed();
#endif
    if (logging_present) {
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
    if (logging_enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }
    if (logging_healthy) {
        control_sensors_health |= MAV_SYS_STATUS_LOGGING;
    }
#endif  // HAL_LOGGING_ENABLED

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
#if !defined(HAL_BUILD_AP_PERIPH)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
    control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_AHRS_ENABLED
    if (ahrs.configured_ekf_type() == AP_AHRS::EKFType::SIM) {
        // always show EKF type 10 as healthy. This prevents spurious error
        // messages in xplane and other simulators that use EKF type 10
        control_sensors_health |= MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
#endif

#if AP_FENCE_ENABLED
    const AC_Fence *fence = AP::fence();
    if (fence != nullptr) {
        if (fence->sys_status_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
        }
        if (fence->sys_status_present()) {
            control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
        }
        if (!fence->sys_status_failed()) {
            control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;
        }
    }
#endif

    // airspeed
#if AP_AIRSPEED_ENABLED
    const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed && airspeed->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        const bool use = airspeed->use();
#if AP_AHRS_ENABLED
        const bool enabled = AP::ahrs().airspeed_sensor_enabled();
#else
        const AP_Airspeed *_airspeed = AP::airspeed();
        const bool enabled = (_airspeed != nullptr && _airspeed->use());
#endif
        if (use) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
        if (airspeed->all_healthy() && (!use || enabled)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
    }
#endif

#if AP_OPTICALFLOW_ENABLED
    const AP_OpticalFlow *optflow = AP::opticalflow();
    if (optflow && optflow->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
    if (optflow && optflow->healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif

#if HAL_VISUALODOM_ENABLED
    const AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom && visual_odom->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        if (visual_odom->healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        }
    }
#endif

    // give GCS status of prearm checks. This is enabled if any arming checks are enabled.
    // it is healthy if armed or checks are passing
#if AP_ARMING_ENABLED
    control_sensors_present |= MAV_SYS_STATUS_PREARM_CHECK;
    if (AP::arming().get_enabled_checks()) {
        control_sensors_enabled |= MAV_SYS_STATUS_PREARM_CHECK;
        if (hal.util->get_soft_armed() || AP_Notify::flags.pre_arm_check) {
            control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
        }
    }
#endif

#if AP_RC_CHANNEL_ENABLED
    if (rc().has_ever_seen_rc_input()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        if (!rc().in_rc_failsafe()) {  // should this be has_valid_input?
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        }
    }
#endif

    update_vehicle_sensor_status_flags();
}

bool GCS::out_of_time() const
{
#if defined(HAL_BUILD_AP_PERIPH)
    // we are never out of time for AP_Periph
    // as we don't have concept of AP_Scheduler in AP_Periph
    return false;
#endif
    // while we are in the delay callback we are never out of time:
    if (hal.scheduler->in_delay_callback()) {
        return false;
    }

    // we always want to be able to send messages out while in the error loop:
    if (AP_BoardConfig::in_config_error()) {
        return false;
    }

#if AP_SCHEDULER_ENABLED
    if (min_loop_time_remaining_for_message_send_us() <= AP::scheduler().time_available_usec()) {
        return false;
    }
#endif

    return true;
}

void gcs_out_of_space_to_send(mavlink_channel_t chan)
{
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return;
    }
    link->out_of_space_to_send();
}

/*
  check there is enough space for a message
 */
bool GCS_MAVLINK::check_payload_size(uint16_t max_payload_len)
{
    if (txspace() < unsigned(packet_overhead()+max_payload_len)) {
        gcs_out_of_space_to_send(chan);
        return false;
    }
    return true;
}

#if AP_SCRIPTING_ENABLED
/*
  lua access to command_int

  Note that this is called with the AP_Scheduler lock, ensuring the
  main thread does not race with a lua command_int
*/
MAV_RESULT GCS::lua_command_int_packet(const mavlink_command_int_t &packet)
{
    // for now we assume channel 0. In the future we may create a dedicated channel
    auto *ch = chan(0);
    if (ch == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }
    // we need a dummy message for some calls
    mavlink_message_t msg {};

    return ch->handle_command_int_packet(packet, msg);
}
#endif // AP_SCRIPTING_ENABLED

/*
  return true if a MAVLink system ID is a GCS for this vehicle
*/
bool GCS::sysid_is_gcs(uint8_t _sysid) const
{
    if (mav_gcs_sysid_high <= mav_gcs_sysid) {
        return mav_gcs_sysid == _sysid;
    }
    return _sysid >= mav_gcs_sysid && _sysid <= mav_gcs_sysid_high;
}

#endif  // HAL_GCS_ENABLED
