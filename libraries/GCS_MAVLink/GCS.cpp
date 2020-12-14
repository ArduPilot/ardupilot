#include "GCS.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Arming/AP_Arming.h>

extern const AP_HAL::HAL& hal;

void GCS::get_sensor_status_flags(uint32_t &present,
                                  uint32_t &enabled,
                                  uint32_t &health)
{
    update_sensor_status_flags();

    present = control_sensors_present;
    enabled = control_sensors_enabled;
    health = control_sensors_health;
}

MissionItemProtocol_Waypoints *GCS::_missionitemprotocol_waypoints;
MissionItemProtocol_Rally *GCS::_missionitemprotocol_rally;
MissionItemProtocol_Fence *GCS::_missionitemprotocol_fence;

const MAV_MISSION_TYPE GCS_MAVLINK::supported_mission_types[] = {
    MAV_MISSION_TYPE_MISSION,
    MAV_MISSION_TYPE_RALLY,
    MAV_MISSION_TYPE_FENCE,
};

void GCS::init()
{
    mavlink_system.sysid = sysid_this_mav();
}

/*
 * returns a mask of channels that statustexts should be sent to
 */
uint8_t GCS::statustext_send_channel_mask() const
{
    uint8_t ret = 0;
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
    uint8_t mask = statustext_send_channel_mask();
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
        if (entry->max_msg_len + c.packet_overhead() > c.txspace()) {
            // no room on this channel
            continue;
        }
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
}

/*
  install an alternative protocol handler. This allows another
  protocol to take over the link if MAVLink goes idle. It is used to
  allow for the AP_BLHeli pass-thru protocols to run on hal.serial(0)
 */
bool GCS::install_alternative_protocol(mavlink_channel_t c, GCS_MAVLINK::protocol_handler_fn_t handler)
{
    if (c >= num_gcs()) {
        return false;
    }
    if (chan(c)->alternative.handler && handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    chan(c)->alternative.handler = handler;
    return true;
}

void GCS::update_sensor_status_flags()
{
    control_sensors_present = 0;
    control_sensors_enabled = 0;
    control_sensors_health = 0;

    AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();

    control_sensors_present |= MAV_SYS_STATUS_AHRS;
    control_sensors_enabled |= MAV_SYS_STATUS_AHRS;
    if (!ahrs.initialised() || ahrs.healthy()) {
        if (!ahrs.have_inertial_nav() || ins.accel_calibrated_ok_all()) {
            control_sensors_health |= MAV_SYS_STATUS_AHRS;
        }
    }

    const Compass &compass = AP::compass();
    if (AP::compass().enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (compass.enabled() && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }

    const AP_Baro &barometer = AP::baro();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }

    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (gps.is_healthy() && gps.status() >= min_status_for_gps_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    const AP_BattMonitor &battery = AP::battery();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
    if (battery.healthy() && !battery.has_failsafed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }

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

    const AP_Logger &logger = AP::logger();
    if (logger.logging_present() || gps.logging_present()) {  // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
    if (logger.logging_enabled() || gps.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }
    if (!logger.logging_failed() && !gps.logging_failed()) {
        control_sensors_health |= MAV_SYS_STATUS_LOGGING;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
    control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (ahrs.get_ekf_type() == 10) {
        // always show EKF type 10 as healthy. This prevents spurious error
        // messages in xplane and other simulators that use EKF type 10
        control_sensors_health |= MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
#endif

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
    control_sensors_present |= MAV_SYS_STATUS_PREARM_CHECK;
    if (AP::arming().get_enabled_checks()) {
        control_sensors_enabled |= MAV_SYS_STATUS_PREARM_CHECK;
        if (hal.util->get_soft_armed() || AP_Notify::flags.pre_arm_check) {
            control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
        }
    }

    update_vehicle_sensor_status_flags();
}

bool GCS::out_of_time() const
{
    // while we are in the delay callback we are never out of time:
    if (hal.scheduler->in_delay_callback()) {
        return false;
    }

    // we always want to be able to send messages out while in the error loop:
    if (AP_BoardConfig::in_config_error()) {
        return false;
    }

    if (min_loop_time_remaining_for_message_send_us() <= AP::scheduler().time_available_usec()) {
        return false;
    }

    return true;
}

void gcs_out_of_space_to_send_count(mavlink_channel_t chan)
{
    gcs().chan(chan)->out_of_space_to_send();
}
