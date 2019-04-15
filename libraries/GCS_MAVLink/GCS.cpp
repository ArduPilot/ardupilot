#include "GCS.h"
#include <AP_Logger/AP_Logger.h>

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

/*
  send a text message to all GCS
 */
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    hal.util->vsnprintf(text, sizeof(text), fmt, arg_list);
    send_statustext(severity, GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask(), text);
}

void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list);
    va_end(arg_list);
}

#define FOR_EACH_ACTIVE_CHANNEL(methodcall)          \
    do {                                             \
        for (uint8_t i=0; i<num_gcs(); i++) {        \
            if (!chan(i).initialised) {              \
                continue;                            \
            }                                        \
            if (!(GCS_MAVLINK::active_channel_mask() & (1 << (chan(i).get_chan()-MAVLINK_COMM_0)))) { \
                continue;                            \
            }                                        \
            chan(i).methodcall;                      \
        }                                            \
    } while (0)

void GCS::send_named_float(const char *name, float value) const
{
    FOR_EACH_ACTIVE_CHANNEL(send_named_float(name, value));
}

/*
  install an alternative protocol handler. This allows another
  protocol to take over the link if MAVLink goes idle. It is used to
  allow for the AP_BLHeli pass-thru protocols to run on hal.uartA
 */
bool GCS::install_alternative_protocol(mavlink_channel_t c, GCS_MAVLINK::protocol_handler_fn_t handler)
{
    if (c >= num_gcs()) {
        return false;
    }
    if (chan(c).alternative.handler && handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    chan(c).alternative.handler = handler;
    return true;
}

#undef FOR_EACH_ACTIVE_CHANNEL

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
    if (logger.logging_present()) {  // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
    if (logger.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }
    if (!logger.logging_failed()) {
        control_sensors_health |= MAV_SYS_STATUS_LOGGING;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
    control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    update_vehicle_sensor_status_flags();
}
