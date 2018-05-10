#include "GCS.h"

extern const AP_HAL::HAL& hal;

uint32_t GCS::control_sensors_present = 0;
uint32_t GCS::control_sensors_enabled = 0;
uint32_t GCS::control_sensors_health = 0;

AP_GPS::GPS_Status GCS::min_gps_state() const {
    return AP_GPS::NO_FIX;
}

void GCS::update_sensor_status_flags()
{
    control_sensors_present =
        MAV_SYS_STATUS_SENSOR_3D_GYRO |
        MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_AHRS |
        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        MAV_SYS_STATUS_SENSOR_BATTERY |
        MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    control_sensors_enabled = 0;
    control_sensors_health = 0;

    // update MAV_SYS_STATUS_SENSOR_3D_MAG:
    const Compass *compass = AP::ahrs().get_compass();
    if (compass != nullptr) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        if (compass_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
            if (compass->healthy(0) && AP::ahrs().use_compass()) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
            }
        }
    }

    // update MAV_SYS_STATUS_SENSOR_GPS:
    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS; // always enabled
        if (gps.status() > min_gps_state() && gps.is_healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
        }
    }

    // Update MAV_SYS_STATUS_LOGGING:
    const DataFlash_Class *DataFlash = DataFlash_Class::instance();
    if (DataFlash->logging_present()) {  // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
        if (DataFlash->logging_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
        }
        if (!DataFlash->logging_failed()) {
            control_sensors_health |= MAV_SYS_STATUS_LOGGING;
        }
    }

    // update MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
    // marked as present in MAVLINK_SENSOR_PRESENT_DEFAULT
    // note that Sub overrides these health settings!
    const AP_Baro &barometer = AP::baro();
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }

    // Update MAV_SYS_STATUS_SENSOR_BATTERY:
    const AP_BattMonitor &battery = AP::battery();
    // marked as present in MAVLINK_SENSOR_PRESENT_DEFAULT
    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
    if (battery.healthy() && !battery.has_failsafed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }

    // update MAV_SYS_STATUS_SENSOR_3D_GYRO and MAV_SYS_STATUS_SENSOR_3D_ACCEL:
    const AP_InertialSensor &ins = AP::ins();
    // marked as present in MAVLINK_SENSOR_PRESENT_DEFAULT

    // sensor is not enabled until vehicle is initialised and
    // calibration is done.  This is to avoid spurious warnings at
    // vehicle boot-time.
    if (vehicle_initialised() && !ins.calibrating()) {
        control_sensors_enabled |= (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        if (ins.get_gyro_health_all() && ins.gyro_calibrated_ok_all()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        }
        if (ins.get_accel_health_all()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        }
    }

    // update MAV_SYS_STATUS_AHRS:
    AP_AHRS &ahrs = AP::ahrs();
    // marked as present in MAVLINK_SENSOR_*_DEFAULT
    // AHRS sensor is not enabled until it is initialised.  This is to
    // avoid spurious warnings at boot-time:
    if (ahrs.initialised()) {
        control_sensors_enabled |= MAV_SYS_STATUS_AHRS;
        if (ahrs.healthy()) {
            // AHRS subsystem is healthy
            control_sensors_health |= MAV_SYS_STATUS_AHRS;
        }
    }

    // update MAV_SYS_STATUS_SENSOR_VISION_POSITION:
    const AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom != nullptr) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        if (visual_odom->enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
            if (visual_odom->healthy()) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
            }
        }
    }

    // update MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS:
    // set motors outputs as enabled if safety switch is not disarmed
    // (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
        // FIXME: update this based on measured RPM on ESCs:
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
}

/*
  send a text message to all GCS
 */
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
    hal.util->vsnprintf((char *)text, sizeof(text)-1, fmt, arg_list);
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

void GCS::send_home() const
{
    FOR_EACH_ACTIVE_CHANNEL(send_home());
}

void GCS::send_ekf_origin() const
{
    FOR_EACH_ACTIVE_CHANNEL(send_ekf_origin());
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
    if (chan(c).alternative.handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    chan(c).alternative.handler = handler;
    return true;
}


#undef FOR_EACH_ACTIVE_CHANNEL
