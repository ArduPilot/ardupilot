#include "AP_Camera.h"
#include <AP_Logger/AP_Logger.h>

// Write a Camera packet
void AP_Camera::Write_CameraInfo(enum LogMessages msg, uint64_t timestamp_us)
{
    const AP_AHRS &ahrs = AP::ahrs();
    update_offset_location();
    int32_t altitude, altitude_rel, altitude_gps;
    if (offset_location.relative_alt) {
        altitude = offset_location.alt+ahrs.get_home().alt;
        altitude_rel = offset_location.alt;
    } else {
        altitude = offset_location.alt;
        altitude_rel = offset_location.alt - ahrs.get_home().alt;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    const struct log_Camera pkt{
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : timestamp_us?timestamp_us:AP_HAL::micros64(),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : offset_location.lat,
        longitude   : offset_location.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        altitude_gps: altitude_gps,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Camera::Write_Camera(uint64_t timestamp_us)
{
    Write_CameraInfo(LOG_CAMERA_MSG, timestamp_us);
}

// Write a Trigger packet
void AP_Camera::Write_Trigger(void)
{
    Write_CameraInfo(LOG_TRIGGER_MSG, 0);
}