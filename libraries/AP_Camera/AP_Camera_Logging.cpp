#include "AP_Camera_Backend.h"
#include <AP_Mount/AP_Mount.h>

#if AP_CAMERA_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>

// Write a Camera packet.  Also writes a Mount packet if available
void AP_Camera_Backend::Write_CameraInfo(enum LogMessages msg, uint64_t timestamp_us)
{
    // exit immediately if no logger
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }

    // exit immediately if should not log camera messages
    if (!logger->should_log(_frontend.get_log_camera_bit())) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    Location current_loc;
    if (!ahrs.get_location(current_loc)) {
        // completely ignore this failure!  AHRS will provide its best guess.
    }

    int32_t altitude, altitude_rel, altitude_gps;
    if (current_loc.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    // if timestamp is zero set to current system time
    if (timestamp_us == 0) {
        timestamp_us = AP_HAL::micros64();
    }

    const struct log_Camera pkt{
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : timestamp_us,
        instance    : _instance,
        image_number: image_index,
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : altitude,
        altitude_rel: altitude_rel,
        altitude_gps: altitude_gps,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));

#if HAL_MOUNT_ENABLED
    auto *mount = AP_Mount::get_singleton();
    if (mount!= nullptr) {
        mount->write_log(get_mount_instance(), timestamp_us);
    }
#endif
}

// Write a Camera packet
void AP_Camera_Backend::Write_Camera(uint64_t timestamp_us)
{
    Write_CameraInfo(LOG_CAMERA_MSG, timestamp_us);
}

// Write a Trigger packet
void AP_Camera_Backend::Write_Trigger()
{
    Write_CameraInfo(LOG_TRIGGER_MSG, 0);
}

#endif
