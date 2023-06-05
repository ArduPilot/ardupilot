#include "AP_Mount_Backend.h"
#include <AP_Mount/AP_Mount_Backend.h>

#if HAL_MOUNT_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>

// Write a Mount packet
void AP_Mount_Backend::Write_MountInfo(enum LogMessages msg, uint64_t timestamp_us)
{
    // exit immediately if no logger
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }

    // exit immediately if should not log Mount messages
    if (!logger->should_log(_frontend.get_log_mount_bit())) {
        return;
    }

    const auto nanf = AP::logger().quiet_nanf();
    float_t mount_current_roll = nanf;
    float_t mount_current_pitch = nanf;
    float_t mount_current_yaw = nanf;
    float_t mount_target_roll = nanf;
    float_t mount_target_pitch = nanf;
    float_t mount_target_yaw = nanf;
    auto *mount = AP_Mount::get_singleton();
    if (mount != nullptr) {
        mount->get_attitude_euler(mount_current_roll,mount_current_pitch,mount_current_yaw);
        mount_target_roll = this->mavt_target.angle_rad.roll;
        mount_target_pitch = this->mavt_target.angle_rad.pitch;
        mount_target_yaw = this->mavt_target.angle_rad.yaw;
    }

    const struct log_mount pkt{
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(msg)),
        time_us     : timestamp_us ? timestamp_us : AP_HAL::micros64(),
        instance    : _instance,
        roll        : (int16_t)mount_target_roll,
        pitch       : (int16_t)mount_target_pitch,
        yaw         : (uint16_t)mount_target_yaw,
        mount_roll        : (int16_t)mount_current_roll,
        mount_pitch       : (int16_t)mount_current_pitch,
        mount_yaw         : (uint16_t)mount_current_yaw
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write a Trigger packet
void AP_Mount_Backend::Write_Mount()
{
    Write_MountInfo(LOG_MOUNT_MSG, 0);
}

#endif