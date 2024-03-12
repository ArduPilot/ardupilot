#include "AP_VisualOdom_Backend.h"
#include <AP_Logger/AP_Logger_config.h>

#if HAL_VISUALODOM_ENABLED && HAL_LOGGING_ENABLED

#include <AP_Logger/AP_Logger.h>

// Write visual odometry sensor data
void AP_VisualOdom_Backend::Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence)
{
    const struct log_VisualOdom pkt_visualodom {
        LOG_PACKET_HEADER_INIT(LOG_VISUALODOM_MSG),
        time_us             : AP_HAL::micros64(),
        time_delta          : time_delta,
        angle_delta_x       : angle_delta.x,
        angle_delta_y       : angle_delta.y,
        angle_delta_z       : angle_delta.z,
        position_delta_x    : position_delta.x,
        position_delta_y    : position_delta.y,
        position_delta_z    : position_delta.z,
        confidence          : confidence
    };
    AP::logger().WriteBlock(&pkt_visualodom, sizeof(log_VisualOdom));
}

// Write visual position sensor data.  x,y,z are in meters, angles are in degrees
void AP_VisualOdom_Backend::Write_VisualPosition(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float pos_err, float ang_err, uint8_t reset_counter, bool ignored, int8_t quality)
{
    const struct log_VisualPosition pkt_visualpos {
        LOG_PACKET_HEADER_INIT(LOG_VISUALPOS_MSG),
        time_us         : AP_HAL::micros64(),
        remote_time_us  : remote_time_us,
        time_ms         : time_ms,
        pos_x           : x,
        pos_y           : y,
        pos_z           : z,
        roll            : roll,
        pitch           : pitch,
        yaw             : yaw,
        pos_err         : pos_err,
        ang_err         : ang_err,
        reset_counter   : reset_counter,
        ignored         : (uint8_t)ignored,
        quality         : quality
    };
    AP::logger().WriteBlock(&pkt_visualpos, sizeof(log_VisualPosition));
}

// Write visual velocity sensor data, velocity in NED meters per second
void AP_VisualOdom_Backend::Write_VisualVelocity(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, bool ignored, int8_t quality)
{
    const struct log_VisualVelocity pkt_visualvel {
        LOG_PACKET_HEADER_INIT(LOG_VISUALVEL_MSG),
        time_us         : AP_HAL::micros64(),
        remote_time_us  : remote_time_us,
        time_ms         : time_ms,
        vel_x           : vel.x,
        vel_y           : vel.y,
        vel_z           : vel.z,
        vel_err         : _frontend.get_vel_noise(),
        reset_counter   : reset_counter,
        ignored         : (uint8_t)ignored,
        quality         : quality
    };
    AP::logger().WriteBlock(&pkt_visualvel, sizeof(log_VisualVelocity));
}

#endif // HAL_VISUALODOM_ENABLED
