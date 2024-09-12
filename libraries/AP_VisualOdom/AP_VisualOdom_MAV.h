#pragma once

#include "AP_VisualOdom_config.h"

#if AP_VISUALODOM_MAV_ENABLED

#include "AP_VisualOdom_Backend.h"

class AP_VisualOdom_MAV : public AP_VisualOdom_Backend
{

public:
    // constructor
    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    // consume vision pose estimate data and send to EKF. distances in meters
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality) override;

    // consume vision velocity estimate data and send to EKF, velocity in NED meters per second
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality) override;
};

#endif  // AP_VISUALODOM_MAV_ENABLED
