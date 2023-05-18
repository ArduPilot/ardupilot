#pragma once

#include "AP_VisualOdom_Backend.h"

#if HAL_VISUALODOM_ENABLED

class AP_VisualOdom_MAV : public AP_VisualOdom_Backend
{

public:
    // constructor
    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    // consume vision position estimate data and send to EKF. distances in meters
    void handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter) override;

    // consume vision velocity estimate data and send to EKF, velocity in NED meters per second
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter) override;
};

#endif
