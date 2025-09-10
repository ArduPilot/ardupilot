#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_ROS_ENABLED

#include "AP_Proximity/AP_Proximity_Backend.h"
#include "sensor_msgs/msg/LaserScan.h"

class AP_Proximity_ROS : public AP_Proximity_Backend {
public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;
    
    void handle_laser_scan(const sensor_msgs_msg_LaserScan& msg);
    void update() override;

    float distance_max() const override { return 40.0f; }
    float distance_min() const override { return 0.1f; }

private:
    AP_Proximity_Boundary_3D::Face _last_face;
    float _last_distance_m;
    float _last_angle_deg;
    bool _last_distance_valid;
    uint32_t _last_update_ms;
};

#endif // AP_PROXIMITY_ROS_ENABLED
