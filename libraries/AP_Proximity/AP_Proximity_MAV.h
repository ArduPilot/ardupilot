#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MAV_ENABLED

#include "AP_Proximity_Backend.h"

class AP_Proximity_MAV : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return _distance_max; }
    float distance_min() const override { return _distance_min; };

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

    // handle mavlink messages
    void handle_msg(const mavlink_message_t &msg) override;

private:

    // handle mavlink DISTANCE_SENSOR messages
    void handle_distance_sensor_msg(const mavlink_message_t &msg);
    // handle mavlink OBSTACLE_DISTANCE messages
    void handle_obstacle_distance_msg(const mavlink_message_t &msg);
    // handle mavlink OBSTACLE_DISTANCE_3D messages
    void handle_obstacle_distance_3d_msg(const mavlink_message_t &msg);

   AP_Proximity_Temp_Boundary temp_boundary;

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last mavlink message received
    uint32_t _last_msg_update_timestamp_ms;   // last stored mavlink message timestamp
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update of upward distance
    float _distance_upward;             // upward distance in meters
};

#endif // AP_PROXIMITY_MAV_ENABLED
