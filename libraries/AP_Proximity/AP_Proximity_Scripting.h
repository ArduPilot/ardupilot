#pragma once

#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED

class AP_Proximity_Scripting : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max_m() const override { return _distance_max; }
    float distance_min_m() const override { return _distance_min; }

    // Set max and min range of the sensor. only needs to be set once
    bool set_distance_min_max(float min, float max) override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

    // handle script messages
    bool handle_script_distance_msg(float dist_m, float yaw_deg, float pitch_deg, bool push_to_boundary) override;
    bool handle_script_3d_msg(const Vector3f &vec_to_obstacle, bool push_to_boundary) override;

    // update the temporary (buffer) boundary
    bool update_virtual_boundary() override;

private:

    // temp boundary to store and sort distances
    AP_Proximity_Temp_Boundary temp_boundary;

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last script message received

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update of upward distance
    float _distance_upward;             // upward distance in meters

    // min and max distance of sensor
    float _distance_min;
    float _distance_max;
};

#endif // HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED
