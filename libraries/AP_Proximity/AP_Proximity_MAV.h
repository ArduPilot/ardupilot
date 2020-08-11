#pragma once

#include "AP_Proximity.h"
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

    // handle mavlink DISTANCE_SENSOR messages
    void handle_msg(const mavlink_message_t &msg) override;

protected:

    // we fill the obstacle distance data in as we get it from the
    // sensor, but if we're not getting data may need to fill in
    // invalid values.
    bool update_obstacle_distance_data() override;

private:
    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last DISTANCE_SENSOR message received
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update distance
    float _distance_upward;             // upward distance in meters
};
