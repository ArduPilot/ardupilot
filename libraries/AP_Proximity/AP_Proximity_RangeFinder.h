#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_RANGEFINDER_ENABLED

#include "AP_Proximity_Backend.h"

#define PROXIMITY_RANGEFIDER_TIMEOUT_MS 200 // requests timeout after 0.2 seconds

class AP_Proximity_RangeFinder : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return _distance_max; }
    float distance_min() const override { return _distance_min; }

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last RangeFinder reading
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update distance
    float _distance_upward = -1;        // upward distance in meters, negative if the last reading was out of range
};

#endif // AP_PROXIMITY_RANGEFINDER_ENABLED
