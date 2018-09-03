#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_FixedRange : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_FixedRange(RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
};
