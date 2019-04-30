#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// Data timeout
#define AP_RANGEFINDER_MAVLINK_TIMEOUT_MS 500

class AP_RangeFinder_MAVLink : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_MAVLink(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

    // Get update from mavlink
    void handle_msg(const mavlink_message_t &msg) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return sensor_type;
    }

private:
    uint16_t distance_cm;

    // start a reading
    static bool start_reading(void);
    static bool get_reading(uint16_t &reading_cm);

    MAV_DISTANCE_SENSOR sensor_type = MAV_DISTANCE_SENSOR_UNKNOWN;
};
