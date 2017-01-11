#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_MAV_TIMEOUT_MS    200 // requests timeout after 0.2 seconds

class AP_Proximity_MAV : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_MAV(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void);

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const { return _distance_max; }
    float distance_min() const { return _distance_min; };

    // handle mavlink DISTANCE_SENSOR messages
    void handle_msg(mavlink_message_t *msg) override;

private:

    // initialise sensor (returns true if sensor is succesfully initialised)
    bool initialise();

    uint32_t _last_update_ms;   // system time of last DISTANCE_SENSOR message received
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters
};
