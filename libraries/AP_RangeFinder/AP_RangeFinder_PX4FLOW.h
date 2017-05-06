#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_PX4FLOW : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_PX4FLOW(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);
};
