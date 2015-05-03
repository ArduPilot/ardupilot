// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_PX4_FLOW_H__
#define __AP_RANGEFINDER_PX4_FLOW_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_PX4_Flow : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_PX4_Flow(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

};
#endif  // __AP_RANGEFINDER_PX4_FLOW_H__
