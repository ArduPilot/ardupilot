/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_analog : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_analog(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance);

    // update state
    void update(void);

private:
    // update raw voltage
    void update_voltage(void);

    AP_HAL::AnalogSource *source;
};
