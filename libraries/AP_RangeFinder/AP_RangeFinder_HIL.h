/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  dummy backend for HIL (and SITL). This doesn't actually need to do
  any work, as setHIL() is in the frontend
 */

#ifndef __AP_RANGEFINDER_HIL_H__
#define __AP_RANGEFINDER_HIL_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_HIL : public AP_RangeFinder_Backend
{
public:
    AP_RangeFinder_HIL(RangeFinder &range, uint8_t instance, RangeFinder::RangeFinder_State &state);
    void update() {}
    static bool detect(RangeFinder &_ranger, uint8_t instance);

private:
    uint8_t _instance;
};

#endif //  __AP_RANGEFINDER_HIL_H__