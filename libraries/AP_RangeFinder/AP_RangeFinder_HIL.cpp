/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "../AP_HAL/AP_HAL.h"

#include "AP_RangeFinder_HIL.h"


extern const AP_HAL::HAL& hal;

AP_RangeFinder_HIL::AP_RangeFinder_HIL(RangeFinder &range, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(range, _instance, _state)
{

}


bool AP_RangeFinder_HIL::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}
