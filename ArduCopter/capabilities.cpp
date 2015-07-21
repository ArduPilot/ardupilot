// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

uint64_t Copter::get_capabilities(void)
{
    uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT
                 | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
                 | MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED
                 | MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;

    #if AP_TERRAIN_AVAILABLE
        capabilities |= MAV_PROTOCOL_CAPABILITY_TERRAIN;
    #endif

    return capabilities;
}
