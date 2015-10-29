// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

void Rover::init_capabilities(void)
{
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
}
