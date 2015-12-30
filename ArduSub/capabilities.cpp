// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

void Copter::init_capabilities(void)
{
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED);
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT);
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET);
}
