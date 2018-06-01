// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

void Tracker::init_capabilities(void)
{
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
}
