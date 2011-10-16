// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.cpp
/// @brief	Supporting bits for MAVLink.

#include "GCS_MAVLink.h"

BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

// this might need to move to the flight software
mavlink_system_t mavlink_system = {7,1,0,0};

#ifdef MAVLINK10
# include "include_v1.0/mavlink_helpers.h"
#else
# include "include/mavlink_helpers.h"
#endif

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (sysid != mavlink_system.sysid)
        return 1;
    // Currently we are not checking for correct compid since APM is not passing mavlink info to any subsystem
    // If it is addressed to our system ID we assume it is for us
    return 0; // no error
}
