// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.cpp
/// @brief	Supporting bits for MAVLink.

#include "GCS_MAVLink.h"

BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

// this might need to move to the flight software
mavlink_system_t mavlink_system = {7,1,0,0};

#include "include/mavlink_helpers.h"
