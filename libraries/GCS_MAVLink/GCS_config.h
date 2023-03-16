#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

// BATTERY2 is slated to be removed:
#ifndef AP_MAVLINK_BATTERY2_ENABLED
#define AP_MAVLINK_BATTERY2_ENABLED 1
#endif

// handling of MISSION_SET_CURRENT (the message) is slated to be
// removed.  It has signficant deficiencies vs MAV_CMD_DO_SET_CURRENT.
// The command was added to the spec in January 2019 and to MAVLink in
// ArduPilot in 4.1.x
#ifndef AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
#define AP_MAVLINK_MISSION_SET_CURRENT_ENABLED 1
#endif
