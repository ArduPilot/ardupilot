#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

// BATTERY2 is slated to be removed:
#ifndef AP_MAVLINK_BATTERY2_ENABLED
#define AP_MAVLINK_BATTERY2_ENABLED 1
#endif

// support for MISSION_ITEM is slated to be removed (in favour of MISSION_ITEM_INT):
#ifndef AP_MAVLINK_MISSION_ITEM_ENABLED
#define AP_MAVLINK_MISSION_ITEM_ENABLED 1
#endif
