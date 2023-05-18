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

// AUTOPILOT_VERSION_REQUEST is slated to be removed; an instance of
// AUTOPILOT_VERSION can be requested with MAV_CMD_REQUEST_MESSAGE,
// which gets you an ACK/NACK
#ifndef AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
#define AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED 1
#endif

// handling of MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES is slated to be
// removed; the message can be requested with MAV_CMD_REQUEST_MESSAGE
#ifndef AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
#define AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED 1
#endif

#ifndef HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
#define HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED ((AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED) && BOARD_FLASH_SIZE > 1024)
#endif

