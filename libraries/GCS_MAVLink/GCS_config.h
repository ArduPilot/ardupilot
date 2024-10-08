#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Relay/AP_Relay_config.h>
#include <AP_Mission/AP_Mission_config.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

#ifndef HAL_MAVLINK_BINDINGS_ENABLED
#define HAL_MAVLINK_BINDINGS_ENABLED HAL_GCS_ENABLED
#endif

// CODE_REMOVAL
// BATTERY2 is slated to be removed:
// ArduPilot 4.6 stops compiling support in
// ArduPilot 4.7 removes the code entirely
#ifndef AP_MAVLINK_BATTERY2_ENABLED
#define AP_MAVLINK_BATTERY2_ENABLED 0
#endif

#ifndef HAL_HIGH_LATENCY2_ENABLED
#define HAL_HIGH_LATENCY2_ENABLED 1
#endif

// handling of MISSION_SET_CURRENT (the message) is slated to be
// removed.  It has signficant deficiencies vs MAV_CMD_DO_SET_CURRENT.
// The command was added to the spec in January 2019 and to MAVLink in
// ArduPilot in 4.1.x
#ifndef AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
#define AP_MAVLINK_MISSION_SET_CURRENT_ENABLED AP_MISSION_ENABLED
#endif

// AUTOPILOT_VERSION_REQUEST is slated to be removed; an instance of
// AUTOPILOT_VERSION can be requested with MAV_CMD_REQUEST_MESSAGE,
// which gets you an ACK/NACK
#ifndef AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
#define AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED 1
#endif

#ifndef AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
#define AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED 1
#endif

// handling of MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES is slated to be
// removed; the message can be requested with MAV_CMD_REQUEST_MESSAGE
#ifndef AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
#define AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED 1
#endif

#ifndef HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
#define HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED ((AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED) && BOARD_FLASH_SIZE > 1024)
#endif

#ifndef AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
#define AP_MAVLINK_MSG_RELAY_STATUS_ENABLED HAL_GCS_ENABLED && AP_RELAY_ENABLED
#endif

// allow removal of developer-centric mavlink commands
#ifndef AP_MAVLINK_FAILURE_CREATION_ENABLED
#define AP_MAVLINK_FAILURE_CREATION_ENABLED 1
#endif

// CODE_REMOVAL
// ArduPilot 4.6 sends deprecation warnings for RALLY_POINT/RALLY_FETCH_POINT
// ArduPilot 4.7 stops compiling them in by default
// ArduPilot 4.8 removes the code entirely
#ifndef AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED
#define AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED HAL_GCS_ENABLED && HAL_RALLY_ENABLED
#endif

// CODE_REMOVAL
// handling of HIL_GPS is slated to be removed in 4.7; GPS_INPUT can be used
// in its place
// ArduPilot 4.6 stops compiling support in
// ArduPilot 4.7 removes the code entirely
#ifndef AP_MAVLINK_MSG_HIL_GPS_ENABLED
#define AP_MAVLINK_MSG_HIL_GPS_ENABLED 0
#endif

// CODE_REMOVAL
// ArduPilot 4.5 sends deprecation warnings for MOUNT_CONTROL/MOUNT_CONFIGURE
// ArduPilot 4.6 stops compiling them in
// ArduPilot 4.7 removes the code entirely
#ifndef AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED
#define AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED 0
#endif

#ifndef AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
#define AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED 0
#endif

// this is for both read and write messages:
#ifndef AP_MAVLINK_MSG_DEVICE_OP_ENABLED
#define AP_MAVLINK_MSG_DEVICE_OP_ENABLED HAL_GCS_ENABLED
#endif

#ifndef AP_MAVLINK_SERVO_RELAY_ENABLED
#define AP_MAVLINK_SERVO_RELAY_ENABLED HAL_GCS_ENABLED && AP_SERVORELAYEVENTS_ENABLED
#endif

#ifndef AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED
#define AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED HAL_GCS_ENABLED
#endif

#ifndef AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
#define AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED HAL_ADSB_ENABLED
#endif

#ifndef AP_MAVLINK_FTP_ENABLED
#define AP_MAVLINK_FTP_ENABLED HAL_GCS_ENABLED
#endif

// GCS should be using MISSION_REQUEST_INT instead; this is a waste of
// flash.  MISSION_REQUEST was deprecated in June 2020.  We started
// sending warnings to the GCS in Sep 2022 if this command was used.
// Copter 4.4.0 sends this warning.
#ifndef AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
#define AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED AP_MISSION_ENABLED
#endif

// all commands can be executed by COMMAND_INT, so COMMAND_LONG isn't
// strictly required.  This option created for 4.5, Nov 2023, and code
// left in place.
#ifndef AP_MAVLINK_COMMAND_LONG_ENABLED
#define AP_MAVLINK_COMMAND_LONG_ENABLED 1
#endif

#ifndef AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
#define AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED (BOARD_FLASH_SIZE > 1024) && AP_INERTIALSENSOR_ENABLED
#endif

#ifndef AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
#define AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#ifndef AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
#define AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED HAL_GCS_ENABLED
#endif
