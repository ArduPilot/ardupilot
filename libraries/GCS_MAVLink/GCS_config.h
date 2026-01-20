#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Relay/AP_Relay_config.h>
#include <AP_Mission/AP_Mission_config.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>
#include <AP_Arming/AP_Arming_config.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

#ifndef HAL_MAVLINK_BINDINGS_ENABLED
#define HAL_MAVLINK_BINDINGS_ENABLED HAL_GCS_ENABLED
#endif

#ifndef AP_MAVLINK_SIGNING_ENABLED
#define AP_MAVLINK_SIGNING_ENABLED HAL_GCS_ENABLED
#endif  // AP_MAVLINK_SIGNING_ENABLED

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
#define HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED ((AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_LITTLEFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED) && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#ifndef AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
#define AP_MAVLINK_MSG_RELAY_STATUS_ENABLED HAL_GCS_ENABLED && AP_RELAY_ENABLED
#endif

#ifndef AP_MAVLINK_MSG_WIND_ENABLED
#define AP_MAVLINK_MSG_WIND_ENABLED AP_AHRS_ENABLED
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
#define AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED 0
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
// sending warnings to the GCS in Sep 2022 if MISSION_REQUEST was used.
// Copter 4.4.0 sends this warning.
// CODE_REMOVAL
// ArduPilot 4.4 sends warnings if MISSION_ITEM used
// ArduPilot 4.8 stops compiling in MISSION_ITEM but still sends warnings
// ArduPilot 4.9 removes the code but sends message about MISSION_ITEM not supported
// ArduPilot 4.10 stops sending the warning
#ifndef AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
#define AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED AP_MISSION_ENABLED
#endif

// RANGEFINDER is a subset of the DISTANCE_SENSOR message which we
// also send.  Rover's send-minimum can be done on the client-side.
// CODE_REMOVAL
// ArduPilot 4.7 stops sending the message by default
// ArduPilot 4.8 compiles the code out by default
// ArduPilot 4.9 removes the code entirely
#ifndef AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
#define AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED AP_RANGEFINDER_ENABLED
#endif

// all commands can be executed by COMMAND_INT, so COMMAND_LONG isn't
// strictly required.  This option created for 4.5, Nov 2023, and code
// left in place.
#ifndef AP_MAVLINK_COMMAND_LONG_ENABLED
#define AP_MAVLINK_COMMAND_LONG_ENABLED 1
#endif

#ifndef AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
#define AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024) && AP_INERTIALSENSOR_ENABLED
#endif

// ArduPilot 4.4 stopped sending this message by default
// ArduPilot 4.7 stops compiling support into the firmware
// ArduPilot 4.8 removes the code
#ifndef AP_MAVLINK_MSG_HWSTATUS_ENABLED
#define AP_MAVLINK_MSG_HWSTATUS_ENABLED 0
#endif

#ifndef AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
#define AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#ifndef AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
#define AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED HAL_GCS_ENABLED
#endif

#ifndef AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
#define AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED HAL_GCS_ENABLED && AP_ARMING_ENABLED
#endif  // AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED

// deprecated 2025-02, replaced by MAV_CMD_DO_SET_GLOBAL_ORIGIN
// ArduPilot 4.8 starts to warn if anyone uses this
// ArduPilot 4.9 continues to warn if anyone uses this
// ArduPilot 4.10 compiles support out
// ArduPilot 4.11 removes the code
#ifndef AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
#define AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED (HAL_GCS_ENABLED && AP_AHRS_ENABLED)
#endif  // AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
