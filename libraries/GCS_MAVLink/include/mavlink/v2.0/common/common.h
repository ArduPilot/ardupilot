/** @file
 *  @brief MAVLink comm protocol generated from common.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_COMMON_H
#define MAVLINK_COMMON_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_COMMON.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_COMMON_XML_HASH 1002008546201500244

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {1, 124, 31, 31, 0, 0, 0}, {2, 137, 12, 12, 0, 0, 0}, {4, 237, 14, 14, 3, 12, 13}, {5, 217, 28, 28, 1, 0, 0}, {6, 104, 3, 3, 0, 0, 0}, {7, 119, 32, 32, 0, 0, 0}, {11, 89, 6, 6, 1, 4, 0}, {20, 214, 20, 20, 3, 2, 3}, {21, 159, 2, 2, 3, 0, 1}, {22, 220, 25, 25, 0, 0, 0}, {23, 168, 23, 23, 3, 4, 5}, {24, 24, 30, 52, 0, 0, 0}, {25, 23, 101, 101, 0, 0, 0}, {26, 170, 22, 24, 0, 0, 0}, {27, 144, 26, 29, 0, 0, 0}, {28, 67, 16, 16, 0, 0, 0}, {29, 115, 14, 16, 0, 0, 0}, {30, 39, 28, 28, 0, 0, 0}, {31, 246, 32, 48, 0, 0, 0}, {32, 185, 28, 28, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {34, 237, 22, 22, 0, 0, 0}, {35, 244, 22, 22, 0, 0, 0}, {36, 222, 21, 37, 0, 0, 0}, {37, 212, 6, 7, 3, 4, 5}, {38, 9, 6, 7, 3, 4, 5}, {39, 254, 37, 38, 3, 32, 33}, {40, 230, 4, 5, 3, 2, 3}, {41, 28, 4, 4, 3, 2, 3}, {42, 28, 2, 6, 0, 0, 0}, {43, 132, 2, 3, 3, 0, 1}, {44, 221, 4, 5, 3, 2, 3}, {45, 232, 2, 3, 3, 0, 1}, {46, 11, 2, 2, 0, 0, 0}, {47, 153, 3, 4, 3, 0, 1}, {48, 41, 13, 21, 1, 12, 0}, {49, 39, 12, 20, 0, 0, 0}, {50, 78, 37, 37, 3, 18, 19}, {51, 196, 4, 5, 3, 2, 3}, {54, 15, 27, 27, 3, 24, 25}, {55, 3, 25, 25, 0, 0, 0}, {61, 167, 72, 72, 0, 0, 0}, {62, 183, 26, 26, 0, 0, 0}, {63, 119, 181, 181, 0, 0, 0}, {64, 191, 225, 225, 0, 0, 0}, {65, 118, 42, 42, 0, 0, 0}, {66, 148, 6, 6, 3, 2, 3}, {67, 21, 4, 4, 0, 0, 0}, {69, 243, 11, 30, 1, 10, 0}, {70, 124, 18, 38, 3, 16, 17}, {73, 38, 37, 38, 3, 32, 33}, {74, 20, 20, 20, 0, 0, 0}, {75, 158, 35, 35, 3, 30, 31}, {76, 152, 33, 33, 3, 30, 31}, {77, 143, 3, 10, 3, 8, 9}, {81, 106, 22, 22, 0, 0, 0}, {82, 49, 39, 39, 3, 36, 37}, {83, 22, 37, 37, 0, 0, 0}, {84, 143, 53, 53, 3, 50, 51}, {85, 140, 51, 51, 0, 0, 0}, {86, 5, 53, 53, 3, 50, 51}, {87, 150, 51, 51, 0, 0, 0}, {89, 231, 28, 28, 0, 0, 0}, {90, 183, 56, 56, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {92, 54, 33, 33, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {100, 175, 26, 34, 0, 0, 0}, {101, 102, 32, 117, 0, 0, 0}, {102, 158, 32, 117, 0, 0, 0}, {103, 208, 20, 57, 0, 0, 0}, {104, 56, 32, 116, 0, 0, 0}, {105, 93, 62, 63, 0, 0, 0}, {106, 138, 44, 44, 0, 0, 0}, {107, 108, 64, 65, 0, 0, 0}, {108, 32, 84, 92, 0, 0, 0}, {109, 185, 9, 9, 0, 0, 0}, {110, 84, 254, 254, 3, 1, 2}, {111, 34, 16, 16, 0, 0, 0}, {112, 174, 12, 12, 0, 0, 0}, {113, 124, 36, 39, 0, 0, 0}, {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {116, 76, 22, 24, 0, 0, 0}, {117, 128, 6, 6, 3, 4, 5}, {118, 56, 14, 14, 0, 0, 0}, {119, 116, 12, 12, 3, 10, 11}, {120, 134, 97, 97, 0, 0, 0}, {121, 237, 2, 2, 3, 0, 1}, {122, 203, 2, 2, 3, 0, 1}, {123, 250, 113, 113, 3, 0, 1}, {124, 87, 35, 57, 0, 0, 0}, {125, 203, 6, 6, 0, 0, 0}, {126, 220, 79, 79, 0, 0, 0}, {127, 25, 35, 35, 0, 0, 0}, {128, 226, 35, 35, 0, 0, 0}, {129, 46, 22, 24, 0, 0, 0}, {130, 29, 13, 13, 0, 0, 0}, {131, 223, 255, 255, 0, 0, 0}, {132, 85, 14, 39, 0, 0, 0}, {133, 6, 18, 18, 0, 0, 0}, {134, 229, 43, 43, 0, 0, 0}, {135, 203, 8, 8, 0, 0, 0}, {136, 1, 22, 22, 0, 0, 0}, {137, 195, 14, 16, 0, 0, 0}, {138, 109, 36, 120, 0, 0, 0}, {139, 168, 43, 43, 3, 41, 42}, {140, 181, 41, 41, 0, 0, 0}, {141, 47, 32, 32, 0, 0, 0}, {142, 72, 243, 243, 0, 0, 0}, {143, 131, 14, 16, 0, 0, 0}, {144, 127, 93, 93, 0, 0, 0}, {146, 103, 100, 100, 0, 0, 0}, {147, 154, 36, 54, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {149, 200, 30, 60, 0, 0, 0}, {162, 189, 8, 9, 0, 0, 0}, {192, 36, 44, 54, 0, 0, 0}, {225, 208, 65, 73, 0, 0, 0}, {230, 163, 42, 42, 0, 0, 0}, {231, 105, 40, 40, 0, 0, 0}, {232, 151, 63, 65, 0, 0, 0}, {233, 35, 182, 182, 0, 0, 0}, {234, 150, 40, 40, 0, 0, 0}, {235, 179, 42, 42, 0, 0, 0}, {241, 90, 32, 32, 0, 0, 0}, {242, 104, 52, 60, 0, 0, 0}, {243, 85, 53, 61, 1, 52, 0}, {244, 95, 6, 6, 0, 0, 0}, {245, 130, 2, 2, 0, 0, 0}, {246, 184, 38, 38, 0, 0, 0}, {247, 81, 19, 19, 0, 0, 0}, {248, 8, 254, 254, 3, 3, 4}, {249, 204, 36, 36, 0, 0, 0}, {250, 49, 30, 30, 0, 0, 0}, {251, 170, 18, 18, 0, 0, 0}, {252, 44, 18, 18, 0, 0, 0}, {253, 83, 51, 54, 0, 0, 0}, {254, 46, 9, 9, 0, 0, 0}, {256, 71, 42, 42, 3, 8, 9}, {257, 131, 9, 9, 0, 0, 0}, {258, 187, 32, 232, 3, 0, 1}, {259, 92, 235, 236, 0, 0, 0}, {260, 146, 5, 13, 0, 0, 0}, {261, 179, 27, 60, 0, 0, 0}, {262, 12, 18, 22, 0, 0, 0}, {263, 133, 255, 255, 0, 0, 0}, {264, 49, 28, 28, 0, 0, 0}, {265, 26, 16, 20, 0, 0, 0}, {266, 193, 255, 255, 3, 2, 3}, {267, 35, 255, 255, 3, 2, 3}, {268, 14, 4, 4, 3, 2, 3}, {269, 109, 213, 214, 0, 0, 0}, {270, 59, 19, 19, 0, 0, 0}, {271, 22, 52, 52, 0, 0, 0}, {275, 126, 31, 31, 0, 0, 0}, {276, 18, 49, 49, 0, 0, 0}, {277, 62, 30, 30, 0, 0, 0}, {280, 70, 33, 33, 0, 0, 0}, {281, 48, 13, 13, 0, 0, 0}, {282, 123, 35, 35, 3, 32, 33}, {283, 74, 144, 145, 0, 0, 0}, {284, 99, 32, 32, 3, 30, 31}, {285, 137, 40, 49, 3, 38, 39}, {286, 210, 53, 57, 3, 50, 51}, {287, 1, 23, 23, 3, 20, 21}, {288, 20, 23, 23, 3, 20, 21}, {299, 19, 96, 96, 0, 0, 0}, {301, 243, 58, 58, 0, 0, 0}, {310, 28, 17, 17, 0, 0, 0}, {311, 95, 116, 116, 0, 0, 0}, {320, 243, 20, 20, 3, 2, 3}, {321, 88, 2, 2, 3, 0, 1}, {322, 243, 149, 149, 0, 0, 0}, {323, 78, 147, 147, 3, 0, 1}, {324, 132, 146, 146, 0, 0, 0}, {330, 23, 158, 167, 0, 0, 0}, {331, 91, 230, 233, 0, 0, 0}, {332, 236, 239, 239, 0, 0, 0}, {333, 231, 109, 109, 0, 0, 0}, {335, 225, 24, 24, 0, 0, 0}, {339, 199, 5, 5, 0, 0, 0}, {340, 99, 70, 70, 0, 0, 0}, {350, 232, 20, 252, 0, 0, 0}, {370, 75, 87, 109, 0, 0, 0}, {373, 117, 42, 42, 0, 0, 0}, {375, 251, 140, 140, 0, 0, 0}, {376, 199, 8, 8, 0, 0, 0}, {385, 147, 133, 133, 3, 2, 3}, {386, 132, 16, 16, 3, 4, 5}, {387, 4, 72, 72, 3, 4, 5}, {388, 8, 37, 37, 3, 32, 33}, {9000, 113, 137, 137, 0, 0, 0}, {9005, 117, 34, 34, 0, 0, 0}, {12900, 114, 44, 44, 3, 0, 1}, {12901, 254, 59, 59, 3, 30, 31}, {12902, 140, 53, 53, 3, 4, 5}, {12903, 249, 46, 46, 3, 0, 1}, {12904, 77, 54, 54, 3, 28, 29}, {12905, 49, 43, 43, 3, 0, 1}, {12915, 94, 249, 249, 3, 0, 1}, {12918, 139, 51, 51, 0, 0, 0}, {12919, 7, 18, 18, 3, 16, 17}, {12920, 20, 5, 5, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_COMMON

// ENUM DEFINITIONS


/** @brief These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. */
#ifndef HAVE_ENUM_FIRMWARE_VERSION_TYPE
#define HAVE_ENUM_FIRMWARE_VERSION_TYPE
typedef enum FIRMWARE_VERSION_TYPE
{
   FIRMWARE_VERSION_TYPE_DEV=0, /* development release | */
   FIRMWARE_VERSION_TYPE_ALPHA=64, /* alpha release | */
   FIRMWARE_VERSION_TYPE_BETA=128, /* beta release | */
   FIRMWARE_VERSION_TYPE_RC=192, /* release candidate | */
   FIRMWARE_VERSION_TYPE_OFFICIAL=255, /* official stable release | */
   FIRMWARE_VERSION_TYPE_ENUM_END=256, /*  | */
} FIRMWARE_VERSION_TYPE;
#endif

/** @brief Flags to report failure cases over the high latency telemetry. */
#ifndef HAVE_ENUM_HL_FAILURE_FLAG
#define HAVE_ENUM_HL_FAILURE_FLAG
typedef enum HL_FAILURE_FLAG
{
   HL_FAILURE_FLAG_GPS=1, /* GPS failure. | */
   HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE=2, /* Differential pressure sensor failure. | */
   HL_FAILURE_FLAG_ABSOLUTE_PRESSURE=4, /* Absolute pressure sensor failure. | */
   HL_FAILURE_FLAG_3D_ACCEL=8, /* Accelerometer sensor failure. | */
   HL_FAILURE_FLAG_3D_GYRO=16, /* Gyroscope sensor failure. | */
   HL_FAILURE_FLAG_3D_MAG=32, /* Magnetometer sensor failure. | */
   HL_FAILURE_FLAG_TERRAIN=64, /* Terrain subsystem failure. | */
   HL_FAILURE_FLAG_BATTERY=128, /* Battery failure/critical low battery. | */
   HL_FAILURE_FLAG_RC_RECEIVER=256, /* RC receiver failure/no RC connection. | */
   HL_FAILURE_FLAG_OFFBOARD_LINK=512, /* Offboard link failure. | */
   HL_FAILURE_FLAG_ENGINE=1024, /* Engine failure. | */
   HL_FAILURE_FLAG_GEOFENCE=2048, /* Geofence violation. | */
   HL_FAILURE_FLAG_ESTIMATOR=4096, /* Estimator failure, for example measurement rejection or large variances. | */
   HL_FAILURE_FLAG_MISSION=8192, /* Mission failure. | */
   HL_FAILURE_FLAG_ENUM_END=8193, /*  | */
} HL_FAILURE_FLAG;
#endif

/** @brief Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution. */
#ifndef HAVE_ENUM_MAV_GOTO
#define HAVE_ENUM_MAV_GOTO
typedef enum MAV_GOTO
{
   MAV_GOTO_DO_HOLD=0, /* Hold at the current position. | */
   MAV_GOTO_DO_CONTINUE=1, /* Continue with the next item in mission execution. | */
   MAV_GOTO_HOLD_AT_CURRENT_POSITION=2, /* Hold at the current position of the system | */
   MAV_GOTO_HOLD_AT_SPECIFIED_POSITION=3, /* Hold at the position specified in the parameters of the DO_HOLD action | */
   MAV_GOTO_ENUM_END=4, /*  | */
} MAV_GOTO;
#endif

/** @brief These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
#ifndef HAVE_ENUM_MAV_MODE
#define HAVE_ENUM_MAV_MODE
typedef enum MAV_MODE
{
   MAV_MODE_PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
   MAV_MODE_MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_ENUM_END=221, /*  | */
} MAV_MODE;
#endif

/** @brief These encode the sensors whose status is sent as part of the SYS_STATUS message. */
#ifndef HAVE_ENUM_MAV_SYS_STATUS_SENSOR
#define HAVE_ENUM_MAV_SYS_STATUS_SENSOR
typedef enum MAV_SYS_STATUS_SENSOR
{
   MAV_SYS_STATUS_SENSOR_3D_GYRO=1, /* 0x01 3D gyro | */
   MAV_SYS_STATUS_SENSOR_3D_ACCEL=2, /* 0x02 3D accelerometer | */
   MAV_SYS_STATUS_SENSOR_3D_MAG=4, /* 0x04 3D magnetometer | */
   MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE=8, /* 0x08 absolute pressure | */
   MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE=16, /* 0x10 differential pressure | */
   MAV_SYS_STATUS_SENSOR_GPS=32, /* 0x20 GPS | */
   MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW=64, /* 0x40 optical flow | */
   MAV_SYS_STATUS_SENSOR_VISION_POSITION=128, /* 0x80 computer vision position | */
   MAV_SYS_STATUS_SENSOR_LASER_POSITION=256, /* 0x100 laser based position | */
   MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH=512, /* 0x200 external ground truth (Vicon or Leica) | */
   MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL=1024, /* 0x400 3D angular rate control | */
   MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION=2048, /* 0x800 attitude stabilization | */
   MAV_SYS_STATUS_SENSOR_YAW_POSITION=4096, /* 0x1000 yaw position | */
   MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL=8192, /* 0x2000 z/altitude control | */
   MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL=16384, /* 0x4000 x/y position control | */
   MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS=32768, /* 0x8000 motor outputs / control | */
   MAV_SYS_STATUS_SENSOR_RC_RECEIVER=65536, /* 0x10000 RC receiver | */
   MAV_SYS_STATUS_SENSOR_3D_GYRO2=131072, /* 0x20000 2nd 3D gyro | */
   MAV_SYS_STATUS_SENSOR_3D_ACCEL2=262144, /* 0x40000 2nd 3D accelerometer | */
   MAV_SYS_STATUS_SENSOR_3D_MAG2=524288, /* 0x80000 2nd 3D magnetometer | */
   MAV_SYS_STATUS_GEOFENCE=1048576, /* 0x100000 geofence | */
   MAV_SYS_STATUS_AHRS=2097152, /* 0x200000 AHRS subsystem health | */
   MAV_SYS_STATUS_TERRAIN=4194304, /* 0x400000 Terrain subsystem health | */
   MAV_SYS_STATUS_REVERSE_MOTOR=8388608, /* 0x800000 Motors are reversed | */
   MAV_SYS_STATUS_LOGGING=16777216, /* 0x1000000 Logging | */
   MAV_SYS_STATUS_SENSOR_BATTERY=33554432, /* 0x2000000 Battery | */
   MAV_SYS_STATUS_SENSOR_PROXIMITY=67108864, /* 0x4000000 Proximity | */
   MAV_SYS_STATUS_SENSOR_SATCOM=134217728, /* 0x8000000 Satellite Communication  | */
   MAV_SYS_STATUS_PREARM_CHECK=268435456, /* 0x10000000 pre-arm check status. Always healthy when armed | */
   MAV_SYS_STATUS_OBSTACLE_AVOIDANCE=536870912, /* 0x20000000 Avoidance/collision prevention | */
   MAV_SYS_STATUS_SENSOR_PROPULSION=1073741824, /* 0x40000000 propulsion (actuator, esc, motor or propellor) | */
   MAV_SYS_STATUS_SENSOR_ENUM_END=1073741825, /*  | */
} MAV_SYS_STATUS_SENSOR;
#endif

/** @brief Coordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles.

      Global frames use the following naming conventions:
      - "GLOBAL": Global coordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default.
        The following modifiers may be used with "GLOBAL":
        - "RELATIVE_ALT": Altitude is relative to the vehicle home position rather than MSL.
        - "TERRAIN_ALT": Altitude is relative to ground level rather than MSL.
        - "INT": Latitude/longitude (in degrees) are scaled by multiplying by 1E7.

      Local frames use the following naming conventions:
      - "LOCAL": Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF").
      - "BODY": Origin of local frame travels with the vehicle. NOTE, "BODY" does NOT indicate alignment of frame axis with vehicle attitude.
      - "OFFSET": Deprecated synonym for "BODY" (origin travels with the vehicle). Not to be used for new frames.

      Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).
  */
#ifndef HAVE_ENUM_MAV_FRAME
#define HAVE_ENUM_MAV_FRAME
typedef enum MAV_FRAME
{
   MAV_FRAME_GLOBAL=0, /* Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL). | */
   MAV_FRAME_LOCAL_NED=1, /* NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth. | */
   MAV_FRAME_MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
   MAV_FRAME_GLOBAL_RELATIVE_ALT=3, /* 
          Global (WGS84) coordinate frame + altitude relative to the home position.
         | */
   MAV_FRAME_LOCAL_ENU=4, /* ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth. | */
   MAV_FRAME_GLOBAL_INT=5, /* Global (WGS84) coordinate frame (scaled) + altitude relative to mean sea level (MSL). | */
   MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6, /* Global (WGS84) coordinate frame (scaled) + altitude relative to the home position.  | */
   MAV_FRAME_LOCAL_OFFSET_NED=7, /* NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle. | */
   MAV_FRAME_BODY_NED=8, /* Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/acceleration values. | */
   MAV_FRAME_BODY_OFFSET_NED=9, /* This is the same as MAV_FRAME_BODY_FRD. | */
   MAV_FRAME_GLOBAL_TERRAIN_ALT=10, /* Global (WGS84) coordinate frame with AGL altitude (altitude at ground level). | */
   MAV_FRAME_GLOBAL_TERRAIN_ALT_INT=11, /* Global (WGS84) coordinate frame (scaled) with AGL altitude (altitude at ground level). | */
   MAV_FRAME_BODY_FRD=12, /* FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle. | */
   MAV_FRAME_RESERVED_13=13, /* MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up). | */
   MAV_FRAME_RESERVED_14=14, /* MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down). | */
   MAV_FRAME_RESERVED_15=15, /* MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up). | */
   MAV_FRAME_RESERVED_16=16, /* MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down). | */
   MAV_FRAME_RESERVED_17=17, /* MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up). | */
   MAV_FRAME_RESERVED_18=18, /* MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down). | */
   MAV_FRAME_RESERVED_19=19, /* MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up). | */
   MAV_FRAME_LOCAL_FRD=20, /* FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane. | */
   MAV_FRAME_LOCAL_FLU=21, /* FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane. | */
   MAV_FRAME_ENUM_END=22, /*  | */
} MAV_FRAME;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAVLINK_DATA_STREAM_TYPE
#define HAVE_ENUM_MAVLINK_DATA_STREAM_TYPE
typedef enum MAVLINK_DATA_STREAM_TYPE
{
   MAVLINK_DATA_STREAM_IMG_JPEG=0, /*  | */
   MAVLINK_DATA_STREAM_IMG_BMP=1, /*  | */
   MAVLINK_DATA_STREAM_IMG_RAW8U=2, /*  | */
   MAVLINK_DATA_STREAM_IMG_RAW32U=3, /*  | */
   MAVLINK_DATA_STREAM_IMG_PGM=4, /*  | */
   MAVLINK_DATA_STREAM_IMG_PNG=5, /*  | */
   MAVLINK_DATA_STREAM_TYPE_ENUM_END=6, /*  | */
} MAVLINK_DATA_STREAM_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_FENCE_BREACH
#define HAVE_ENUM_FENCE_BREACH
typedef enum FENCE_BREACH
{
   FENCE_BREACH_NONE=0, /* No last fence breach | */
   FENCE_BREACH_MINALT=1, /* Breached minimum altitude | */
   FENCE_BREACH_MAXALT=2, /* Breached maximum altitude | */
   FENCE_BREACH_BOUNDARY=3, /* Breached fence boundary | */
   FENCE_BREACH_ENUM_END=4, /*  | */
} FENCE_BREACH;
#endif

/** @brief Actions being taken to mitigate/prevent fence breach */
#ifndef HAVE_ENUM_FENCE_MITIGATE
#define HAVE_ENUM_FENCE_MITIGATE
typedef enum FENCE_MITIGATE
{
   FENCE_MITIGATE_UNKNOWN=0, /* Unknown | */
   FENCE_MITIGATE_NONE=1, /* No actions being taken | */
   FENCE_MITIGATE_VEL_LIMIT=2, /* Velocity limiting active to prevent breach | */
   FENCE_MITIGATE_ENUM_END=3, /*  | */
} FENCE_MITIGATE;
#endif

/** @brief Fence types to enable or disable as a bitmask. Used in MAV_CMD_DO_FENCE_ENABLE. */
#ifndef HAVE_ENUM_FENCE_TYPE
#define HAVE_ENUM_FENCE_TYPE
typedef enum FENCE_TYPE
{
   FENCE_TYPE_ALT_MAX=1, /* Maximum altitude fence | */
   FENCE_TYPE_CIRCLE=2, /* Circle fence | */
   FENCE_TYPE_POLYGON=4, /* Polygon fence | */
   FENCE_TYPE_ALT_MIN=8, /* Minimum altitude fence | */
   FENCE_TYPE_ENUM_END=9, /*  | */
} FENCE_TYPE;
#endif

/** @brief Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages. */
#ifndef HAVE_ENUM_MAV_MOUNT_MODE
#define HAVE_ENUM_MAV_MOUNT_MODE
typedef enum MAV_MOUNT_MODE
{
   MAV_MOUNT_MODE_RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from permanent memory and stop stabilization | */
   MAV_MOUNT_MODE_NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
   MAV_MOUNT_MODE_MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
   MAV_MOUNT_MODE_RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
   MAV_MOUNT_MODE_GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
   MAV_MOUNT_MODE_SYSID_TARGET=5, /* Gimbal tracks system with specified system ID | */
   MAV_MOUNT_MODE_HOME_LOCATION=6, /* Gimbal tracks home position | */
   MAV_MOUNT_MODE_ENUM_END=7, /*  | */
} MAV_MOUNT_MODE;
#endif

/** @brief Gimbal device (low level) capability flags (bitmap). */
#ifndef HAVE_ENUM_GIMBAL_DEVICE_CAP_FLAGS
#define HAVE_ENUM_GIMBAL_DEVICE_CAP_FLAGS
typedef enum GIMBAL_DEVICE_CAP_FLAGS
{
   GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT=1, /* Gimbal device supports a retracted position. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL=2, /* Gimbal device supports a horizontal, forward looking position, stabilized. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS=4, /* Gimbal device supports rotating around roll axis. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW=8, /* Gimbal device supports to follow a roll angle relative to the vehicle. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK=16, /* Gimbal device supports locking to a roll angle (generally that's the default with roll stabilized). | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS=32, /* Gimbal device supports rotating around pitch axis. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW=64, /* Gimbal device supports to follow a pitch angle relative to the vehicle. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK=128, /* Gimbal device supports locking to a pitch angle (generally that's the default with pitch stabilized). | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS=256, /* Gimbal device supports rotating around yaw axis. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW=512, /* Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default). | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK=1024, /* Gimbal device supports locking to an absolute heading, i.e., yaw angle relative to North (earth frame, often this is an option available). | */
   GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW=2048, /* Gimbal device supports yawing/panning infinitely (e.g. using slip disk). | */
   GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME=4096, /* Gimbal device supports yaw angles and angular velocities relative to North (earth frame). This usually requires support by an autopilot via AUTOPILOT_STATE_FOR_GIMBAL_DEVICE. Support can go on and off during runtime, which is reported by the flag GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_IN_EARTH_FRAME. | */
   GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS=8192, /* Gimbal device supports radio control inputs as an alternative input for controlling the gimbal orientation. | */
   GIMBAL_DEVICE_CAP_FLAGS_ENUM_END=8193, /*  | */
} GIMBAL_DEVICE_CAP_FLAGS;
#endif

/** @brief Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags. */
#ifndef HAVE_ENUM_GIMBAL_MANAGER_CAP_FLAGS
#define HAVE_ENUM_GIMBAL_MANAGER_CAP_FLAGS
typedef enum GIMBAL_MANAGER_CAP_FLAGS
{
   GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT=1, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL=2, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS=4, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW=8, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK=16, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS=32, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW=64, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK=128, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS=256, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW=512, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK=1024, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK. | */
   GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW=2048, /* Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW. | */
   GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME=4096, /* Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME. | */
   GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS=8192, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS. | */
   GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL=65536, /* Gimbal manager supports to point to a local position. | */
   GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL=131072, /* Gimbal manager supports to point to a global latitude, longitude, altitude position. | */
   GIMBAL_MANAGER_CAP_FLAGS_ENUM_END=131073, /*  | */
} GIMBAL_MANAGER_CAP_FLAGS;
#endif

/** @brief Flags for gimbal device (lower level) operation. */
#ifndef HAVE_ENUM_GIMBAL_DEVICE_FLAGS
#define HAVE_ENUM_GIMBAL_DEVICE_FLAGS
typedef enum GIMBAL_DEVICE_FLAGS
{
   GIMBAL_DEVICE_FLAGS_RETRACT=1, /* Set to retracted safe position (no stabilization), takes precedence over all other flags. | */
   GIMBAL_DEVICE_FLAGS_NEUTRAL=2, /* Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (roll=pitch=yaw=0) but may be any orientation. | */
   GIMBAL_DEVICE_FLAGS_ROLL_LOCK=4, /* Lock roll angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal. | */
   GIMBAL_DEVICE_FLAGS_PITCH_LOCK=8, /* Lock pitch angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal. | */
   GIMBAL_DEVICE_FLAGS_YAW_LOCK=16, /* Lock yaw angle to absolute angle relative to North (not relative to vehicle). If this flag is set, the yaw angle and z component of angular velocity are relative to North (earth frame, x-axis pointing North), else they are relative to the vehicle heading (vehicle frame, earth frame rotated so that the x-axis is pointing forward). | */
   GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME=32, /* Yaw angle and z component of angular velocity are relative to the vehicle heading (vehicle frame, earth frame rotated such that the x-axis is pointing forward). | */
   GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME=64, /* Yaw angle and z component of angular velocity are relative to North (earth frame, x-axis is pointing North). | */
   GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME=128, /* Gimbal device can accept yaw angle inputs relative to North (earth frame). This flag is only for reporting (attempts to set this flag are ignored). | */
   GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE=256, /* The gimbal orientation is set exclusively by the RC signals feed to the gimbal's radio control inputs. MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE) are ignored. | */
   GIMBAL_DEVICE_FLAGS_RC_MIXED=512, /* The gimbal orientation is determined by combining/mixing the RC signals feed to the gimbal's radio control inputs and the MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE). How these two controls are combined or mixed is not defined by the protocol but is up to the implementation. | */
   GIMBAL_DEVICE_FLAGS_ENUM_END=513, /*  | */
} GIMBAL_DEVICE_FLAGS;
#endif

/** @brief Flags for high level gimbal manager operation The first 16 bits are identical to the GIMBAL_DEVICE_FLAGS. */
#ifndef HAVE_ENUM_GIMBAL_MANAGER_FLAGS
#define HAVE_ENUM_GIMBAL_MANAGER_FLAGS
typedef enum GIMBAL_MANAGER_FLAGS
{
   GIMBAL_MANAGER_FLAGS_RETRACT=1, /* Based on GIMBAL_DEVICE_FLAGS_RETRACT. | */
   GIMBAL_MANAGER_FLAGS_NEUTRAL=2, /* Based on GIMBAL_DEVICE_FLAGS_NEUTRAL. | */
   GIMBAL_MANAGER_FLAGS_ROLL_LOCK=4, /* Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK. | */
   GIMBAL_MANAGER_FLAGS_PITCH_LOCK=8, /* Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK. | */
   GIMBAL_MANAGER_FLAGS_YAW_LOCK=16, /* Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK. | */
   GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME=32, /* Based on GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME. | */
   GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME=64, /* Based on GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME. | */
   GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME=128, /* Based on GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME. | */
   GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE=256, /* Based on GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE. | */
   GIMBAL_MANAGER_FLAGS_RC_MIXED=512, /* Based on GIMBAL_DEVICE_FLAGS_RC_MIXED. | */
   GIMBAL_MANAGER_FLAGS_ENUM_END=513, /*  | */
} GIMBAL_MANAGER_FLAGS;
#endif

/** @brief Gimbal device (low level) error flags (bitmap, 0 means no error) */
#ifndef HAVE_ENUM_GIMBAL_DEVICE_ERROR_FLAGS
#define HAVE_ENUM_GIMBAL_DEVICE_ERROR_FLAGS
typedef enum GIMBAL_DEVICE_ERROR_FLAGS
{
   GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT=1, /* Gimbal device is limited by hardware roll limit. | */
   GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT=2, /* Gimbal device is limited by hardware pitch limit. | */
   GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT=4, /* Gimbal device is limited by hardware yaw limit. | */
   GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR=8, /* There is an error with the gimbal encoders. | */
   GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR=16, /* There is an error with the gimbal power source. | */
   GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR=32, /* There is an error with the gimbal motors. | */
   GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR=64, /* There is an error with the gimbal's software. | */
   GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR=128, /* There is an error with the gimbal's communication. | */
   GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING=256, /* Gimbal device is currently calibrating. | */
   GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER=512, /* Gimbal device is not assigned to a gimbal manager. | */
   GIMBAL_DEVICE_ERROR_FLAGS_ENUM_END=513, /*  | */
} GIMBAL_DEVICE_ERROR_FLAGS;
#endif

/** @brief Gripper actions. */
#ifndef HAVE_ENUM_GRIPPER_ACTIONS
#define HAVE_ENUM_GRIPPER_ACTIONS
typedef enum GRIPPER_ACTIONS
{
   GRIPPER_ACTION_RELEASE=0, /* Gripper release cargo. | */
   GRIPPER_ACTION_GRAB=1, /* Gripper grab onto cargo. | */
   GRIPPER_ACTIONS_ENUM_END=2, /*  | */
} GRIPPER_ACTIONS;
#endif

/** @brief Winch actions. */
#ifndef HAVE_ENUM_WINCH_ACTIONS
#define HAVE_ENUM_WINCH_ACTIONS
typedef enum WINCH_ACTIONS
{
   WINCH_RELAXED=0, /* Allow motor to freewheel. | */
   WINCH_RELATIVE_LENGTH_CONTROL=1, /* Wind or unwind specified length of line, optionally using specified rate. | */
   WINCH_RATE_CONTROL=2, /* Wind or unwind line at specified rate. | */
   WINCH_LOCK=3, /* Perform the locking sequence to relieve motor while in the fully retracted position. Only action and instance command parameters are used, others are ignored. | */
   WINCH_DELIVER=4, /* Sequence of drop, slow down, touch down, reel up, lock. Only action and instance command parameters are used, others are ignored. | */
   WINCH_HOLD=5, /* Engage motor and hold current position. Only action and instance command parameters are used, others are ignored. | */
   WINCH_RETRACT=6, /* Return the reel to the fully retracted position. Only action and instance command parameters are used, others are ignored. | */
   WINCH_LOAD_LINE=7, /* Load the reel with line. The winch will calculate the total loaded length and stop when the tension exceeds a threshold. Only action and instance command parameters are used, others are ignored. | */
   WINCH_ABANDON_LINE=8, /* Spool out the entire length of the line. Only action and instance command parameters are used, others are ignored. | */
   WINCH_ACTIONS_ENUM_END=9, /*  | */
} WINCH_ACTIONS;
#endif

/** @brief Generalized UAVCAN node health */
#ifndef HAVE_ENUM_UAVCAN_NODE_HEALTH
#define HAVE_ENUM_UAVCAN_NODE_HEALTH
typedef enum UAVCAN_NODE_HEALTH
{
   UAVCAN_NODE_HEALTH_OK=0, /* The node is functioning properly. | */
   UAVCAN_NODE_HEALTH_WARNING=1, /* A critical parameter went out of range or the node has encountered a minor failure. | */
   UAVCAN_NODE_HEALTH_ERROR=2, /* The node has encountered a major failure. | */
   UAVCAN_NODE_HEALTH_CRITICAL=3, /* The node has suffered a fatal malfunction. | */
   UAVCAN_NODE_HEALTH_ENUM_END=4, /*  | */
} UAVCAN_NODE_HEALTH;
#endif

/** @brief Generalized UAVCAN node mode */
#ifndef HAVE_ENUM_UAVCAN_NODE_MODE
#define HAVE_ENUM_UAVCAN_NODE_MODE
typedef enum UAVCAN_NODE_MODE
{
   UAVCAN_NODE_MODE_OPERATIONAL=0, /* The node is performing its primary functions. | */
   UAVCAN_NODE_MODE_INITIALIZATION=1, /* The node is initializing; this mode is entered immediately after startup. | */
   UAVCAN_NODE_MODE_MAINTENANCE=2, /* The node is under maintenance. | */
   UAVCAN_NODE_MODE_SOFTWARE_UPDATE=3, /* The node is in the process of updating its software. | */
   UAVCAN_NODE_MODE_OFFLINE=7, /* The node is no longer available online. | */
   UAVCAN_NODE_MODE_ENUM_END=8, /*  | */
} UAVCAN_NODE_MODE;
#endif

/** @brief Flags to indicate the status of camera storage. */
#ifndef HAVE_ENUM_STORAGE_STATUS
#define HAVE_ENUM_STORAGE_STATUS
typedef enum STORAGE_STATUS
{
   STORAGE_STATUS_EMPTY=0, /* Storage is missing (no microSD card loaded for example.) | */
   STORAGE_STATUS_UNFORMATTED=1, /* Storage present but unformatted. | */
   STORAGE_STATUS_READY=2, /* Storage present and ready. | */
   STORAGE_STATUS_NOT_SUPPORTED=3, /* Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored. | */
   STORAGE_STATUS_ENUM_END=4, /*  | */
} STORAGE_STATUS;
#endif

/** @brief Flags to indicate the type of storage. */
#ifndef HAVE_ENUM_STORAGE_TYPE
#define HAVE_ENUM_STORAGE_TYPE
typedef enum STORAGE_TYPE
{
   STORAGE_TYPE_UNKNOWN=0, /* Storage type is not known. | */
   STORAGE_TYPE_USB_STICK=1, /* Storage type is USB device. | */
   STORAGE_TYPE_SD=2, /* Storage type is SD card. | */
   STORAGE_TYPE_MICROSD=3, /* Storage type is microSD card. | */
   STORAGE_TYPE_CF=4, /* Storage type is CFast. | */
   STORAGE_TYPE_CFE=5, /* Storage type is CFexpress. | */
   STORAGE_TYPE_XQD=6, /* Storage type is XQD. | */
   STORAGE_TYPE_HD=7, /* Storage type is HD mass storage type. | */
   STORAGE_TYPE_OTHER=254, /* Storage type is other, not listed type. | */
   STORAGE_TYPE_ENUM_END=255, /*  | */
} STORAGE_TYPE;
#endif

/** @brief Flags to indicate usage for a particular storage (see STORAGE_INFORMATION.storage_usage and MAV_CMD_SET_STORAGE_USAGE). */
#ifndef HAVE_ENUM_STORAGE_USAGE_FLAG
#define HAVE_ENUM_STORAGE_USAGE_FLAG
typedef enum STORAGE_USAGE_FLAG
{
   STORAGE_USAGE_FLAG_SET=1, /* Always set to 1 (indicates STORAGE_INFORMATION.storage_usage is supported). | */
   STORAGE_USAGE_FLAG_PHOTO=2, /* Storage for saving photos. | */
   STORAGE_USAGE_FLAG_VIDEO=4, /* Storage for saving videos. | */
   STORAGE_USAGE_FLAG_LOGS=8, /* Storage for saving logs. | */
   STORAGE_USAGE_FLAG_ENUM_END=9, /*  | */
} STORAGE_USAGE_FLAG;
#endif

/** @brief Axes that will be autotuned by MAV_CMD_DO_AUTOTUNE_ENABLE.
        Note that at least one flag must be set in MAV_CMD_DO_AUTOTUNE_ENABLE.param2: if none are set, the flight stack will tune its default set of axes. */
#ifndef HAVE_ENUM_AUTOTUNE_AXIS
#define HAVE_ENUM_AUTOTUNE_AXIS
typedef enum AUTOTUNE_AXIS
{
   AUTOTUNE_AXIS_ROLL=1, /* Autotune roll axis. | */
   AUTOTUNE_AXIS_PITCH=2, /* Autotune pitch axis. | */
   AUTOTUNE_AXIS_YAW=4, /* Autotune yaw axis. | */
   AUTOTUNE_AXIS_ENUM_END=5, /*  | */
} AUTOTUNE_AXIS;
#endif

/** @brief A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages. */
#ifndef HAVE_ENUM_MAV_DATA_STREAM
#define HAVE_ENUM_MAV_DATA_STREAM
typedef enum MAV_DATA_STREAM
{
   MAV_DATA_STREAM_ALL=0, /* Enable all data streams | */
   MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
   MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
   MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
   MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
   MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION_INT messages. | */
   MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot | */
   MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot | */
   MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot | */
   MAV_DATA_STREAM_ENUM_END=13, /*  | */
} MAV_DATA_STREAM;
#endif

/** @brief The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI). */
#ifndef HAVE_ENUM_MAV_ROI
#define HAVE_ENUM_MAV_ROI
typedef enum MAV_ROI
{
   MAV_ROI_NONE=0, /* No region of interest. | */
   MAV_ROI_WPNEXT=1, /* Point toward next waypoint, with optional pitch/roll/yaw offset. | */
   MAV_ROI_WPINDEX=2, /* Point toward given waypoint. | */
   MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
   MAV_ROI_TARGET=4, /* Point toward of given id. | */
   MAV_ROI_ENUM_END=5, /*  | */
} MAV_ROI;
#endif

/** @brief Specifies the datatype of a MAVLink parameter. */
#ifndef HAVE_ENUM_MAV_PARAM_TYPE
#define HAVE_ENUM_MAV_PARAM_TYPE
typedef enum MAV_PARAM_TYPE
{
   MAV_PARAM_TYPE_UINT8=1, /* 8-bit unsigned integer | */
   MAV_PARAM_TYPE_INT8=2, /* 8-bit signed integer | */
   MAV_PARAM_TYPE_UINT16=3, /* 16-bit unsigned integer | */
   MAV_PARAM_TYPE_INT16=4, /* 16-bit signed integer | */
   MAV_PARAM_TYPE_UINT32=5, /* 32-bit unsigned integer | */
   MAV_PARAM_TYPE_INT32=6, /* 32-bit signed integer | */
   MAV_PARAM_TYPE_UINT64=7, /* 64-bit unsigned integer | */
   MAV_PARAM_TYPE_INT64=8, /* 64-bit signed integer | */
   MAV_PARAM_TYPE_REAL32=9, /* 32-bit floating-point | */
   MAV_PARAM_TYPE_REAL64=10, /* 64-bit floating-point | */
   MAV_PARAM_TYPE_ENUM_END=11, /*  | */
} MAV_PARAM_TYPE;
#endif

/** @brief Specifies the datatype of a MAVLink extended parameter. */
#ifndef HAVE_ENUM_MAV_PARAM_EXT_TYPE
#define HAVE_ENUM_MAV_PARAM_EXT_TYPE
typedef enum MAV_PARAM_EXT_TYPE
{
   MAV_PARAM_EXT_TYPE_UINT8=1, /* 8-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT8=2, /* 8-bit signed integer | */
   MAV_PARAM_EXT_TYPE_UINT16=3, /* 16-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT16=4, /* 16-bit signed integer | */
   MAV_PARAM_EXT_TYPE_UINT32=5, /* 32-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT32=6, /* 32-bit signed integer | */
   MAV_PARAM_EXT_TYPE_UINT64=7, /* 64-bit unsigned integer | */
   MAV_PARAM_EXT_TYPE_INT64=8, /* 64-bit signed integer | */
   MAV_PARAM_EXT_TYPE_REAL32=9, /* 32-bit floating-point | */
   MAV_PARAM_EXT_TYPE_REAL64=10, /* 64-bit floating-point | */
   MAV_PARAM_EXT_TYPE_CUSTOM=11, /* Custom Type | */
   MAV_PARAM_EXT_TYPE_ENUM_END=12, /*  | */
} MAV_PARAM_EXT_TYPE;
#endif

/** @brief Result from a MAVLink command (MAV_CMD) */
#ifndef HAVE_ENUM_MAV_RESULT
#define HAVE_ENUM_MAV_RESULT
typedef enum MAV_RESULT
{
   MAV_RESULT_ACCEPTED=0, /* Command is valid (is supported and has valid parameters), and was executed. | */
   MAV_RESULT_TEMPORARILY_REJECTED=1, /* Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work. | */
   MAV_RESULT_DENIED=2, /* Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work. | */
   MAV_RESULT_UNSUPPORTED=3, /* Command is not supported (unknown). | */
   MAV_RESULT_FAILED=4, /* Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc. | */
   MAV_RESULT_IN_PROGRESS=5, /* Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation. There is no need for the sender to retry the command, but if done during execution, the component will return MAV_RESULT_IN_PROGRESS with an updated progress. | */
   MAV_RESULT_COMMAND_LONG_ONLY=7, /* Command is only accepted when sent as a COMMAND_LONG. | */
   MAV_RESULT_COMMAND_INT_ONLY=8, /* Command is only accepted when sent as a COMMAND_INT. | */
   MAV_RESULT_ENUM_END=9, /*  | */
} MAV_RESULT;
#endif

/** @brief Result of mission operation (in a MISSION_ACK message). */
#ifndef HAVE_ENUM_MAV_MISSION_RESULT
#define HAVE_ENUM_MAV_MISSION_RESULT
typedef enum MAV_MISSION_RESULT
{
   MAV_MISSION_ACCEPTED=0, /* mission accepted OK | */
   MAV_MISSION_ERROR=1, /* Generic error / not accepting mission commands at all right now. | */
   MAV_MISSION_UNSUPPORTED_FRAME=2, /* Coordinate frame is not supported. | */
   MAV_MISSION_UNSUPPORTED=3, /* Command is not supported. | */
   MAV_MISSION_NO_SPACE=4, /* Mission items exceed storage space. | */
   MAV_MISSION_INVALID=5, /* One of the parameters has an invalid value. | */
   MAV_MISSION_INVALID_PARAM1=6, /* param1 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM2=7, /* param2 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM3=8, /* param3 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM4=9, /* param4 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM5_X=10, /* x / param5 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM6_Y=11, /* y / param6 has an invalid value. | */
   MAV_MISSION_INVALID_PARAM7=12, /* z / param7 has an invalid value. | */
   MAV_MISSION_INVALID_SEQUENCE=13, /* Mission item received out of sequence | */
   MAV_MISSION_DENIED=14, /* Not accepting any mission commands from this communication partner. | */
   MAV_MISSION_OPERATION_CANCELLED=15, /* Current mission operation cancelled (e.g. mission upload, mission download). | */
   MAV_MISSION_RESULT_ENUM_END=16, /*  | */
} MAV_MISSION_RESULT;
#endif

/** @brief Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. */
#ifndef HAVE_ENUM_MAV_SEVERITY
#define HAVE_ENUM_MAV_SEVERITY
typedef enum MAV_SEVERITY
{
   MAV_SEVERITY_EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
   MAV_SEVERITY_ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
   MAV_SEVERITY_CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
   MAV_SEVERITY_ERROR=3, /* Indicates an error in secondary/redundant systems. | */
   MAV_SEVERITY_WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
   MAV_SEVERITY_NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
   MAV_SEVERITY_INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
   MAV_SEVERITY_DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
   MAV_SEVERITY_ENUM_END=8, /*  | */
} MAV_SEVERITY;
#endif

/** @brief Power supply status flags (bitmask) */
#ifndef HAVE_ENUM_MAV_POWER_STATUS
#define HAVE_ENUM_MAV_POWER_STATUS
typedef enum MAV_POWER_STATUS
{
   MAV_POWER_STATUS_BRICK_VALID=1, /* main brick power supply valid | */
   MAV_POWER_STATUS_SERVO_VALID=2, /* main servo power supply valid for FMU | */
   MAV_POWER_STATUS_USB_CONNECTED=4, /* USB power is connected | */
   MAV_POWER_STATUS_PERIPH_OVERCURRENT=8, /* peripheral supply is in over-current state | */
   MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT=16, /* hi-power peripheral supply is in over-current state | */
   MAV_POWER_STATUS_CHANGED=32, /* Power status has changed since boot | */
   MAV_POWER_STATUS_ENUM_END=33, /*  | */
} MAV_POWER_STATUS;
#endif

/** @brief SERIAL_CONTROL device types */
#ifndef HAVE_ENUM_SERIAL_CONTROL_DEV
#define HAVE_ENUM_SERIAL_CONTROL_DEV
typedef enum SERIAL_CONTROL_DEV
{
   SERIAL_CONTROL_DEV_TELEM1=0, /* First telemetry port | */
   SERIAL_CONTROL_DEV_TELEM2=1, /* Second telemetry port | */
   SERIAL_CONTROL_DEV_GPS1=2, /* First GPS port | */
   SERIAL_CONTROL_DEV_GPS2=3, /* Second GPS port | */
   SERIAL_CONTROL_DEV_SHELL=10, /* system shell | */
   SERIAL_CONTROL_SERIAL0=100, /* SERIAL0 | */
   SERIAL_CONTROL_SERIAL1=101, /* SERIAL1 | */
   SERIAL_CONTROL_SERIAL2=102, /* SERIAL2 | */
   SERIAL_CONTROL_SERIAL3=103, /* SERIAL3 | */
   SERIAL_CONTROL_SERIAL4=104, /* SERIAL4 | */
   SERIAL_CONTROL_SERIAL5=105, /* SERIAL5 | */
   SERIAL_CONTROL_SERIAL6=106, /* SERIAL6 | */
   SERIAL_CONTROL_SERIAL7=107, /* SERIAL7 | */
   SERIAL_CONTROL_SERIAL8=108, /* SERIAL8 | */
   SERIAL_CONTROL_SERIAL9=109, /* SERIAL9 | */
   SERIAL_CONTROL_DEV_ENUM_END=110, /*  | */
} SERIAL_CONTROL_DEV;
#endif

/** @brief SERIAL_CONTROL flags (bitmask) */
#ifndef HAVE_ENUM_SERIAL_CONTROL_FLAG
#define HAVE_ENUM_SERIAL_CONTROL_FLAG
typedef enum SERIAL_CONTROL_FLAG
{
   SERIAL_CONTROL_FLAG_REPLY=1, /* Set if this is a reply | */
   SERIAL_CONTROL_FLAG_RESPOND=2, /* Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message | */
   SERIAL_CONTROL_FLAG_EXCLUSIVE=4, /* Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set | */
   SERIAL_CONTROL_FLAG_BLOCKING=8, /* Block on writes to the serial port | */
   SERIAL_CONTROL_FLAG_MULTI=16, /* Send multiple replies until port is drained | */
   SERIAL_CONTROL_FLAG_ENUM_END=17, /*  | */
} SERIAL_CONTROL_FLAG;
#endif

/** @brief Enumeration of distance sensor types */
#ifndef HAVE_ENUM_MAV_DISTANCE_SENSOR
#define HAVE_ENUM_MAV_DISTANCE_SENSOR
typedef enum MAV_DISTANCE_SENSOR
{
   MAV_DISTANCE_SENSOR_LASER=0, /* Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units | */
   MAV_DISTANCE_SENSOR_ULTRASOUND=1, /* Ultrasound rangefinder, e.g. MaxBotix units | */
   MAV_DISTANCE_SENSOR_INFRARED=2, /* Infrared rangefinder, e.g. Sharp units | */
   MAV_DISTANCE_SENSOR_RADAR=3, /* Radar type, e.g. uLanding units | */
   MAV_DISTANCE_SENSOR_UNKNOWN=4, /* Broken or unknown type, e.g. analog units | */
   MAV_DISTANCE_SENSOR_ENUM_END=5, /*  | */
} MAV_DISTANCE_SENSOR;
#endif

/** @brief Enumeration of sensor orientation, according to its rotations */
#ifndef HAVE_ENUM_MAV_SENSOR_ORIENTATION
#define HAVE_ENUM_MAV_SENSOR_ORIENTATION
typedef enum MAV_SENSOR_ORIENTATION
{
   MAV_SENSOR_ROTATION_NONE=0, /* Roll: 0, Pitch: 0, Yaw: 0 | */
   MAV_SENSOR_ROTATION_YAW_45=1, /* Roll: 0, Pitch: 0, Yaw: 45 | */
   MAV_SENSOR_ROTATION_YAW_90=2, /* Roll: 0, Pitch: 0, Yaw: 90 | */
   MAV_SENSOR_ROTATION_YAW_135=3, /* Roll: 0, Pitch: 0, Yaw: 135 | */
   MAV_SENSOR_ROTATION_YAW_180=4, /* Roll: 0, Pitch: 0, Yaw: 180 | */
   MAV_SENSOR_ROTATION_YAW_225=5, /* Roll: 0, Pitch: 0, Yaw: 225 | */
   MAV_SENSOR_ROTATION_YAW_270=6, /* Roll: 0, Pitch: 0, Yaw: 270 | */
   MAV_SENSOR_ROTATION_YAW_315=7, /* Roll: 0, Pitch: 0, Yaw: 315 | */
   MAV_SENSOR_ROTATION_ROLL_180=8, /* Roll: 180, Pitch: 0, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_180_YAW_45=9, /* Roll: 180, Pitch: 0, Yaw: 45 | */
   MAV_SENSOR_ROTATION_ROLL_180_YAW_90=10, /* Roll: 180, Pitch: 0, Yaw: 90 | */
   MAV_SENSOR_ROTATION_ROLL_180_YAW_135=11, /* Roll: 180, Pitch: 0, Yaw: 135 | */
   MAV_SENSOR_ROTATION_PITCH_180=12, /* Roll: 0, Pitch: 180, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_180_YAW_225=13, /* Roll: 180, Pitch: 0, Yaw: 225 | */
   MAV_SENSOR_ROTATION_ROLL_180_YAW_270=14, /* Roll: 180, Pitch: 0, Yaw: 270 | */
   MAV_SENSOR_ROTATION_ROLL_180_YAW_315=15, /* Roll: 180, Pitch: 0, Yaw: 315 | */
   MAV_SENSOR_ROTATION_ROLL_90=16, /* Roll: 90, Pitch: 0, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_90_YAW_45=17, /* Roll: 90, Pitch: 0, Yaw: 45 | */
   MAV_SENSOR_ROTATION_ROLL_90_YAW_90=18, /* Roll: 90, Pitch: 0, Yaw: 90 | */
   MAV_SENSOR_ROTATION_ROLL_90_YAW_135=19, /* Roll: 90, Pitch: 0, Yaw: 135 | */
   MAV_SENSOR_ROTATION_ROLL_270=20, /* Roll: 270, Pitch: 0, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_270_YAW_45=21, /* Roll: 270, Pitch: 0, Yaw: 45 | */
   MAV_SENSOR_ROTATION_ROLL_270_YAW_90=22, /* Roll: 270, Pitch: 0, Yaw: 90 | */
   MAV_SENSOR_ROTATION_ROLL_270_YAW_135=23, /* Roll: 270, Pitch: 0, Yaw: 135 | */
   MAV_SENSOR_ROTATION_PITCH_90=24, /* Roll: 0, Pitch: 90, Yaw: 0 | */
   MAV_SENSOR_ROTATION_PITCH_270=25, /* Roll: 0, Pitch: 270, Yaw: 0 | */
   MAV_SENSOR_ROTATION_PITCH_180_YAW_90=26, /* Roll: 0, Pitch: 180, Yaw: 90 | */
   MAV_SENSOR_ROTATION_PITCH_180_YAW_270=27, /* Roll: 0, Pitch: 180, Yaw: 270 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_90=28, /* Roll: 90, Pitch: 90, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_180_PITCH_90=29, /* Roll: 180, Pitch: 90, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_270_PITCH_90=30, /* Roll: 270, Pitch: 90, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_180=31, /* Roll: 90, Pitch: 180, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_270_PITCH_180=32, /* Roll: 270, Pitch: 180, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_270=33, /* Roll: 90, Pitch: 270, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_180_PITCH_270=34, /* Roll: 180, Pitch: 270, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_270_PITCH_270=35, /* Roll: 270, Pitch: 270, Yaw: 0 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90=36, /* Roll: 90, Pitch: 180, Yaw: 90 | */
   MAV_SENSOR_ROTATION_ROLL_90_YAW_270=37, /* Roll: 90, Pitch: 0, Yaw: 270 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293=38, /* Roll: 90, Pitch: 68, Yaw: 293 | */
   MAV_SENSOR_ROTATION_PITCH_315=39, /* Pitch: 315 | */
   MAV_SENSOR_ROTATION_ROLL_90_PITCH_315=40, /* Roll: 90, Pitch: 315 | */
   MAV_SENSOR_ROTATION_CUSTOM=100, /* Custom orientation | */
   MAV_SENSOR_ORIENTATION_ENUM_END=101, /*  | */
} MAV_SENSOR_ORIENTATION;
#endif

/** @brief Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. */
#ifndef HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
#define HAVE_ENUM_MAV_PROTOCOL_CAPABILITY
typedef enum MAV_PROTOCOL_CAPABILITY
{
   MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT=1, /* Autopilot supports the MISSION_ITEM float message type.
          Note that MISSION_ITEM is deprecated, and autopilots should use MISSION_INT instead.
         | */
   MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT=2, /* Autopilot supports the new param float message type. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_INT=4, /* Autopilot supports MISSION_ITEM_INT scaled integer message type.
          Note that this flag must always be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM, which is deprecated).
         | */
   MAV_PROTOCOL_CAPABILITY_COMMAND_INT=8, /* Autopilot supports COMMAND_INT scaled integer message type. | */
   MAV_PROTOCOL_CAPABILITY_PARAM_UNION=16, /* Autopilot supports the new param union message type. | */
   MAV_PROTOCOL_CAPABILITY_FTP=32, /* Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html. | */
   MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET=64, /* Autopilot supports commanding attitude offboard. | */
   MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED=128, /* Autopilot supports commanding position and velocity targets in local NED frame. | */
   MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT=256, /* Autopilot supports commanding position and velocity targets in global scaled integers. | */
   MAV_PROTOCOL_CAPABILITY_TERRAIN=512, /* Autopilot supports terrain protocol / data handling. | */
   MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET=1024, /* Autopilot supports direct actuator control. | */
   MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION=2048, /* Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination). | */
   MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION=4096, /* Autopilot supports onboard compass calibration. | */
   MAV_PROTOCOL_CAPABILITY_MAVLINK2=8192, /* Autopilot supports MAVLink version 2. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_FENCE=16384, /* Autopilot supports mission fence protocol. | */
   MAV_PROTOCOL_CAPABILITY_MISSION_RALLY=32768, /* Autopilot supports mission rally point protocol. | */
   MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION=65536, /* Autopilot supports the flight information protocol. | */
   MAV_PROTOCOL_CAPABILITY_ENUM_END=65537, /*  | */
} MAV_PROTOCOL_CAPABILITY;
#endif

/** @brief Type of mission items being requested/sent in mission protocol. */
#ifndef HAVE_ENUM_MAV_MISSION_TYPE
#define HAVE_ENUM_MAV_MISSION_TYPE
typedef enum MAV_MISSION_TYPE
{
   MAV_MISSION_TYPE_MISSION=0, /* Items are mission commands for main mission. | */
   MAV_MISSION_TYPE_FENCE=1, /* Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items. | */
   MAV_MISSION_TYPE_RALLY=2, /* Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items. | */
   MAV_MISSION_TYPE_ALL=255, /* Only used in MISSION_CLEAR_ALL to clear all mission types. | */
   MAV_MISSION_TYPE_ENUM_END=256, /*  | */
} MAV_MISSION_TYPE;
#endif

/** @brief Enumeration of estimator types */
#ifndef HAVE_ENUM_MAV_ESTIMATOR_TYPE
#define HAVE_ENUM_MAV_ESTIMATOR_TYPE
typedef enum MAV_ESTIMATOR_TYPE
{
   MAV_ESTIMATOR_TYPE_UNKNOWN=0, /* Unknown type of the estimator. | */
   MAV_ESTIMATOR_TYPE_NAIVE=1, /* This is a naive estimator without any real covariance feedback. | */
   MAV_ESTIMATOR_TYPE_VISION=2, /* Computer vision based estimate. Might be up to scale. | */
   MAV_ESTIMATOR_TYPE_VIO=3, /* Visual-inertial estimate. | */
   MAV_ESTIMATOR_TYPE_GPS=4, /* Plain GPS estimate. | */
   MAV_ESTIMATOR_TYPE_GPS_INS=5, /* Estimator integrating GPS and inertial sensing. | */
   MAV_ESTIMATOR_TYPE_MOCAP=6, /* Estimate from external motion capturing system. | */
   MAV_ESTIMATOR_TYPE_LIDAR=7, /* Estimator based on lidar sensor input. | */
   MAV_ESTIMATOR_TYPE_AUTOPILOT=8, /* Estimator on autopilot. | */
   MAV_ESTIMATOR_TYPE_ENUM_END=9, /*  | */
} MAV_ESTIMATOR_TYPE;
#endif

/** @brief Enumeration of battery types */
#ifndef HAVE_ENUM_MAV_BATTERY_TYPE
#define HAVE_ENUM_MAV_BATTERY_TYPE
typedef enum MAV_BATTERY_TYPE
{
   MAV_BATTERY_TYPE_UNKNOWN=0, /* Not specified. | */
   MAV_BATTERY_TYPE_LIPO=1, /* Lithium polymer battery | */
   MAV_BATTERY_TYPE_LIFE=2, /* Lithium-iron-phosphate battery | */
   MAV_BATTERY_TYPE_LION=3, /* Lithium-ION battery | */
   MAV_BATTERY_TYPE_NIMH=4, /* Nickel metal hydride battery | */
   MAV_BATTERY_TYPE_ENUM_END=5, /*  | */
} MAV_BATTERY_TYPE;
#endif

/** @brief Enumeration of battery functions */
#ifndef HAVE_ENUM_MAV_BATTERY_FUNCTION
#define HAVE_ENUM_MAV_BATTERY_FUNCTION
typedef enum MAV_BATTERY_FUNCTION
{
   MAV_BATTERY_FUNCTION_UNKNOWN=0, /* Battery function is unknown | */
   MAV_BATTERY_FUNCTION_ALL=1, /* Battery supports all flight systems | */
   MAV_BATTERY_FUNCTION_PROPULSION=2, /* Battery for the propulsion system | */
   MAV_BATTERY_FUNCTION_AVIONICS=3, /* Avionics battery | */
   MAV_BATTERY_TYPE_PAYLOAD=4, /* Payload battery | */
   MAV_BATTERY_FUNCTION_ENUM_END=5, /*  | */
} MAV_BATTERY_FUNCTION;
#endif

/** @brief Enumeration for battery charge states. */
#ifndef HAVE_ENUM_MAV_BATTERY_CHARGE_STATE
#define HAVE_ENUM_MAV_BATTERY_CHARGE_STATE
typedef enum MAV_BATTERY_CHARGE_STATE
{
   MAV_BATTERY_CHARGE_STATE_UNDEFINED=0, /* Low battery state is not provided | */
   MAV_BATTERY_CHARGE_STATE_OK=1, /* Battery is not in low state. Normal operation. | */
   MAV_BATTERY_CHARGE_STATE_LOW=2, /* Battery state is low, warn and monitor close. | */
   MAV_BATTERY_CHARGE_STATE_CRITICAL=3, /* Battery state is critical, return or abort immediately. | */
   MAV_BATTERY_CHARGE_STATE_EMERGENCY=4, /* Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage. | */
   MAV_BATTERY_CHARGE_STATE_FAILED=5, /* Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT. | */
   MAV_BATTERY_CHARGE_STATE_UNHEALTHY=6, /* Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT. | */
   MAV_BATTERY_CHARGE_STATE_CHARGING=7, /* Battery is charging. | */
   MAV_BATTERY_CHARGE_STATE_ENUM_END=8, /*  | */
} MAV_BATTERY_CHARGE_STATE;
#endif

/** @brief Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight. */
#ifndef HAVE_ENUM_MAV_BATTERY_MODE
#define HAVE_ENUM_MAV_BATTERY_MODE
typedef enum MAV_BATTERY_MODE
{
   MAV_BATTERY_MODE_UNKNOWN=0, /* Battery mode not supported/unknown battery mode/normal operation. | */
   MAV_BATTERY_MODE_AUTO_DISCHARGING=1, /* Battery is auto discharging (towards storage level). | */
   MAV_BATTERY_MODE_HOT_SWAP=2, /* Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits). | */
   MAV_BATTERY_MODE_ENUM_END=3, /*  | */
} MAV_BATTERY_MODE;
#endif

/** @brief Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set. */
#ifndef HAVE_ENUM_MAV_BATTERY_FAULT
#define HAVE_ENUM_MAV_BATTERY_FAULT
typedef enum MAV_BATTERY_FAULT
{
   MAV_BATTERY_FAULT_DEEP_DISCHARGE=1, /* Battery has deep discharged. | */
   MAV_BATTERY_FAULT_SPIKES=2, /* Voltage spikes. | */
   MAV_BATTERY_FAULT_CELL_FAIL=4, /* One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used). | */
   MAV_BATTERY_FAULT_OVER_CURRENT=8, /* Over-current fault. | */
   MAV_BATTERY_FAULT_OVER_TEMPERATURE=16, /* Over-temperature fault. | */
   MAV_BATTERY_FAULT_UNDER_TEMPERATURE=32, /* Under-temperature fault. | */
   MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE=64, /* Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage). | */
   MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE=128, /* Battery firmware is not compatible with current autopilot firmware. | */
   BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION=256, /* Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s). | */
   MAV_BATTERY_FAULT_ENUM_END=257, /*  | */
} MAV_BATTERY_FAULT;
#endif

/** @brief Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS are conditions that cause the generator to fail. Warnings are conditions that require attention before the next use (they indicate the system is not operating properly). */
#ifndef HAVE_ENUM_MAV_GENERATOR_STATUS_FLAG
#define HAVE_ENUM_MAV_GENERATOR_STATUS_FLAG
typedef enum MAV_GENERATOR_STATUS_FLAG
{
   MAV_GENERATOR_STATUS_FLAG_OFF=1, /* Generator is off. | */
   MAV_GENERATOR_STATUS_FLAG_READY=2, /* Generator is ready to start generating power. | */
   MAV_GENERATOR_STATUS_FLAG_GENERATING=4, /* Generator is generating power. | */
   MAV_GENERATOR_STATUS_FLAG_CHARGING=8, /* Generator is charging the batteries (generating enough power to charge and provide the load). | */
   MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER=16, /* Generator is operating at a reduced maximum power. | */
   MAV_GENERATOR_STATUS_FLAG_MAXPOWER=32, /* Generator is providing the maximum output. | */
   MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING=64, /* Generator is near the maximum operating temperature, cooling is insufficient. | */
   MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT=128, /* Generator hit the maximum operating temperature and shutdown. | */
   MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING=256, /* Power electronics are near the maximum operating temperature, cooling is insufficient. | */
   MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT=512, /* Power electronics hit the maximum operating temperature and shutdown. | */
   MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT=1024, /* Power electronics experienced a fault and shutdown. | */
   MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT=2048, /* The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening. | */
   MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING=4096, /* Generator controller having communication problems. | */
   MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING=8192, /* Power electronic or generator cooling system error. | */
   MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT=16384, /* Generator controller power rail experienced a fault. | */
   MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT=32768, /* Generator controller exceeded the overcurrent threshold and shutdown to prevent damage. | */
   MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT=65536, /* Generator controller detected a high current going into the batteries and shutdown to prevent battery damage. | */
   MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT=131072, /* Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating. | */
   MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT=262144, /* Batteries are under voltage (generator will not start). | */
   MAV_GENERATOR_STATUS_FLAG_START_INHIBITED=524288, /* Generator start is inhibited by e.g. a safety switch. | */
   MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED=1048576, /* Generator requires maintenance. | */
   MAV_GENERATOR_STATUS_FLAG_WARMING_UP=2097152, /* Generator is not ready to generate yet. | */
   MAV_GENERATOR_STATUS_FLAG_IDLE=4194304, /* Generator is idle. | */
   MAV_GENERATOR_STATUS_FLAG_ENUM_END=4194305, /*  | */
} MAV_GENERATOR_STATUS_FLAG;
#endif

/** @brief Enumeration of VTOL states */
#ifndef HAVE_ENUM_MAV_VTOL_STATE
#define HAVE_ENUM_MAV_VTOL_STATE
typedef enum MAV_VTOL_STATE
{
   MAV_VTOL_STATE_UNDEFINED=0, /* MAV is not configured as VTOL | */
   MAV_VTOL_STATE_TRANSITION_TO_FW=1, /* VTOL is in transition from multicopter to fixed-wing | */
   MAV_VTOL_STATE_TRANSITION_TO_MC=2, /* VTOL is in transition from fixed-wing to multicopter | */
   MAV_VTOL_STATE_MC=3, /* VTOL is in multicopter state | */
   MAV_VTOL_STATE_FW=4, /* VTOL is in fixed-wing state | */
   MAV_VTOL_STATE_ENUM_END=5, /*  | */
} MAV_VTOL_STATE;
#endif

/** @brief Enumeration of landed detector states */
#ifndef HAVE_ENUM_MAV_LANDED_STATE
#define HAVE_ENUM_MAV_LANDED_STATE
typedef enum MAV_LANDED_STATE
{
   MAV_LANDED_STATE_UNDEFINED=0, /* MAV landed state is unknown | */
   MAV_LANDED_STATE_ON_GROUND=1, /* MAV is landed (on ground) | */
   MAV_LANDED_STATE_IN_AIR=2, /* MAV is in air | */
   MAV_LANDED_STATE_TAKEOFF=3, /* MAV currently taking off | */
   MAV_LANDED_STATE_LANDING=4, /* MAV currently landing | */
   MAV_LANDED_STATE_ENUM_END=5, /*  | */
} MAV_LANDED_STATE;
#endif

/** @brief Enumeration of the ADSB altimeter types */
#ifndef HAVE_ENUM_ADSB_ALTITUDE_TYPE
#define HAVE_ENUM_ADSB_ALTITUDE_TYPE
typedef enum ADSB_ALTITUDE_TYPE
{
   ADSB_ALTITUDE_TYPE_PRESSURE_QNH=0, /* Altitude reported from a Baro source using QNH reference | */
   ADSB_ALTITUDE_TYPE_GEOMETRIC=1, /* Altitude reported from a GNSS source | */
   ADSB_ALTITUDE_TYPE_ENUM_END=2, /*  | */
} ADSB_ALTITUDE_TYPE;
#endif

/** @brief ADSB classification for the type of vehicle emitting the transponder signal */
#ifndef HAVE_ENUM_ADSB_EMITTER_TYPE
#define HAVE_ENUM_ADSB_EMITTER_TYPE
typedef enum ADSB_EMITTER_TYPE
{
   ADSB_EMITTER_TYPE_NO_INFO=0, /*  | */
   ADSB_EMITTER_TYPE_LIGHT=1, /*  | */
   ADSB_EMITTER_TYPE_SMALL=2, /*  | */
   ADSB_EMITTER_TYPE_LARGE=3, /*  | */
   ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE=4, /*  | */
   ADSB_EMITTER_TYPE_HEAVY=5, /*  | */
   ADSB_EMITTER_TYPE_HIGHLY_MANUV=6, /*  | */
   ADSB_EMITTER_TYPE_ROTOCRAFT=7, /*  | */
   ADSB_EMITTER_TYPE_UNASSIGNED=8, /*  | */
   ADSB_EMITTER_TYPE_GLIDER=9, /*  | */
   ADSB_EMITTER_TYPE_LIGHTER_AIR=10, /*  | */
   ADSB_EMITTER_TYPE_PARACHUTE=11, /*  | */
   ADSB_EMITTER_TYPE_ULTRA_LIGHT=12, /*  | */
   ADSB_EMITTER_TYPE_UNASSIGNED2=13, /*  | */
   ADSB_EMITTER_TYPE_UAV=14, /*  | */
   ADSB_EMITTER_TYPE_SPACE=15, /*  | */
   ADSB_EMITTER_TYPE_UNASSGINED3=16, /*  | */
   ADSB_EMITTER_TYPE_EMERGENCY_SURFACE=17, /*  | */
   ADSB_EMITTER_TYPE_SERVICE_SURFACE=18, /*  | */
   ADSB_EMITTER_TYPE_POINT_OBSTACLE=19, /*  | */
   ADSB_EMITTER_TYPE_ENUM_END=20, /*  | */
} ADSB_EMITTER_TYPE;
#endif

/** @brief These flags indicate status such as data validity of each data source. Set = data valid */
#ifndef HAVE_ENUM_ADSB_FLAGS
#define HAVE_ENUM_ADSB_FLAGS
typedef enum ADSB_FLAGS
{
   ADSB_FLAGS_VALID_COORDS=1, /*  | */
   ADSB_FLAGS_VALID_ALTITUDE=2, /*  | */
   ADSB_FLAGS_VALID_HEADING=4, /*  | */
   ADSB_FLAGS_VALID_VELOCITY=8, /*  | */
   ADSB_FLAGS_VALID_CALLSIGN=16, /*  | */
   ADSB_FLAGS_VALID_SQUAWK=32, /*  | */
   ADSB_FLAGS_SIMULATED=64, /*  | */
   ADSB_FLAGS_VERTICAL_VELOCITY_VALID=128, /*  | */
   ADSB_FLAGS_BARO_VALID=256, /*  | */
   ADSB_FLAGS_SOURCE_UAT=32768, /*  | */
   ADSB_FLAGS_ENUM_END=32769, /*  | */
} ADSB_FLAGS;
#endif

/** @brief Bitmap of options for the MAV_CMD_DO_REPOSITION */
#ifndef HAVE_ENUM_MAV_DO_REPOSITION_FLAGS
#define HAVE_ENUM_MAV_DO_REPOSITION_FLAGS
typedef enum MAV_DO_REPOSITION_FLAGS
{
   MAV_DO_REPOSITION_FLAGS_CHANGE_MODE=1, /* The aircraft should immediately transition into guided. This should not be set for follow me applications | */
   MAV_DO_REPOSITION_FLAGS_ENUM_END=2, /*  | */
} MAV_DO_REPOSITION_FLAGS;
#endif

/** @brief Speed setpoint types used in MAV_CMD_DO_CHANGE_SPEED */
#ifndef HAVE_ENUM_SPEED_TYPE
#define HAVE_ENUM_SPEED_TYPE
typedef enum SPEED_TYPE
{
   SPEED_TYPE_AIRSPEED=0, /* Airspeed | */
   SPEED_TYPE_GROUNDSPEED=1, /* Groundspeed | */
   SPEED_TYPE_CLIMB_SPEED=2, /* Climb speed | */
   SPEED_TYPE_DESCENT_SPEED=3, /* Descent speed | */
   SPEED_TYPE_ENUM_END=4, /*  | */
} SPEED_TYPE;
#endif

/** @brief Flags in ESTIMATOR_STATUS message */
#ifndef HAVE_ENUM_ESTIMATOR_STATUS_FLAGS
#define HAVE_ENUM_ESTIMATOR_STATUS_FLAGS
typedef enum ESTIMATOR_STATUS_FLAGS
{
   ESTIMATOR_ATTITUDE=1, /* True if the attitude estimate is good | */
   ESTIMATOR_VELOCITY_HORIZ=2, /* True if the horizontal velocity estimate is good | */
   ESTIMATOR_VELOCITY_VERT=4, /* True if the  vertical velocity estimate is good | */
   ESTIMATOR_POS_HORIZ_REL=8, /* True if the horizontal position (relative) estimate is good | */
   ESTIMATOR_POS_HORIZ_ABS=16, /* True if the horizontal position (absolute) estimate is good | */
   ESTIMATOR_POS_VERT_ABS=32, /* True if the vertical position (absolute) estimate is good | */
   ESTIMATOR_POS_VERT_AGL=64, /* True if the vertical position (above ground) estimate is good | */
   ESTIMATOR_CONST_POS_MODE=128, /* True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow) | */
   ESTIMATOR_PRED_POS_HORIZ_REL=256, /* True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate | */
   ESTIMATOR_PRED_POS_HORIZ_ABS=512, /* True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate | */
   ESTIMATOR_GPS_GLITCH=1024, /* True if the EKF has detected a GPS glitch | */
   ESTIMATOR_ACCEL_ERROR=2048, /* True if the EKF has detected bad accelerometer data | */
   ESTIMATOR_STATUS_FLAGS_ENUM_END=2049, /*  | */
} ESTIMATOR_STATUS_FLAGS;
#endif

/** @brief Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST. */
#ifndef HAVE_ENUM_MOTOR_TEST_ORDER
#define HAVE_ENUM_MOTOR_TEST_ORDER
typedef enum MOTOR_TEST_ORDER
{
   MOTOR_TEST_ORDER_DEFAULT=0, /* Default autopilot motor test method. | */
   MOTOR_TEST_ORDER_SEQUENCE=1, /* Motor numbers are specified as their index in a predefined vehicle-specific sequence. | */
   MOTOR_TEST_ORDER_BOARD=2, /* Motor numbers are specified as the output as labeled on the board. | */
   MOTOR_TEST_ORDER_ENUM_END=3, /*  | */
} MOTOR_TEST_ORDER;
#endif

/** @brief Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST. */
#ifndef HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
#define HAVE_ENUM_MOTOR_TEST_THROTTLE_TYPE
typedef enum MOTOR_TEST_THROTTLE_TYPE
{
   MOTOR_TEST_THROTTLE_PERCENT=0, /* Throttle as a percentage (0 ~ 100) | */
   MOTOR_TEST_THROTTLE_PWM=1, /* Throttle as an absolute PWM value (normally in range of 1000~2000). | */
   MOTOR_TEST_THROTTLE_PILOT=2, /* Throttle pass-through from pilot's transmitter. | */
   MOTOR_TEST_COMPASS_CAL=3, /* Per-motor compass calibration test. | */
   MOTOR_TEST_THROTTLE_TYPE_ENUM_END=4, /*  | */
} MOTOR_TEST_THROTTLE_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GPS_INPUT_IGNORE_FLAGS
#define HAVE_ENUM_GPS_INPUT_IGNORE_FLAGS
typedef enum GPS_INPUT_IGNORE_FLAGS
{
   GPS_INPUT_IGNORE_FLAG_ALT=1, /* ignore altitude field | */
   GPS_INPUT_IGNORE_FLAG_HDOP=2, /* ignore hdop field | */
   GPS_INPUT_IGNORE_FLAG_VDOP=4, /* ignore vdop field | */
   GPS_INPUT_IGNORE_FLAG_VEL_HORIZ=8, /* ignore horizontal velocity field (vn and ve) | */
   GPS_INPUT_IGNORE_FLAG_VEL_VERT=16, /* ignore vertical velocity field (vd) | */
   GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY=32, /* ignore speed accuracy field | */
   GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY=64, /* ignore horizontal accuracy field | */
   GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY=128, /* ignore vertical accuracy field | */
   GPS_INPUT_IGNORE_FLAGS_ENUM_END=129, /*  | */
} GPS_INPUT_IGNORE_FLAGS;
#endif

/** @brief Possible actions an aircraft can take to avoid a collision. */
#ifndef HAVE_ENUM_MAV_COLLISION_ACTION
#define HAVE_ENUM_MAV_COLLISION_ACTION
typedef enum MAV_COLLISION_ACTION
{
   MAV_COLLISION_ACTION_NONE=0, /* Ignore any potential collisions | */
   MAV_COLLISION_ACTION_REPORT=1, /* Report potential collision | */
   MAV_COLLISION_ACTION_ASCEND_OR_DESCEND=2, /* Ascend or Descend to avoid threat | */
   MAV_COLLISION_ACTION_MOVE_HORIZONTALLY=3, /* Move horizontally to avoid threat | */
   MAV_COLLISION_ACTION_MOVE_PERPENDICULAR=4, /* Aircraft to move perpendicular to the collision's velocity vector | */
   MAV_COLLISION_ACTION_RTL=5, /* Aircraft to fly directly back to its launch point | */
   MAV_COLLISION_ACTION_HOVER=6, /* Aircraft to stop in place | */
   MAV_COLLISION_ACTION_ENUM_END=7, /*  | */
} MAV_COLLISION_ACTION;
#endif

/** @brief Aircraft-rated danger from this threat. */
#ifndef HAVE_ENUM_MAV_COLLISION_THREAT_LEVEL
#define HAVE_ENUM_MAV_COLLISION_THREAT_LEVEL
typedef enum MAV_COLLISION_THREAT_LEVEL
{
   MAV_COLLISION_THREAT_LEVEL_NONE=0, /* Not a threat | */
   MAV_COLLISION_THREAT_LEVEL_LOW=1, /* Craft is mildly concerned about this threat | */
   MAV_COLLISION_THREAT_LEVEL_HIGH=2, /* Craft is panicking, and may take actions to avoid threat | */
   MAV_COLLISION_THREAT_LEVEL_ENUM_END=3, /*  | */
} MAV_COLLISION_THREAT_LEVEL;
#endif

/** @brief Source of information about this collision. */
#ifndef HAVE_ENUM_MAV_COLLISION_SRC
#define HAVE_ENUM_MAV_COLLISION_SRC
typedef enum MAV_COLLISION_SRC
{
   MAV_COLLISION_SRC_ADSB=0, /* ID field references ADSB_VEHICLE packets | */
   MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT=1, /* ID field references MAVLink SRC ID | */
   MAV_COLLISION_SRC_ENUM_END=2, /*  | */
} MAV_COLLISION_SRC;
#endif

/** @brief Type of GPS fix */
#ifndef HAVE_ENUM_GPS_FIX_TYPE
#define HAVE_ENUM_GPS_FIX_TYPE
typedef enum GPS_FIX_TYPE
{
   GPS_FIX_TYPE_NO_GPS=0, /* No GPS connected | */
   GPS_FIX_TYPE_NO_FIX=1, /* No position information, GPS is connected | */
   GPS_FIX_TYPE_2D_FIX=2, /* 2D position | */
   GPS_FIX_TYPE_3D_FIX=3, /* 3D position | */
   GPS_FIX_TYPE_DGPS=4, /* DGPS/SBAS aided 3D position | */
   GPS_FIX_TYPE_RTK_FLOAT=5, /* RTK float, 3D position | */
   GPS_FIX_TYPE_RTK_FIXED=6, /* RTK Fixed, 3D position | */
   GPS_FIX_TYPE_STATIC=7, /* Static fixed, typically used for base stations | */
   GPS_FIX_TYPE_PPP=8, /* PPP, 3D position. | */
   GPS_FIX_TYPE_ENUM_END=9, /*  | */
} GPS_FIX_TYPE;
#endif

/** @brief RTK GPS baseline coordinate system, used for RTK corrections */
#ifndef HAVE_ENUM_RTK_BASELINE_COORDINATE_SYSTEM
#define HAVE_ENUM_RTK_BASELINE_COORDINATE_SYSTEM
typedef enum RTK_BASELINE_COORDINATE_SYSTEM
{
   RTK_BASELINE_COORDINATE_SYSTEM_ECEF=0, /* Earth-centered, Earth-fixed | */
   RTK_BASELINE_COORDINATE_SYSTEM_NED=1, /* RTK basestation centered, north, east, down | */
   RTK_BASELINE_COORDINATE_SYSTEM_ENUM_END=2, /*  | */
} RTK_BASELINE_COORDINATE_SYSTEM;
#endif

/** @brief Type of landing target */
#ifndef HAVE_ENUM_LANDING_TARGET_TYPE
#define HAVE_ENUM_LANDING_TARGET_TYPE
typedef enum LANDING_TARGET_TYPE
{
   LANDING_TARGET_TYPE_LIGHT_BEACON=0, /* Landing target signaled by light beacon (ex: IR-LOCK) | */
   LANDING_TARGET_TYPE_RADIO_BEACON=1, /* Landing target signaled by radio beacon (ex: ILS, NDB) | */
   LANDING_TARGET_TYPE_VISION_FIDUCIAL=2, /* Landing target represented by a fiducial marker (ex: ARTag) | */
   LANDING_TARGET_TYPE_VISION_OTHER=3, /* Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square) | */
   LANDING_TARGET_TYPE_ENUM_END=4, /*  | */
} LANDING_TARGET_TYPE;
#endif

/** @brief Direction of VTOL transition */
#ifndef HAVE_ENUM_VTOL_TRANSITION_HEADING
#define HAVE_ENUM_VTOL_TRANSITION_HEADING
typedef enum VTOL_TRANSITION_HEADING
{
   VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT=0, /* Respect the heading configuration of the vehicle. | */
   VTOL_TRANSITION_HEADING_NEXT_WAYPOINT=1, /* Use the heading pointing towards the next waypoint. | */
   VTOL_TRANSITION_HEADING_TAKEOFF=2, /* Use the heading on takeoff (while sitting on the ground). | */
   VTOL_TRANSITION_HEADING_SPECIFIED=3, /* Use the specified heading in parameter 4. | */
   VTOL_TRANSITION_HEADING_ANY=4, /* Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active). | */
   VTOL_TRANSITION_HEADING_ENUM_END=5, /*  | */
} VTOL_TRANSITION_HEADING;
#endif

/** @brief Camera capability flags (Bitmap) */
#ifndef HAVE_ENUM_CAMERA_CAP_FLAGS
#define HAVE_ENUM_CAMERA_CAP_FLAGS
typedef enum CAMERA_CAP_FLAGS
{
   CAMERA_CAP_FLAGS_CAPTURE_VIDEO=1, /* Camera is able to record video | */
   CAMERA_CAP_FLAGS_CAPTURE_IMAGE=2, /* Camera is able to capture images | */
   CAMERA_CAP_FLAGS_HAS_MODES=4, /* Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE) | */
   CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE=8, /* Camera can capture images while in video mode | */
   CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE=16, /* Camera can capture videos while in Photo/Image mode | */
   CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE=32, /* Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE) | */
   CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM=64, /* Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM) | */
   CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS=128, /* Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS) | */
   CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM=256, /* Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info) | */
   CAMERA_CAP_FLAGS_HAS_TRACKING_POINT=512, /* Camera supports tracking of a point on the camera view. | */
   CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE=1024, /* Camera supports tracking of a selection rectangle on the camera view. | */
   CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS=2048, /* Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS). | */
   CAMERA_CAP_FLAGS_HAS_THERMAL_RANGE=4096, /* Camera supports absolute thermal range (request CAMERA_THERMAL_RANGE with MAV_CMD_REQUEST_MESSAGE). | */
   CAMERA_CAP_FLAGS_ENUM_END=4097, /*  | */
} CAMERA_CAP_FLAGS;
#endif

/** @brief Stream status flags (Bitmap) */
#ifndef HAVE_ENUM_VIDEO_STREAM_STATUS_FLAGS
#define HAVE_ENUM_VIDEO_STREAM_STATUS_FLAGS
typedef enum VIDEO_STREAM_STATUS_FLAGS
{
   VIDEO_STREAM_STATUS_FLAGS_RUNNING=1, /* Stream is active (running) | */
   VIDEO_STREAM_STATUS_FLAGS_THERMAL=2, /* Stream is thermal imaging | */
   VIDEO_STREAM_STATUS_FLAGS_THERMAL_RANGE_ENABLED=4, /* Stream can report absolute thermal range (see CAMERA_THERMAL_RANGE). | */
   VIDEO_STREAM_STATUS_FLAGS_ENUM_END=5, /*  | */
} VIDEO_STREAM_STATUS_FLAGS;
#endif

/** @brief Video stream types */
#ifndef HAVE_ENUM_VIDEO_STREAM_TYPE
#define HAVE_ENUM_VIDEO_STREAM_TYPE
typedef enum VIDEO_STREAM_TYPE
{
   VIDEO_STREAM_TYPE_RTSP=0, /* Stream is RTSP | */
   VIDEO_STREAM_TYPE_RTPUDP=1, /* Stream is RTP UDP (URI gives the port number) | */
   VIDEO_STREAM_TYPE_TCP_MPEG=2, /* Stream is MPEG on TCP | */
   VIDEO_STREAM_TYPE_MPEG_TS=3, /* Stream is MPEG TS (URI gives the port number) | */
   VIDEO_STREAM_TYPE_ENUM_END=4, /*  | */
} VIDEO_STREAM_TYPE;
#endif

/** @brief Video stream encodings */
#ifndef HAVE_ENUM_VIDEO_STREAM_ENCODING
#define HAVE_ENUM_VIDEO_STREAM_ENCODING
typedef enum VIDEO_STREAM_ENCODING
{
   VIDEO_STREAM_ENCODING_UNKNOWN=0, /* Stream encoding is unknown | */
   VIDEO_STREAM_ENCODING_H264=1, /* Stream encoding is H.264 | */
   VIDEO_STREAM_ENCODING_H265=2, /* Stream encoding is H.265 | */
   VIDEO_STREAM_ENCODING_ENUM_END=3, /*  | */
} VIDEO_STREAM_ENCODING;
#endif

/** @brief Camera tracking status flags */
#ifndef HAVE_ENUM_CAMERA_TRACKING_STATUS_FLAGS
#define HAVE_ENUM_CAMERA_TRACKING_STATUS_FLAGS
typedef enum CAMERA_TRACKING_STATUS_FLAGS
{
   CAMERA_TRACKING_STATUS_FLAGS_IDLE=0, /* Camera is not tracking | */
   CAMERA_TRACKING_STATUS_FLAGS_ACTIVE=1, /* Camera is tracking | */
   CAMERA_TRACKING_STATUS_FLAGS_ERROR=2, /* Camera tracking in error state | */
   CAMERA_TRACKING_STATUS_FLAGS_ENUM_END=3, /*  | */
} CAMERA_TRACKING_STATUS_FLAGS;
#endif

/** @brief Camera tracking modes */
#ifndef HAVE_ENUM_CAMERA_TRACKING_MODE
#define HAVE_ENUM_CAMERA_TRACKING_MODE
typedef enum CAMERA_TRACKING_MODE
{
   CAMERA_TRACKING_MODE_NONE=0, /* Not tracking | */
   CAMERA_TRACKING_MODE_POINT=1, /* Target is a point | */
   CAMERA_TRACKING_MODE_RECTANGLE=2, /* Target is a rectangle | */
   CAMERA_TRACKING_MODE_ENUM_END=3, /*  | */
} CAMERA_TRACKING_MODE;
#endif

/** @brief Camera tracking target data (shows where tracked target is within image) */
#ifndef HAVE_ENUM_CAMERA_TRACKING_TARGET_DATA
#define HAVE_ENUM_CAMERA_TRACKING_TARGET_DATA
typedef enum CAMERA_TRACKING_TARGET_DATA
{
   CAMERA_TRACKING_TARGET_DATA_EMBEDDED=1, /* Target data embedded in image data (proprietary) | */
   CAMERA_TRACKING_TARGET_DATA_RENDERED=2, /* Target data rendered in image | */
   CAMERA_TRACKING_TARGET_DATA_IN_STATUS=4, /* Target data within status message (Point or Rectangle) | */
   CAMERA_TRACKING_TARGET_DATA_ENUM_END=5, /*  | */
} CAMERA_TRACKING_TARGET_DATA;
#endif

/** @brief Zoom types for MAV_CMD_SET_CAMERA_ZOOM */
#ifndef HAVE_ENUM_CAMERA_ZOOM_TYPE
#define HAVE_ENUM_CAMERA_ZOOM_TYPE
typedef enum CAMERA_ZOOM_TYPE
{
   ZOOM_TYPE_STEP=0, /* Zoom one step increment (-1 for wide, 1 for tele) | */
   ZOOM_TYPE_CONTINUOUS=1, /* Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming) | */
   ZOOM_TYPE_RANGE=2, /* Zoom value as proportion of full camera range (a percentage value between 0.0 and 100.0) | */
   ZOOM_TYPE_FOCAL_LENGTH=3, /* Zoom value/variable focal length in millimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera) | */
   CAMERA_ZOOM_TYPE_ENUM_END=4, /*  | */
} CAMERA_ZOOM_TYPE;
#endif

/** @brief Focus types for MAV_CMD_SET_CAMERA_FOCUS */
#ifndef HAVE_ENUM_SET_FOCUS_TYPE
#define HAVE_ENUM_SET_FOCUS_TYPE
typedef enum SET_FOCUS_TYPE
{
   FOCUS_TYPE_STEP=0, /* Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity). | */
   FOCUS_TYPE_CONTINUOUS=1, /* Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing) | */
   FOCUS_TYPE_RANGE=2, /* Focus value as proportion of full camera focus range (a value between 0.0 and 100.0) | */
   FOCUS_TYPE_METERS=3, /* Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera). | */
   FOCUS_TYPE_AUTO=4, /* Focus automatically. | */
   FOCUS_TYPE_AUTO_SINGLE=5, /* Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S. | */
   FOCUS_TYPE_AUTO_CONTINUOUS=6, /* Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C. | */
   SET_FOCUS_TYPE_ENUM_END=7, /*  | */
} SET_FOCUS_TYPE;
#endif

/** @brief Camera sources for MAV_CMD_SET_CAMERA_SOURCE */
#ifndef HAVE_ENUM_CAMERA_SOURCE
#define HAVE_ENUM_CAMERA_SOURCE
typedef enum CAMERA_SOURCE
{
   CAMERA_SOURCE_DEFAULT=0, /* Default camera source. | */
   CAMERA_SOURCE_RGB=1, /* RGB camera source. | */
   CAMERA_SOURCE_IR=2, /* IR camera source. | */
   CAMERA_SOURCE_NDVI=3, /* NDVI camera source. | */
   CAMERA_SOURCE_ENUM_END=4, /*  | */
} CAMERA_SOURCE;
#endif

/** @brief Result from PARAM_EXT_SET message. */
#ifndef HAVE_ENUM_PARAM_ACK
#define HAVE_ENUM_PARAM_ACK
typedef enum PARAM_ACK
{
   PARAM_ACK_ACCEPTED=0, /* Parameter value ACCEPTED and SET | */
   PARAM_ACK_VALUE_UNSUPPORTED=1, /* Parameter value UNKNOWN/UNSUPPORTED | */
   PARAM_ACK_FAILED=2, /* Parameter failed to set | */
   PARAM_ACK_IN_PROGRESS=3, /* Parameter value received but not yet set/accepted. A subsequent PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating that the the parameter was received and does not need to be resent. | */
   PARAM_ACK_ENUM_END=4, /*  | */
} PARAM_ACK;
#endif

/** @brief Camera Modes. */
#ifndef HAVE_ENUM_CAMERA_MODE
#define HAVE_ENUM_CAMERA_MODE
typedef enum CAMERA_MODE
{
   CAMERA_MODE_IMAGE=0, /* Camera is in image/photo capture mode. | */
   CAMERA_MODE_VIDEO=1, /* Camera is in video capture mode. | */
   CAMERA_MODE_IMAGE_SURVEY=2, /* Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys. | */
   CAMERA_MODE_ENUM_END=3, /*  | */
} CAMERA_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ARM_AUTH_DENIED_REASON
#define HAVE_ENUM_MAV_ARM_AUTH_DENIED_REASON
typedef enum MAV_ARM_AUTH_DENIED_REASON
{
   MAV_ARM_AUTH_DENIED_REASON_GENERIC=0, /* Not a specific reason | */
   MAV_ARM_AUTH_DENIED_REASON_NONE=1, /* Authorizer will send the error as string to GCS | */
   MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT=2, /* At least one waypoint have a invalid value | */
   MAV_ARM_AUTH_DENIED_REASON_TIMEOUT=3, /* Timeout in the authorizer process(in case it depends on network) | */
   MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE=4, /* Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied. | */
   MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER=5, /* Weather is not good to fly | */
   MAV_ARM_AUTH_DENIED_REASON_ENUM_END=6, /*  | */
} MAV_ARM_AUTH_DENIED_REASON;
#endif

/** @brief RC type */
#ifndef HAVE_ENUM_RC_TYPE
#define HAVE_ENUM_RC_TYPE
typedef enum RC_TYPE
{
   RC_TYPE_SPEKTRUM_DSM2=0, /* Spektrum DSM2 | */
   RC_TYPE_SPEKTRUM_DSMX=1, /* Spektrum DSMX | */
   RC_TYPE_ENUM_END=2, /*  | */
} RC_TYPE;
#endif

/** @brief Engine control options */
#ifndef HAVE_ENUM_ENGINE_CONTROL_OPTIONS
#define HAVE_ENUM_ENGINE_CONTROL_OPTIONS
typedef enum ENGINE_CONTROL_OPTIONS
{
   ENGINE_CONTROL_OPTIONS_ALLOW_START_WHILE_DISARMED=1, /* Allow starting the engine once while disarmed | */
   ENGINE_CONTROL_OPTIONS_ENUM_END=2, /*  | */
} ENGINE_CONTROL_OPTIONS;
#endif

/** @brief Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration. */
#ifndef HAVE_ENUM_POSITION_TARGET_TYPEMASK
#define HAVE_ENUM_POSITION_TARGET_TYPEMASK
typedef enum POSITION_TARGET_TYPEMASK
{
   POSITION_TARGET_TYPEMASK_X_IGNORE=1, /* Ignore position x | */
   POSITION_TARGET_TYPEMASK_Y_IGNORE=2, /* Ignore position y | */
   POSITION_TARGET_TYPEMASK_Z_IGNORE=4, /* Ignore position z | */
   POSITION_TARGET_TYPEMASK_VX_IGNORE=8, /* Ignore velocity x | */
   POSITION_TARGET_TYPEMASK_VY_IGNORE=16, /* Ignore velocity y | */
   POSITION_TARGET_TYPEMASK_VZ_IGNORE=32, /* Ignore velocity z | */
   POSITION_TARGET_TYPEMASK_AX_IGNORE=64, /* Ignore acceleration x | */
   POSITION_TARGET_TYPEMASK_AY_IGNORE=128, /* Ignore acceleration y | */
   POSITION_TARGET_TYPEMASK_AZ_IGNORE=256, /* Ignore acceleration z | */
   POSITION_TARGET_TYPEMASK_FORCE_SET=512, /* Use force instead of acceleration | */
   POSITION_TARGET_TYPEMASK_YAW_IGNORE=1024, /* Ignore yaw | */
   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=2048, /* Ignore yaw rate | */
   POSITION_TARGET_TYPEMASK_ENUM_END=2049, /*  | */
} POSITION_TARGET_TYPEMASK;
#endif

/** @brief Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored. */
#ifndef HAVE_ENUM_ATTITUDE_TARGET_TYPEMASK
#define HAVE_ENUM_ATTITUDE_TARGET_TYPEMASK
typedef enum ATTITUDE_TARGET_TYPEMASK
{
   ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE=1, /* Ignore body roll rate | */
   ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE=2, /* Ignore body pitch rate | */
   ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE=4, /* Ignore body yaw rate | */
   ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE=64, /* Ignore throttle | */
   ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE=128, /* Ignore attitude | */
   ATTITUDE_TARGET_TYPEMASK_ENUM_END=129, /*  | */
} ATTITUDE_TARGET_TYPEMASK;
#endif

/** @brief Airborne status of UAS. */
#ifndef HAVE_ENUM_UTM_FLIGHT_STATE
#define HAVE_ENUM_UTM_FLIGHT_STATE
typedef enum UTM_FLIGHT_STATE
{
   UTM_FLIGHT_STATE_UNKNOWN=1, /* The flight state can't be determined. | */
   UTM_FLIGHT_STATE_GROUND=2, /* UAS on ground. | */
   UTM_FLIGHT_STATE_AIRBORNE=3, /* UAS airborne. | */
   UTM_FLIGHT_STATE_EMERGENCY=16, /* UAS is in an emergency flight state. | */
   UTM_FLIGHT_STATE_NOCTRL=32, /* UAS has no active controls. | */
   UTM_FLIGHT_STATE_ENUM_END=33, /*  | */
} UTM_FLIGHT_STATE;
#endif

/** @brief Flags for the global position report. */
#ifndef HAVE_ENUM_UTM_DATA_AVAIL_FLAGS
#define HAVE_ENUM_UTM_DATA_AVAIL_FLAGS
typedef enum UTM_DATA_AVAIL_FLAGS
{
   UTM_DATA_AVAIL_FLAGS_TIME_VALID=1, /* The field time contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE=2, /* The field uas_id contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE=4, /* The fields lat, lon and h_acc contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE=8, /* The fields alt and v_acc contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE=16, /* The field relative_alt contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE=32, /* The fields vx and vy contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE=64, /* The field vz contains valid data. | */
   UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE=128, /* The fields next_lat, next_lon and next_alt contain valid data. | */
   UTM_DATA_AVAIL_FLAGS_ENUM_END=129, /*  | */
} UTM_DATA_AVAIL_FLAGS;
#endif

/** @brief Precision land modes (used in MAV_CMD_NAV_LAND). */
#ifndef HAVE_ENUM_PRECISION_LAND_MODE
#define HAVE_ENUM_PRECISION_LAND_MODE
typedef enum PRECISION_LAND_MODE
{
   PRECISION_LAND_MODE_DISABLED=0, /* Normal (non-precision) landing. | */
   PRECISION_LAND_MODE_OPPORTUNISTIC=1, /* Use precision landing if beacon detected when land command accepted, otherwise land normally. | */
   PRECISION_LAND_MODE_REQUIRED=2, /* Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found). | */
   PRECISION_LAND_MODE_ENUM_END=3, /*  | */
} PRECISION_LAND_MODE;
#endif

/** @brief Parachute actions. Trigger release and enable/disable auto-release. */
#ifndef HAVE_ENUM_PARACHUTE_ACTION
#define HAVE_ENUM_PARACHUTE_ACTION
typedef enum PARACHUTE_ACTION
{
   PARACHUTE_DISABLE=0, /* Disable auto-release of parachute (i.e. release triggered by crash detectors). | */
   PARACHUTE_ENABLE=1, /* Enable auto-release of parachute. | */
   PARACHUTE_RELEASE=2, /* Release parachute and kill motors. | */
   PARACHUTE_ACTION_ENUM_END=3, /*  | */
} PARACHUTE_ACTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_TUNNEL_PAYLOAD_TYPE
#define HAVE_ENUM_MAV_TUNNEL_PAYLOAD_TYPE
typedef enum MAV_TUNNEL_PAYLOAD_TYPE
{
   MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN=0, /* Encoding of payload unknown. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0=200, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1=201, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2=202, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3=203, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4=204, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5=205, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6=206, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7=207, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8=208, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9=209, /* Registered for STorM32 gimbal controller. | */
   MAV_TUNNEL_PAYLOAD_TYPE_ENUM_END=210, /*  | */
} MAV_TUNNEL_PAYLOAD_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_ID_TYPE
#define HAVE_ENUM_MAV_ODID_ID_TYPE
typedef enum MAV_ODID_ID_TYPE
{
   MAV_ODID_ID_TYPE_NONE=0, /* No type defined. | */
   MAV_ODID_ID_TYPE_SERIAL_NUMBER=1, /* Manufacturer Serial Number (ANSI/CTA-2063 format). | */
   MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID=2, /* CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID]. | */
   MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID=3, /* UTM (Unmanned Traffic Management) assigned UUID (RFC4122). | */
   MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID=4, /* A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id and these type values are managed by ICAO. | */
   MAV_ODID_ID_TYPE_ENUM_END=5, /*  | */
} MAV_ODID_ID_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_UA_TYPE
#define HAVE_ENUM_MAV_ODID_UA_TYPE
typedef enum MAV_ODID_UA_TYPE
{
   MAV_ODID_UA_TYPE_NONE=0, /* No UA (Unmanned Aircraft) type defined. | */
   MAV_ODID_UA_TYPE_AEROPLANE=1, /* Aeroplane/Airplane. Fixed wing. | */
   MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR=2, /* Helicopter or multirotor. | */
   MAV_ODID_UA_TYPE_GYROPLANE=3, /* Gyroplane. | */
   MAV_ODID_UA_TYPE_HYBRID_LIFT=4, /* VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically. | */
   MAV_ODID_UA_TYPE_ORNITHOPTER=5, /* Ornithopter. | */
   MAV_ODID_UA_TYPE_GLIDER=6, /* Glider. | */
   MAV_ODID_UA_TYPE_KITE=7, /* Kite. | */
   MAV_ODID_UA_TYPE_FREE_BALLOON=8, /* Free Balloon. | */
   MAV_ODID_UA_TYPE_CAPTIVE_BALLOON=9, /* Captive Balloon. | */
   MAV_ODID_UA_TYPE_AIRSHIP=10, /* Airship. E.g. a blimp. | */
   MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE=11, /* Free Fall/Parachute (unpowered). | */
   MAV_ODID_UA_TYPE_ROCKET=12, /* Rocket. | */
   MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT=13, /* Tethered powered aircraft. | */
   MAV_ODID_UA_TYPE_GROUND_OBSTACLE=14, /* Ground Obstacle. | */
   MAV_ODID_UA_TYPE_OTHER=15, /* Other type of aircraft not listed earlier. | */
   MAV_ODID_UA_TYPE_ENUM_END=16, /*  | */
} MAV_ODID_UA_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_STATUS
#define HAVE_ENUM_MAV_ODID_STATUS
typedef enum MAV_ODID_STATUS
{
   MAV_ODID_STATUS_UNDECLARED=0, /* The status of the (UA) Unmanned Aircraft is undefined. | */
   MAV_ODID_STATUS_GROUND=1, /* The UA is on the ground. | */
   MAV_ODID_STATUS_AIRBORNE=2, /* The UA is in the air. | */
   MAV_ODID_STATUS_EMERGENCY=3, /* The UA is having an emergency. | */
   MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE=4, /* The remote ID system is failing or unreliable in some way. | */
   MAV_ODID_STATUS_ENUM_END=5, /*  | */
} MAV_ODID_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_HEIGHT_REF
#define HAVE_ENUM_MAV_ODID_HEIGHT_REF
typedef enum MAV_ODID_HEIGHT_REF
{
   MAV_ODID_HEIGHT_REF_OVER_TAKEOFF=0, /* The height field is relative to the take-off location. | */
   MAV_ODID_HEIGHT_REF_OVER_GROUND=1, /* The height field is relative to ground. | */
   MAV_ODID_HEIGHT_REF_ENUM_END=2, /*  | */
} MAV_ODID_HEIGHT_REF;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_HOR_ACC
#define HAVE_ENUM_MAV_ODID_HOR_ACC
typedef enum MAV_ODID_HOR_ACC
{
   MAV_ODID_HOR_ACC_UNKNOWN=0, /* The horizontal accuracy is unknown. | */
   MAV_ODID_HOR_ACC_10NM=1, /* The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km. | */
   MAV_ODID_HOR_ACC_4NM=2, /* The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km. | */
   MAV_ODID_HOR_ACC_2NM=3, /* The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km. | */
   MAV_ODID_HOR_ACC_1NM=4, /* The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km. | */
   MAV_ODID_HOR_ACC_0_5NM=5, /* The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m. | */
   MAV_ODID_HOR_ACC_0_3NM=6, /* The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m. | */
   MAV_ODID_HOR_ACC_0_1NM=7, /* The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m. | */
   MAV_ODID_HOR_ACC_0_05NM=8, /* The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m. | */
   MAV_ODID_HOR_ACC_30_METER=9, /* The horizontal accuracy is smaller than 30 meter. | */
   MAV_ODID_HOR_ACC_10_METER=10, /* The horizontal accuracy is smaller than 10 meter. | */
   MAV_ODID_HOR_ACC_3_METER=11, /* The horizontal accuracy is smaller than 3 meter. | */
   MAV_ODID_HOR_ACC_1_METER=12, /* The horizontal accuracy is smaller than 1 meter. | */
   MAV_ODID_HOR_ACC_ENUM_END=13, /*  | */
} MAV_ODID_HOR_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_VER_ACC
#define HAVE_ENUM_MAV_ODID_VER_ACC
typedef enum MAV_ODID_VER_ACC
{
   MAV_ODID_VER_ACC_UNKNOWN=0, /* The vertical accuracy is unknown. | */
   MAV_ODID_VER_ACC_150_METER=1, /* The vertical accuracy is smaller than 150 meter. | */
   MAV_ODID_VER_ACC_45_METER=2, /* The vertical accuracy is smaller than 45 meter. | */
   MAV_ODID_VER_ACC_25_METER=3, /* The vertical accuracy is smaller than 25 meter. | */
   MAV_ODID_VER_ACC_10_METER=4, /* The vertical accuracy is smaller than 10 meter. | */
   MAV_ODID_VER_ACC_3_METER=5, /* The vertical accuracy is smaller than 3 meter. | */
   MAV_ODID_VER_ACC_1_METER=6, /* The vertical accuracy is smaller than 1 meter. | */
   MAV_ODID_VER_ACC_ENUM_END=7, /*  | */
} MAV_ODID_VER_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_SPEED_ACC
#define HAVE_ENUM_MAV_ODID_SPEED_ACC
typedef enum MAV_ODID_SPEED_ACC
{
   MAV_ODID_SPEED_ACC_UNKNOWN=0, /* The speed accuracy is unknown. | */
   MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND=1, /* The speed accuracy is smaller than 10 meters per second. | */
   MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND=2, /* The speed accuracy is smaller than 3 meters per second. | */
   MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND=3, /* The speed accuracy is smaller than 1 meters per second. | */
   MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND=4, /* The speed accuracy is smaller than 0.3 meters per second. | */
   MAV_ODID_SPEED_ACC_ENUM_END=5, /*  | */
} MAV_ODID_SPEED_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_TIME_ACC
#define HAVE_ENUM_MAV_ODID_TIME_ACC
typedef enum MAV_ODID_TIME_ACC
{
   MAV_ODID_TIME_ACC_UNKNOWN=0, /* The timestamp accuracy is unknown. | */
   MAV_ODID_TIME_ACC_0_1_SECOND=1, /* The timestamp accuracy is smaller than or equal to 0.1 second. | */
   MAV_ODID_TIME_ACC_0_2_SECOND=2, /* The timestamp accuracy is smaller than or equal to 0.2 second. | */
   MAV_ODID_TIME_ACC_0_3_SECOND=3, /* The timestamp accuracy is smaller than or equal to 0.3 second. | */
   MAV_ODID_TIME_ACC_0_4_SECOND=4, /* The timestamp accuracy is smaller than or equal to 0.4 second. | */
   MAV_ODID_TIME_ACC_0_5_SECOND=5, /* The timestamp accuracy is smaller than or equal to 0.5 second. | */
   MAV_ODID_TIME_ACC_0_6_SECOND=6, /* The timestamp accuracy is smaller than or equal to 0.6 second. | */
   MAV_ODID_TIME_ACC_0_7_SECOND=7, /* The timestamp accuracy is smaller than or equal to 0.7 second. | */
   MAV_ODID_TIME_ACC_0_8_SECOND=8, /* The timestamp accuracy is smaller than or equal to 0.8 second. | */
   MAV_ODID_TIME_ACC_0_9_SECOND=9, /* The timestamp accuracy is smaller than or equal to 0.9 second. | */
   MAV_ODID_TIME_ACC_1_0_SECOND=10, /* The timestamp accuracy is smaller than or equal to 1.0 second. | */
   MAV_ODID_TIME_ACC_1_1_SECOND=11, /* The timestamp accuracy is smaller than or equal to 1.1 second. | */
   MAV_ODID_TIME_ACC_1_2_SECOND=12, /* The timestamp accuracy is smaller than or equal to 1.2 second. | */
   MAV_ODID_TIME_ACC_1_3_SECOND=13, /* The timestamp accuracy is smaller than or equal to 1.3 second. | */
   MAV_ODID_TIME_ACC_1_4_SECOND=14, /* The timestamp accuracy is smaller than or equal to 1.4 second. | */
   MAV_ODID_TIME_ACC_1_5_SECOND=15, /* The timestamp accuracy is smaller than or equal to 1.5 second. | */
   MAV_ODID_TIME_ACC_ENUM_END=16, /*  | */
} MAV_ODID_TIME_ACC;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_AUTH_TYPE
#define HAVE_ENUM_MAV_ODID_AUTH_TYPE
typedef enum MAV_ODID_AUTH_TYPE
{
   MAV_ODID_AUTH_TYPE_NONE=0, /* No authentication type is specified. | */
   MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE=1, /* Signature for the UAS (Unmanned Aircraft System) ID. | */
   MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE=2, /* Signature for the Operator ID. | */
   MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE=3, /* Signature for the entire message set. | */
   MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID=4, /* Authentication is provided by Network Remote ID. | */
   MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION=5, /* The exact authentication type is indicated by the first byte of authentication_data and these type values are managed by ICAO. | */
   MAV_ODID_AUTH_TYPE_ENUM_END=6, /*  | */
} MAV_ODID_AUTH_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_DESC_TYPE
#define HAVE_ENUM_MAV_ODID_DESC_TYPE
typedef enum MAV_ODID_DESC_TYPE
{
   MAV_ODID_DESC_TYPE_TEXT=0, /* Free-form text description of the purpose of the flight. | */
   MAV_ODID_DESC_TYPE_EMERGENCY=1, /* Optional additional clarification when status == MAV_ODID_STATUS_EMERGENCY. | */
   MAV_ODID_DESC_TYPE_EXTENDED_STATUS=2, /* Optional additional clarification when status != MAV_ODID_STATUS_EMERGENCY. | */
   MAV_ODID_DESC_TYPE_ENUM_END=3, /*  | */
} MAV_ODID_DESC_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_OPERATOR_LOCATION_TYPE
#define HAVE_ENUM_MAV_ODID_OPERATOR_LOCATION_TYPE
typedef enum MAV_ODID_OPERATOR_LOCATION_TYPE
{
   MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF=0, /* The location of the operator is the same as the take-off location. | */
   MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS=1, /* The location of the operator is based on live GNSS data. | */
   MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED=2, /* The location of the operator is a fixed location. | */
   MAV_ODID_OPERATOR_LOCATION_TYPE_ENUM_END=3, /*  | */
} MAV_ODID_OPERATOR_LOCATION_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_CLASSIFICATION_TYPE
#define HAVE_ENUM_MAV_ODID_CLASSIFICATION_TYPE
typedef enum MAV_ODID_CLASSIFICATION_TYPE
{
   MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED=0, /* The classification type for the UA is undeclared. | */
   MAV_ODID_CLASSIFICATION_TYPE_EU=1, /* The classification type for the UA follows EU (European Union) specifications. | */
   MAV_ODID_CLASSIFICATION_TYPE_ENUM_END=2, /*  | */
} MAV_ODID_CLASSIFICATION_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_CATEGORY_EU
#define HAVE_ENUM_MAV_ODID_CATEGORY_EU
typedef enum MAV_ODID_CATEGORY_EU
{
   MAV_ODID_CATEGORY_EU_UNDECLARED=0, /* The category for the UA, according to the EU specification, is undeclared. | */
   MAV_ODID_CATEGORY_EU_OPEN=1, /* The category for the UA, according to the EU specification, is the Open category. | */
   MAV_ODID_CATEGORY_EU_SPECIFIC=2, /* The category for the UA, according to the EU specification, is the Specific category. | */
   MAV_ODID_CATEGORY_EU_CERTIFIED=3, /* The category for the UA, according to the EU specification, is the Certified category. | */
   MAV_ODID_CATEGORY_EU_ENUM_END=4, /*  | */
} MAV_ODID_CATEGORY_EU;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_CLASS_EU
#define HAVE_ENUM_MAV_ODID_CLASS_EU
typedef enum MAV_ODID_CLASS_EU
{
   MAV_ODID_CLASS_EU_UNDECLARED=0, /* The class for the UA, according to the EU specification, is undeclared. | */
   MAV_ODID_CLASS_EU_CLASS_0=1, /* The class for the UA, according to the EU specification, is Class 0. | */
   MAV_ODID_CLASS_EU_CLASS_1=2, /* The class for the UA, according to the EU specification, is Class 1. | */
   MAV_ODID_CLASS_EU_CLASS_2=3, /* The class for the UA, according to the EU specification, is Class 2. | */
   MAV_ODID_CLASS_EU_CLASS_3=4, /* The class for the UA, according to the EU specification, is Class 3. | */
   MAV_ODID_CLASS_EU_CLASS_4=5, /* The class for the UA, according to the EU specification, is Class 4. | */
   MAV_ODID_CLASS_EU_CLASS_5=6, /* The class for the UA, according to the EU specification, is Class 5. | */
   MAV_ODID_CLASS_EU_CLASS_6=7, /* The class for the UA, according to the EU specification, is Class 6. | */
   MAV_ODID_CLASS_EU_ENUM_END=8, /*  | */
} MAV_ODID_CLASS_EU;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_OPERATOR_ID_TYPE
#define HAVE_ENUM_MAV_ODID_OPERATOR_ID_TYPE
typedef enum MAV_ODID_OPERATOR_ID_TYPE
{
   MAV_ODID_OPERATOR_ID_TYPE_CAA=0, /* CAA (Civil Aviation Authority) registered operator ID. | */
   MAV_ODID_OPERATOR_ID_TYPE_ENUM_END=1, /*  | */
} MAV_ODID_OPERATOR_ID_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_ODID_ARM_STATUS
#define HAVE_ENUM_MAV_ODID_ARM_STATUS
typedef enum MAV_ODID_ARM_STATUS
{
   MAV_ODID_ARM_STATUS_GOOD_TO_ARM=0, /* Passing arming checks. | */
   MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC=1, /* Generic arming failure, see error string for details. | */
   MAV_ODID_ARM_STATUS_ENUM_END=2, /*  | */
} MAV_ODID_ARM_STATUS;
#endif

/** @brief Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html */
#ifndef HAVE_ENUM_AIS_TYPE
#define HAVE_ENUM_AIS_TYPE
typedef enum AIS_TYPE
{
   AIS_TYPE_UNKNOWN=0, /* Not available (default). | */
   AIS_TYPE_RESERVED_1=1, /*  | */
   AIS_TYPE_RESERVED_2=2, /*  | */
   AIS_TYPE_RESERVED_3=3, /*  | */
   AIS_TYPE_RESERVED_4=4, /*  | */
   AIS_TYPE_RESERVED_5=5, /*  | */
   AIS_TYPE_RESERVED_6=6, /*  | */
   AIS_TYPE_RESERVED_7=7, /*  | */
   AIS_TYPE_RESERVED_8=8, /*  | */
   AIS_TYPE_RESERVED_9=9, /*  | */
   AIS_TYPE_RESERVED_10=10, /*  | */
   AIS_TYPE_RESERVED_11=11, /*  | */
   AIS_TYPE_RESERVED_12=12, /*  | */
   AIS_TYPE_RESERVED_13=13, /*  | */
   AIS_TYPE_RESERVED_14=14, /*  | */
   AIS_TYPE_RESERVED_15=15, /*  | */
   AIS_TYPE_RESERVED_16=16, /*  | */
   AIS_TYPE_RESERVED_17=17, /*  | */
   AIS_TYPE_RESERVED_18=18, /*  | */
   AIS_TYPE_RESERVED_19=19, /*  | */
   AIS_TYPE_WIG=20, /* Wing In Ground effect. | */
   AIS_TYPE_WIG_HAZARDOUS_A=21, /*  | */
   AIS_TYPE_WIG_HAZARDOUS_B=22, /*  | */
   AIS_TYPE_WIG_HAZARDOUS_C=23, /*  | */
   AIS_TYPE_WIG_HAZARDOUS_D=24, /*  | */
   AIS_TYPE_WIG_RESERVED_1=25, /*  | */
   AIS_TYPE_WIG_RESERVED_2=26, /*  | */
   AIS_TYPE_WIG_RESERVED_3=27, /*  | */
   AIS_TYPE_WIG_RESERVED_4=28, /*  | */
   AIS_TYPE_WIG_RESERVED_5=29, /*  | */
   AIS_TYPE_FISHING=30, /*  | */
   AIS_TYPE_TOWING=31, /*  | */
   AIS_TYPE_TOWING_LARGE=32, /* Towing: length exceeds 200m or breadth exceeds 25m. | */
   AIS_TYPE_DREDGING=33, /* Dredging or other underwater ops. | */
   AIS_TYPE_DIVING=34, /*  | */
   AIS_TYPE_MILITARY=35, /*  | */
   AIS_TYPE_SAILING=36, /*  | */
   AIS_TYPE_PLEASURE=37, /*  | */
   AIS_TYPE_RESERVED_20=38, /*  | */
   AIS_TYPE_RESERVED_21=39, /*  | */
   AIS_TYPE_HSC=40, /* High Speed Craft. | */
   AIS_TYPE_HSC_HAZARDOUS_A=41, /*  | */
   AIS_TYPE_HSC_HAZARDOUS_B=42, /*  | */
   AIS_TYPE_HSC_HAZARDOUS_C=43, /*  | */
   AIS_TYPE_HSC_HAZARDOUS_D=44, /*  | */
   AIS_TYPE_HSC_RESERVED_1=45, /*  | */
   AIS_TYPE_HSC_RESERVED_2=46, /*  | */
   AIS_TYPE_HSC_RESERVED_3=47, /*  | */
   AIS_TYPE_HSC_RESERVED_4=48, /*  | */
   AIS_TYPE_HSC_UNKNOWN=49, /*  | */
   AIS_TYPE_PILOT=50, /*  | */
   AIS_TYPE_SAR=51, /* Search And Rescue vessel. | */
   AIS_TYPE_TUG=52, /*  | */
   AIS_TYPE_PORT_TENDER=53, /*  | */
   AIS_TYPE_ANTI_POLLUTION=54, /* Anti-pollution equipment. | */
   AIS_TYPE_LAW_ENFORCEMENT=55, /*  | */
   AIS_TYPE_SPARE_LOCAL_1=56, /*  | */
   AIS_TYPE_SPARE_LOCAL_2=57, /*  | */
   AIS_TYPE_MEDICAL_TRANSPORT=58, /*  | */
   AIS_TYPE_NONECOMBATANT=59, /* Noncombatant ship according to RR Resolution No. 18. | */
   AIS_TYPE_PASSENGER=60, /*  | */
   AIS_TYPE_PASSENGER_HAZARDOUS_A=61, /*  | */
   AIS_TYPE_PASSENGER_HAZARDOUS_B=62, /*  | */
   AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C=63, /*  | */
   AIS_TYPE_PASSENGER_HAZARDOUS_D=64, /*  | */
   AIS_TYPE_PASSENGER_RESERVED_1=65, /*  | */
   AIS_TYPE_PASSENGER_RESERVED_2=66, /*  | */
   AIS_TYPE_PASSENGER_RESERVED_3=67, /*  | */
   AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4=68, /*  | */
   AIS_TYPE_PASSENGER_UNKNOWN=69, /*  | */
   AIS_TYPE_CARGO=70, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_A=71, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_B=72, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_C=73, /*  | */
   AIS_TYPE_CARGO_HAZARDOUS_D=74, /*  | */
   AIS_TYPE_CARGO_RESERVED_1=75, /*  | */
   AIS_TYPE_CARGO_RESERVED_2=76, /*  | */
   AIS_TYPE_CARGO_RESERVED_3=77, /*  | */
   AIS_TYPE_CARGO_RESERVED_4=78, /*  | */
   AIS_TYPE_CARGO_UNKNOWN=79, /*  | */
   AIS_TYPE_TANKER=80, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_A=81, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_B=82, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_C=83, /*  | */
   AIS_TYPE_TANKER_HAZARDOUS_D=84, /*  | */
   AIS_TYPE_TANKER_RESERVED_1=85, /*  | */
   AIS_TYPE_TANKER_RESERVED_2=86, /*  | */
   AIS_TYPE_TANKER_RESERVED_3=87, /*  | */
   AIS_TYPE_TANKER_RESERVED_4=88, /*  | */
   AIS_TYPE_TANKER_UNKNOWN=89, /*  | */
   AIS_TYPE_OTHER=90, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_A=91, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_B=92, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_C=93, /*  | */
   AIS_TYPE_OTHER_HAZARDOUS_D=94, /*  | */
   AIS_TYPE_OTHER_RESERVED_1=95, /*  | */
   AIS_TYPE_OTHER_RESERVED_2=96, /*  | */
   AIS_TYPE_OTHER_RESERVED_3=97, /*  | */
   AIS_TYPE_OTHER_RESERVED_4=98, /*  | */
   AIS_TYPE_OTHER_UNKNOWN=99, /*  | */
   AIS_TYPE_ENUM_END=100, /*  | */
} AIS_TYPE;
#endif

/** @brief Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html */
#ifndef HAVE_ENUM_AIS_NAV_STATUS
#define HAVE_ENUM_AIS_NAV_STATUS
typedef enum AIS_NAV_STATUS
{
   UNDER_WAY=0, /* Under way using engine. | */
   AIS_NAV_ANCHORED=1, /*  | */
   AIS_NAV_UN_COMMANDED=2, /*  | */
   AIS_NAV_RESTRICTED_MANOEUVERABILITY=3, /*  | */
   AIS_NAV_DRAUGHT_CONSTRAINED=4, /*  | */
   AIS_NAV_MOORED=5, /*  | */
   AIS_NAV_AGROUND=6, /*  | */
   AIS_NAV_FISHING=7, /*  | */
   AIS_NAV_SAILING=8, /*  | */
   AIS_NAV_RESERVED_HSC=9, /*  | */
   AIS_NAV_RESERVED_WIG=10, /*  | */
   AIS_NAV_RESERVED_1=11, /*  | */
   AIS_NAV_RESERVED_2=12, /*  | */
   AIS_NAV_RESERVED_3=13, /*  | */
   AIS_NAV_AIS_SART=14, /* Search And Rescue Transponder. | */
   AIS_NAV_UNKNOWN=15, /* Not available (default). | */
   AIS_NAV_STATUS_ENUM_END=16, /*  | */
} AIS_NAV_STATUS;
#endif

/** @brief These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid. */
#ifndef HAVE_ENUM_AIS_FLAGS
#define HAVE_ENUM_AIS_FLAGS
typedef enum AIS_FLAGS
{
   AIS_FLAGS_POSITION_ACCURACY=1, /* 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m. | */
   AIS_FLAGS_VALID_COG=2, /*  | */
   AIS_FLAGS_VALID_VELOCITY=4, /*  | */
   AIS_FLAGS_HIGH_VELOCITY=8, /* 1 = Velocity over 52.5765m/s (102.2 knots) | */
   AIS_FLAGS_VALID_TURN_RATE=16, /*  | */
   AIS_FLAGS_TURN_RATE_SIGN_ONLY=32, /* Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s | */
   AIS_FLAGS_VALID_DIMENSIONS=64, /*  | */
   AIS_FLAGS_LARGE_BOW_DIMENSION=128, /* Distance to bow is larger than 511m | */
   AIS_FLAGS_LARGE_STERN_DIMENSION=256, /* Distance to stern is larger than 511m | */
   AIS_FLAGS_LARGE_PORT_DIMENSION=512, /* Distance to port side is larger than 63m | */
   AIS_FLAGS_LARGE_STARBOARD_DIMENSION=1024, /* Distance to starboard side is larger than 63m | */
   AIS_FLAGS_VALID_CALLSIGN=2048, /*  | */
   AIS_FLAGS_VALID_NAME=4096, /*  | */
   AIS_FLAGS_ENUM_END=4097, /*  | */
} AIS_FLAGS;
#endif

/** @brief Winch status flags used in WINCH_STATUS */
#ifndef HAVE_ENUM_MAV_WINCH_STATUS_FLAG
#define HAVE_ENUM_MAV_WINCH_STATUS_FLAG
typedef enum MAV_WINCH_STATUS_FLAG
{
   MAV_WINCH_STATUS_HEALTHY=1, /* Winch is healthy | */
   MAV_WINCH_STATUS_FULLY_RETRACTED=2, /* Winch thread is fully retracted | */
   MAV_WINCH_STATUS_MOVING=4, /* Winch motor is moving | */
   MAV_WINCH_STATUS_CLUTCH_ENGAGED=8, /* Winch clutch is engaged allowing motor to move freely | */
   MAV_WINCH_STATUS_FLAG_ENUM_END=9, /*  | */
} MAV_WINCH_STATUS_FLAG;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAG_CAL_STATUS
#define HAVE_ENUM_MAG_CAL_STATUS
typedef enum MAG_CAL_STATUS
{
   MAG_CAL_NOT_STARTED=0, /*  | */
   MAG_CAL_WAITING_TO_START=1, /*  | */
   MAG_CAL_RUNNING_STEP_ONE=2, /*  | */
   MAG_CAL_RUNNING_STEP_TWO=3, /*  | */
   MAG_CAL_SUCCESS=4, /*  | */
   MAG_CAL_FAILED=5, /*  | */
   MAG_CAL_BAD_ORIENTATION=6, /*  | */
   MAG_CAL_BAD_RADIUS=7, /*  | */
   MAG_CAL_STATUS_ENUM_END=8, /*  | */
} MAG_CAL_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CAN_FILTER_OP
#define HAVE_ENUM_CAN_FILTER_OP
typedef enum CAN_FILTER_OP
{
   CAN_FILTER_REPLACE=0, /*  | */
   CAN_FILTER_ADD=1, /*  | */
   CAN_FILTER_REMOVE=2, /*  | */
   CAN_FILTER_OP_ENUM_END=3, /*  | */
} CAN_FILTER_OP;
#endif

/** @brief  */
#ifndef HAVE_ENUM_NAV_VTOL_LAND_OPTIONS
#define HAVE_ENUM_NAV_VTOL_LAND_OPTIONS
typedef enum NAV_VTOL_LAND_OPTIONS
{
   NAV_VTOL_LAND_OPTIONS_DEFAULT=0, /* Default autopilot landing behaviour. | */
   NAV_VTOL_LAND_OPTIONS_FW_SPIRAL_APPROACH=1, /* Use a fixed wing spiral desent approach before landing. | */
   NAV_VTOL_LAND_OPTIONS_FW_APPROACH=2, /* Use a fixed wing approach before detransitioning and landing vertically. | */
   NAV_VTOL_LAND_OPTIONS_ENUM_END=3, /*  | */
} NAV_VTOL_LAND_OPTIONS;
#endif

/** @brief 
        States of the mission state machine.
        Note that these states are independent of whether the mission is in a mode that can execute mission items or not (is suspended).
        They may not all be relevant on all vehicles.
       */
#ifndef HAVE_ENUM_MISSION_STATE
#define HAVE_ENUM_MISSION_STATE
typedef enum MISSION_STATE
{
   MISSION_STATE_UNKNOWN=0, /* The mission status reporting is not supported. | */
   MISSION_STATE_NO_MISSION=1, /* No mission on the vehicle. | */
   MISSION_STATE_NOT_STARTED=2, /* Mission has not started. This is the case after a mission has uploaded but not yet started executing. | */
   MISSION_STATE_ACTIVE=3, /* Mission is active, and will execute mission items when in auto mode. | */
   MISSION_STATE_PAUSED=4, /* Mission is paused when in auto mode. | */
   MISSION_STATE_COMPLETE=5, /* Mission has executed all mission items. | */
   MISSION_STATE_ENUM_END=6, /*  | */
} MISSION_STATE;
#endif

/** @brief 
	Possible safety switch states.
       */
#ifndef HAVE_ENUM_SAFETY_SWITCH_STATE
#define HAVE_ENUM_SAFETY_SWITCH_STATE
typedef enum SAFETY_SWITCH_STATE
{
   SAFETY_SWITCH_STATE_SAFE=0, /* Safety switch is engaged and vehicle should be safe to approach. | */
   SAFETY_SWITCH_STATE_DANGEROUS=1, /* Safety switch is NOT engaged and motors, propellers and other actuators should be considered active. | */
   SAFETY_SWITCH_STATE_ENUM_END=2, /*  | */
} SAFETY_SWITCH_STATE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_sys_status.h"
#include "./mavlink_msg_system_time.h"
#include "./mavlink_msg_ping.h"
#include "./mavlink_msg_change_operator_control.h"
#include "./mavlink_msg_change_operator_control_ack.h"
#include "./mavlink_msg_auth_key.h"
#include "./mavlink_msg_set_mode.h"
#include "./mavlink_msg_param_request_read.h"
#include "./mavlink_msg_param_request_list.h"
#include "./mavlink_msg_param_value.h"
#include "./mavlink_msg_param_set.h"
#include "./mavlink_msg_gps_raw_int.h"
#include "./mavlink_msg_gps_status.h"
#include "./mavlink_msg_scaled_imu.h"
#include "./mavlink_msg_raw_imu.h"
#include "./mavlink_msg_raw_pressure.h"
#include "./mavlink_msg_scaled_pressure.h"
#include "./mavlink_msg_attitude.h"
#include "./mavlink_msg_attitude_quaternion.h"
#include "./mavlink_msg_local_position_ned.h"
#include "./mavlink_msg_global_position_int.h"
#include "./mavlink_msg_rc_channels_scaled.h"
#include "./mavlink_msg_rc_channels_raw.h"
#include "./mavlink_msg_servo_output_raw.h"
#include "./mavlink_msg_mission_request_partial_list.h"
#include "./mavlink_msg_mission_write_partial_list.h"
#include "./mavlink_msg_mission_item.h"
#include "./mavlink_msg_mission_request.h"
#include "./mavlink_msg_mission_set_current.h"
#include "./mavlink_msg_mission_current.h"
#include "./mavlink_msg_mission_request_list.h"
#include "./mavlink_msg_mission_count.h"
#include "./mavlink_msg_mission_clear_all.h"
#include "./mavlink_msg_mission_item_reached.h"
#include "./mavlink_msg_mission_ack.h"
#include "./mavlink_msg_set_gps_global_origin.h"
#include "./mavlink_msg_gps_global_origin.h"
#include "./mavlink_msg_param_map_rc.h"
#include "./mavlink_msg_mission_request_int.h"
#include "./mavlink_msg_safety_set_allowed_area.h"
#include "./mavlink_msg_safety_allowed_area.h"
#include "./mavlink_msg_attitude_quaternion_cov.h"
#include "./mavlink_msg_nav_controller_output.h"
#include "./mavlink_msg_global_position_int_cov.h"
#include "./mavlink_msg_local_position_ned_cov.h"
#include "./mavlink_msg_rc_channels.h"
#include "./mavlink_msg_request_data_stream.h"
#include "./mavlink_msg_data_stream.h"
#include "./mavlink_msg_manual_control.h"
#include "./mavlink_msg_rc_channels_override.h"
#include "./mavlink_msg_mission_item_int.h"
#include "./mavlink_msg_vfr_hud.h"
#include "./mavlink_msg_command_int.h"
#include "./mavlink_msg_command_long.h"
#include "./mavlink_msg_command_ack.h"
#include "./mavlink_msg_manual_setpoint.h"
#include "./mavlink_msg_set_attitude_target.h"
#include "./mavlink_msg_attitude_target.h"
#include "./mavlink_msg_set_position_target_local_ned.h"
#include "./mavlink_msg_position_target_local_ned.h"
#include "./mavlink_msg_set_position_target_global_int.h"
#include "./mavlink_msg_position_target_global_int.h"
#include "./mavlink_msg_local_position_ned_system_global_offset.h"
#include "./mavlink_msg_hil_state.h"
#include "./mavlink_msg_hil_controls.h"
#include "./mavlink_msg_hil_rc_inputs_raw.h"
#include "./mavlink_msg_hil_actuator_controls.h"
#include "./mavlink_msg_optical_flow.h"
#include "./mavlink_msg_global_vision_position_estimate.h"
#include "./mavlink_msg_vision_position_estimate.h"
#include "./mavlink_msg_vision_speed_estimate.h"
#include "./mavlink_msg_vicon_position_estimate.h"
#include "./mavlink_msg_highres_imu.h"
#include "./mavlink_msg_optical_flow_rad.h"
#include "./mavlink_msg_hil_sensor.h"
#include "./mavlink_msg_sim_state.h"
#include "./mavlink_msg_radio_status.h"
#include "./mavlink_msg_file_transfer_protocol.h"
#include "./mavlink_msg_timesync.h"
#include "./mavlink_msg_camera_trigger.h"
#include "./mavlink_msg_hil_gps.h"
#include "./mavlink_msg_hil_optical_flow.h"
#include "./mavlink_msg_hil_state_quaternion.h"
#include "./mavlink_msg_scaled_imu2.h"
#include "./mavlink_msg_log_request_list.h"
#include "./mavlink_msg_log_entry.h"
#include "./mavlink_msg_log_request_data.h"
#include "./mavlink_msg_log_data.h"
#include "./mavlink_msg_log_erase.h"
#include "./mavlink_msg_log_request_end.h"
#include "./mavlink_msg_gps_inject_data.h"
#include "./mavlink_msg_gps2_raw.h"
#include "./mavlink_msg_power_status.h"
#include "./mavlink_msg_serial_control.h"
#include "./mavlink_msg_gps_rtk.h"
#include "./mavlink_msg_gps2_rtk.h"
#include "./mavlink_msg_scaled_imu3.h"
#include "./mavlink_msg_data_transmission_handshake.h"
#include "./mavlink_msg_encapsulated_data.h"
#include "./mavlink_msg_distance_sensor.h"
#include "./mavlink_msg_terrain_request.h"
#include "./mavlink_msg_terrain_data.h"
#include "./mavlink_msg_terrain_check.h"
#include "./mavlink_msg_terrain_report.h"
#include "./mavlink_msg_scaled_pressure2.h"
#include "./mavlink_msg_att_pos_mocap.h"
#include "./mavlink_msg_set_actuator_control_target.h"
#include "./mavlink_msg_actuator_control_target.h"
#include "./mavlink_msg_altitude.h"
#include "./mavlink_msg_resource_request.h"
#include "./mavlink_msg_scaled_pressure3.h"
#include "./mavlink_msg_follow_target.h"
#include "./mavlink_msg_control_system_state.h"
#include "./mavlink_msg_battery_status.h"
#include "./mavlink_msg_autopilot_version.h"
#include "./mavlink_msg_landing_target.h"
#include "./mavlink_msg_fence_status.h"
#include "./mavlink_msg_mag_cal_report.h"
#include "./mavlink_msg_efi_status.h"
#include "./mavlink_msg_estimator_status.h"
#include "./mavlink_msg_wind_cov.h"
#include "./mavlink_msg_gps_input.h"
#include "./mavlink_msg_gps_rtcm_data.h"
#include "./mavlink_msg_high_latency.h"
#include "./mavlink_msg_high_latency2.h"
#include "./mavlink_msg_vibration.h"
#include "./mavlink_msg_home_position.h"
#include "./mavlink_msg_set_home_position.h"
#include "./mavlink_msg_message_interval.h"
#include "./mavlink_msg_extended_sys_state.h"
#include "./mavlink_msg_adsb_vehicle.h"
#include "./mavlink_msg_collision.h"
#include "./mavlink_msg_v2_extension.h"
#include "./mavlink_msg_memory_vect.h"
#include "./mavlink_msg_debug_vect.h"
#include "./mavlink_msg_named_value_float.h"
#include "./mavlink_msg_named_value_int.h"
#include "./mavlink_msg_statustext.h"
#include "./mavlink_msg_debug.h"
#include "./mavlink_msg_setup_signing.h"
#include "./mavlink_msg_button_change.h"
#include "./mavlink_msg_play_tune.h"
#include "./mavlink_msg_camera_information.h"
#include "./mavlink_msg_camera_settings.h"
#include "./mavlink_msg_storage_information.h"
#include "./mavlink_msg_camera_capture_status.h"
#include "./mavlink_msg_camera_image_captured.h"
#include "./mavlink_msg_flight_information.h"
#include "./mavlink_msg_mount_orientation.h"
#include "./mavlink_msg_logging_data.h"
#include "./mavlink_msg_logging_data_acked.h"
#include "./mavlink_msg_logging_ack.h"
#include "./mavlink_msg_video_stream_information.h"
#include "./mavlink_msg_video_stream_status.h"
#include "./mavlink_msg_camera_fov_status.h"
#include "./mavlink_msg_camera_tracking_image_status.h"
#include "./mavlink_msg_camera_tracking_geo_status.h"
#include "./mavlink_msg_camera_thermal_range.h"
#include "./mavlink_msg_gimbal_manager_information.h"
#include "./mavlink_msg_gimbal_manager_status.h"
#include "./mavlink_msg_gimbal_manager_set_attitude.h"
#include "./mavlink_msg_gimbal_device_information.h"
#include "./mavlink_msg_gimbal_device_set_attitude.h"
#include "./mavlink_msg_gimbal_device_attitude_status.h"
#include "./mavlink_msg_autopilot_state_for_gimbal_device.h"
#include "./mavlink_msg_gimbal_manager_set_pitchyaw.h"
#include "./mavlink_msg_gimbal_manager_set_manual_control.h"
#include "./mavlink_msg_wifi_config_ap.h"
#include "./mavlink_msg_ais_vessel.h"
#include "./mavlink_msg_uavcan_node_status.h"
#include "./mavlink_msg_uavcan_node_info.h"
#include "./mavlink_msg_param_ext_request_read.h"
#include "./mavlink_msg_param_ext_request_list.h"
#include "./mavlink_msg_param_ext_value.h"
#include "./mavlink_msg_param_ext_set.h"
#include "./mavlink_msg_param_ext_ack.h"
#include "./mavlink_msg_obstacle_distance.h"
#include "./mavlink_msg_odometry.h"
#include "./mavlink_msg_trajectory_representation_waypoints.h"
#include "./mavlink_msg_trajectory_representation_bezier.h"
#include "./mavlink_msg_isbd_link_status.h"
#include "./mavlink_msg_raw_rpm.h"
#include "./mavlink_msg_utm_global_position.h"
#include "./mavlink_msg_debug_float_array.h"
#include "./mavlink_msg_smart_battery_info.h"
#include "./mavlink_msg_generator_status.h"
#include "./mavlink_msg_actuator_output_status.h"
#include "./mavlink_msg_relay_status.h"
#include "./mavlink_msg_tunnel.h"
#include "./mavlink_msg_can_frame.h"
#include "./mavlink_msg_canfd_frame.h"
#include "./mavlink_msg_can_filter_modify.h"
#include "./mavlink_msg_wheel_distance.h"
#include "./mavlink_msg_winch_status.h"
#include "./mavlink_msg_open_drone_id_basic_id.h"
#include "./mavlink_msg_open_drone_id_location.h"
#include "./mavlink_msg_open_drone_id_authentication.h"
#include "./mavlink_msg_open_drone_id_self_id.h"
#include "./mavlink_msg_open_drone_id_system.h"
#include "./mavlink_msg_open_drone_id_operator_id.h"
#include "./mavlink_msg_open_drone_id_arm_status.h"
#include "./mavlink_msg_open_drone_id_message_pack.h"
#include "./mavlink_msg_open_drone_id_system_update.h"
#include "./mavlink_msg_hygrometer_sensor.h"

// base include
#include "../minimal/minimal.h"


#if MAVLINK_COMMON_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_SYS_STATUS, MAVLINK_MESSAGE_INFO_SYSTEM_TIME, MAVLINK_MESSAGE_INFO_PING, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL_ACK, MAVLINK_MESSAGE_INFO_AUTH_KEY, MAVLINK_MESSAGE_INFO_SET_MODE, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_VALUE, MAVLINK_MESSAGE_INFO_PARAM_SET, MAVLINK_MESSAGE_INFO_GPS_RAW_INT, MAVLINK_MESSAGE_INFO_GPS_STATUS, MAVLINK_MESSAGE_INFO_SCALED_IMU, MAVLINK_MESSAGE_INFO_RAW_IMU, MAVLINK_MESSAGE_INFO_RAW_PRESSURE, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED, MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW, MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_WRITE_PARTIAL_LIST, MAVLINK_MESSAGE_INFO_MISSION_ITEM, MAVLINK_MESSAGE_INFO_MISSION_REQUEST, MAVLINK_MESSAGE_INFO_MISSION_SET_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_CURRENT, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_LIST, MAVLINK_MESSAGE_INFO_MISSION_COUNT, MAVLINK_MESSAGE_INFO_MISSION_CLEAR_ALL, MAVLINK_MESSAGE_INFO_MISSION_ITEM_REACHED, MAVLINK_MESSAGE_INFO_MISSION_ACK, MAVLINK_MESSAGE_INFO_SET_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_GPS_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_PARAM_MAP_RC, MAVLINK_MESSAGE_INFO_MISSION_REQUEST_INT, MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_SAFETY_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION_COV, MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT_COV, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_COV, MAVLINK_MESSAGE_INFO_RC_CHANNELS, MAVLINK_MESSAGE_INFO_REQUEST_DATA_STREAM, MAVLINK_MESSAGE_INFO_DATA_STREAM, MAVLINK_MESSAGE_INFO_MANUAL_CONTROL, MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE, MAVLINK_MESSAGE_INFO_MISSION_ITEM_INT, MAVLINK_MESSAGE_INFO_VFR_HUD, MAVLINK_MESSAGE_INFO_COMMAND_INT, MAVLINK_MESSAGE_INFO_COMMAND_LONG, MAVLINK_MESSAGE_INFO_COMMAND_ACK, MAVLINK_MESSAGE_INFO_MANUAL_SETPOINT, MAVLINK_MESSAGE_INFO_SET_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_ATTITUDE_TARGET, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_POSITION_TARGET_LOCAL_NED, MAVLINK_MESSAGE_INFO_SET_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_POSITION_TARGET_GLOBAL_INT, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, MAVLINK_MESSAGE_INFO_HIL_STATE, MAVLINK_MESSAGE_INFO_HIL_CONTROLS, MAVLINK_MESSAGE_INFO_HIL_RC_INPUTS_RAW, MAVLINK_MESSAGE_INFO_HIL_ACTUATOR_CONTROLS, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_GLOBAL_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_VISION_SPEED_ESTIMATE, MAVLINK_MESSAGE_INFO_VICON_POSITION_ESTIMATE, MAVLINK_MESSAGE_INFO_HIGHRES_IMU, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW_RAD, MAVLINK_MESSAGE_INFO_HIL_SENSOR, MAVLINK_MESSAGE_INFO_SIM_STATE, MAVLINK_MESSAGE_INFO_RADIO_STATUS, MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL, MAVLINK_MESSAGE_INFO_TIMESYNC, MAVLINK_MESSAGE_INFO_CAMERA_TRIGGER, MAVLINK_MESSAGE_INFO_HIL_GPS, MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION, MAVLINK_MESSAGE_INFO_SCALED_IMU2, MAVLINK_MESSAGE_INFO_LOG_REQUEST_LIST, MAVLINK_MESSAGE_INFO_LOG_ENTRY, MAVLINK_MESSAGE_INFO_LOG_REQUEST_DATA, MAVLINK_MESSAGE_INFO_LOG_DATA, MAVLINK_MESSAGE_INFO_LOG_ERASE, MAVLINK_MESSAGE_INFO_LOG_REQUEST_END, MAVLINK_MESSAGE_INFO_GPS_INJECT_DATA, MAVLINK_MESSAGE_INFO_GPS2_RAW, MAVLINK_MESSAGE_INFO_POWER_STATUS, MAVLINK_MESSAGE_INFO_SERIAL_CONTROL, MAVLINK_MESSAGE_INFO_GPS_RTK, MAVLINK_MESSAGE_INFO_GPS2_RTK, MAVLINK_MESSAGE_INFO_SCALED_IMU3, MAVLINK_MESSAGE_INFO_DATA_TRANSMISSION_HANDSHAKE, MAVLINK_MESSAGE_INFO_ENCAPSULATED_DATA, MAVLINK_MESSAGE_INFO_DISTANCE_SENSOR, MAVLINK_MESSAGE_INFO_TERRAIN_REQUEST, MAVLINK_MESSAGE_INFO_TERRAIN_DATA, MAVLINK_MESSAGE_INFO_TERRAIN_CHECK, MAVLINK_MESSAGE_INFO_TERRAIN_REPORT, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE2, MAVLINK_MESSAGE_INFO_ATT_POS_MOCAP, MAVLINK_MESSAGE_INFO_SET_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ACTUATOR_CONTROL_TARGET, MAVLINK_MESSAGE_INFO_ALTITUDE, MAVLINK_MESSAGE_INFO_RESOURCE_REQUEST, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE3, MAVLINK_MESSAGE_INFO_FOLLOW_TARGET, MAVLINK_MESSAGE_INFO_CONTROL_SYSTEM_STATE, MAVLINK_MESSAGE_INFO_BATTERY_STATUS, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION, MAVLINK_MESSAGE_INFO_LANDING_TARGET, MAVLINK_MESSAGE_INFO_FENCE_STATUS, MAVLINK_MESSAGE_INFO_MAG_CAL_REPORT, MAVLINK_MESSAGE_INFO_EFI_STATUS, MAVLINK_MESSAGE_INFO_ESTIMATOR_STATUS, MAVLINK_MESSAGE_INFO_WIND_COV, MAVLINK_MESSAGE_INFO_GPS_INPUT, MAVLINK_MESSAGE_INFO_GPS_RTCM_DATA, MAVLINK_MESSAGE_INFO_HIGH_LATENCY, MAVLINK_MESSAGE_INFO_HIGH_LATENCY2, MAVLINK_MESSAGE_INFO_VIBRATION, MAVLINK_MESSAGE_INFO_HOME_POSITION, MAVLINK_MESSAGE_INFO_SET_HOME_POSITION, MAVLINK_MESSAGE_INFO_MESSAGE_INTERVAL, MAVLINK_MESSAGE_INFO_EXTENDED_SYS_STATE, MAVLINK_MESSAGE_INFO_ADSB_VEHICLE, MAVLINK_MESSAGE_INFO_COLLISION, MAVLINK_MESSAGE_INFO_V2_EXTENSION, MAVLINK_MESSAGE_INFO_MEMORY_VECT, MAVLINK_MESSAGE_INFO_DEBUG_VECT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_FLOAT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_DEBUG, MAVLINK_MESSAGE_INFO_SETUP_SIGNING, MAVLINK_MESSAGE_INFO_BUTTON_CHANGE, MAVLINK_MESSAGE_INFO_PLAY_TUNE, MAVLINK_MESSAGE_INFO_CAMERA_INFORMATION, MAVLINK_MESSAGE_INFO_CAMERA_SETTINGS, MAVLINK_MESSAGE_INFO_STORAGE_INFORMATION, MAVLINK_MESSAGE_INFO_CAMERA_CAPTURE_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_IMAGE_CAPTURED, MAVLINK_MESSAGE_INFO_FLIGHT_INFORMATION, MAVLINK_MESSAGE_INFO_MOUNT_ORIENTATION, MAVLINK_MESSAGE_INFO_LOGGING_DATA, MAVLINK_MESSAGE_INFO_LOGGING_DATA_ACKED, MAVLINK_MESSAGE_INFO_LOGGING_ACK, MAVLINK_MESSAGE_INFO_VIDEO_STREAM_INFORMATION, MAVLINK_MESSAGE_INFO_VIDEO_STREAM_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_FOV_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_TRACKING_IMAGE_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_TRACKING_GEO_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_THERMAL_RANGE, MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_INFORMATION, MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_STATUS, MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_ATTITUDE, MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_INFORMATION, MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_SET_ATTITUDE, MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_ATTITUDE_STATUS, MAVLINK_MESSAGE_INFO_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_PITCHYAW, MAVLINK_MESSAGE_INFO_GIMBAL_MANAGER_SET_MANUAL_CONTROL, MAVLINK_MESSAGE_INFO_WIFI_CONFIG_AP, MAVLINK_MESSAGE_INFO_AIS_VESSEL, MAVLINK_MESSAGE_INFO_UAVCAN_NODE_STATUS, MAVLINK_MESSAGE_INFO_UAVCAN_NODE_INFO, MAVLINK_MESSAGE_INFO_PARAM_EXT_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_EXT_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_EXT_VALUE, MAVLINK_MESSAGE_INFO_PARAM_EXT_SET, MAVLINK_MESSAGE_INFO_PARAM_EXT_ACK, MAVLINK_MESSAGE_INFO_OBSTACLE_DISTANCE, MAVLINK_MESSAGE_INFO_ODOMETRY, MAVLINK_MESSAGE_INFO_TRAJECTORY_REPRESENTATION_WAYPOINTS, MAVLINK_MESSAGE_INFO_TRAJECTORY_REPRESENTATION_BEZIER, MAVLINK_MESSAGE_INFO_ISBD_LINK_STATUS, MAVLINK_MESSAGE_INFO_RAW_RPM, MAVLINK_MESSAGE_INFO_UTM_GLOBAL_POSITION, MAVLINK_MESSAGE_INFO_DEBUG_FLOAT_ARRAY, MAVLINK_MESSAGE_INFO_SMART_BATTERY_INFO, MAVLINK_MESSAGE_INFO_GENERATOR_STATUS, MAVLINK_MESSAGE_INFO_ACTUATOR_OUTPUT_STATUS, MAVLINK_MESSAGE_INFO_RELAY_STATUS, MAVLINK_MESSAGE_INFO_TUNNEL, MAVLINK_MESSAGE_INFO_CAN_FRAME, MAVLINK_MESSAGE_INFO_CANFD_FRAME, MAVLINK_MESSAGE_INFO_CAN_FILTER_MODIFY, MAVLINK_MESSAGE_INFO_WHEEL_DISTANCE, MAVLINK_MESSAGE_INFO_WINCH_STATUS, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_BASIC_ID, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_LOCATION, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_AUTHENTICATION, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SELF_ID, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_OPERATOR_ID, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_MESSAGE_PACK, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_ARM_STATUS, MAVLINK_MESSAGE_INFO_OPEN_DRONE_ID_SYSTEM_UPDATE, MAVLINK_MESSAGE_INFO_HYGROMETER_SENSOR}
# define MAVLINK_MESSAGE_NAMES {{ "ACTUATOR_CONTROL_TARGET", 140 }, { "ACTUATOR_OUTPUT_STATUS", 375 }, { "ADSB_VEHICLE", 246 }, { "AIS_VESSEL", 301 }, { "ALTITUDE", 141 }, { "ATTITUDE", 30 }, { "ATTITUDE_QUATERNION", 31 }, { "ATTITUDE_QUATERNION_COV", 61 }, { "ATTITUDE_TARGET", 83 }, { "ATT_POS_MOCAP", 138 }, { "AUTH_KEY", 7 }, { "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE", 286 }, { "AUTOPILOT_VERSION", 148 }, { "BATTERY_STATUS", 147 }, { "BUTTON_CHANGE", 257 }, { "CAMERA_CAPTURE_STATUS", 262 }, { "CAMERA_FOV_STATUS", 271 }, { "CAMERA_IMAGE_CAPTURED", 263 }, { "CAMERA_INFORMATION", 259 }, { "CAMERA_SETTINGS", 260 }, { "CAMERA_THERMAL_RANGE", 277 }, { "CAMERA_TRACKING_GEO_STATUS", 276 }, { "CAMERA_TRACKING_IMAGE_STATUS", 275 }, { "CAMERA_TRIGGER", 112 }, { "CANFD_FRAME", 387 }, { "CAN_FILTER_MODIFY", 388 }, { "CAN_FRAME", 386 }, { "CHANGE_OPERATOR_CONTROL", 5 }, { "CHANGE_OPERATOR_CONTROL_ACK", 6 }, { "COLLISION", 247 }, { "COMMAND_ACK", 77 }, { "COMMAND_INT", 75 }, { "COMMAND_LONG", 76 }, { "CONTROL_SYSTEM_STATE", 146 }, { "DATA_STREAM", 67 }, { "DATA_TRANSMISSION_HANDSHAKE", 130 }, { "DEBUG", 254 }, { "DEBUG_FLOAT_ARRAY", 350 }, { "DEBUG_VECT", 250 }, { "DISTANCE_SENSOR", 132 }, { "EFI_STATUS", 225 }, { "ENCAPSULATED_DATA", 131 }, { "ESTIMATOR_STATUS", 230 }, { "EXTENDED_SYS_STATE", 245 }, { "FENCE_STATUS", 162 }, { "FILE_TRANSFER_PROTOCOL", 110 }, { "FLIGHT_INFORMATION", 264 }, { "FOLLOW_TARGET", 144 }, { "GENERATOR_STATUS", 373 }, { "GIMBAL_DEVICE_ATTITUDE_STATUS", 285 }, { "GIMBAL_DEVICE_INFORMATION", 283 }, { "GIMBAL_DEVICE_SET_ATTITUDE", 284 }, { "GIMBAL_MANAGER_INFORMATION", 280 }, { "GIMBAL_MANAGER_SET_ATTITUDE", 282 }, { "GIMBAL_MANAGER_SET_MANUAL_CONTROL", 288 }, { "GIMBAL_MANAGER_SET_PITCHYAW", 287 }, { "GIMBAL_MANAGER_STATUS", 281 }, { "GLOBAL_POSITION_INT", 33 }, { "GLOBAL_POSITION_INT_COV", 63 }, { "GLOBAL_VISION_POSITION_ESTIMATE", 101 }, { "GPS2_RAW", 124 }, { "GPS2_RTK", 128 }, { "GPS_GLOBAL_ORIGIN", 49 }, { "GPS_INJECT_DATA", 123 }, { "GPS_INPUT", 232 }, { "GPS_RAW_INT", 24 }, { "GPS_RTCM_DATA", 233 }, { "GPS_RTK", 127 }, { "GPS_STATUS", 25 }, { "HEARTBEAT", 0 }, { "HIGHRES_IMU", 105 }, { "HIGH_LATENCY", 234 }, { "HIGH_LATENCY2", 235 }, { "HIL_ACTUATOR_CONTROLS", 93 }, { "HIL_CONTROLS", 91 }, { "HIL_GPS", 113 }, { "HIL_OPTICAL_FLOW", 114 }, { "HIL_RC_INPUTS_RAW", 92 }, { "HIL_SENSOR", 107 }, { "HIL_STATE", 90 }, { "HIL_STATE_QUATERNION", 115 }, { "HOME_POSITION", 242 }, { "HYGROMETER_SENSOR", 12920 }, { "ISBD_LINK_STATUS", 335 }, { "LANDING_TARGET", 149 }, { "LOCAL_POSITION_NED", 32 }, { "LOCAL_POSITION_NED_COV", 64 }, { "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", 89 }, { "LOGGING_ACK", 268 }, { "LOGGING_DATA", 266 }, { "LOGGING_DATA_ACKED", 267 }, { "LOG_DATA", 120 }, { "LOG_ENTRY", 118 }, { "LOG_ERASE", 121 }, { "LOG_REQUEST_DATA", 119 }, { "LOG_REQUEST_END", 122 }, { "LOG_REQUEST_LIST", 117 }, { "MAG_CAL_REPORT", 192 }, { "MANUAL_CONTROL", 69 }, { "MANUAL_SETPOINT", 81 }, { "MEMORY_VECT", 249 }, { "MESSAGE_INTERVAL", 244 }, { "MISSION_ACK", 47 }, { "MISSION_CLEAR_ALL", 45 }, { "MISSION_COUNT", 44 }, { "MISSION_CURRENT", 42 }, { "MISSION_ITEM", 39 }, { "MISSION_ITEM_INT", 73 }, { "MISSION_ITEM_REACHED", 46 }, { "MISSION_REQUEST", 40 }, { "MISSION_REQUEST_INT", 51 }, { "MISSION_REQUEST_LIST", 43 }, { "MISSION_REQUEST_PARTIAL_LIST", 37 }, { "MISSION_SET_CURRENT", 41 }, { "MISSION_WRITE_PARTIAL_LIST", 38 }, { "MOUNT_ORIENTATION", 265 }, { "NAMED_VALUE_FLOAT", 251 }, { "NAMED_VALUE_INT", 252 }, { "NAV_CONTROLLER_OUTPUT", 62 }, { "OBSTACLE_DISTANCE", 330 }, { "ODOMETRY", 331 }, { "OPEN_DRONE_ID_ARM_STATUS", 12918 }, { "OPEN_DRONE_ID_AUTHENTICATION", 12902 }, { "OPEN_DRONE_ID_BASIC_ID", 12900 }, { "OPEN_DRONE_ID_LOCATION", 12901 }, { "OPEN_DRONE_ID_MESSAGE_PACK", 12915 }, { "OPEN_DRONE_ID_OPERATOR_ID", 12905 }, { "OPEN_DRONE_ID_SELF_ID", 12903 }, { "OPEN_DRONE_ID_SYSTEM", 12904 }, { "OPEN_DRONE_ID_SYSTEM_UPDATE", 12919 }, { "OPTICAL_FLOW", 100 }, { "OPTICAL_FLOW_RAD", 106 }, { "PARAM_EXT_ACK", 324 }, { "PARAM_EXT_REQUEST_LIST", 321 }, { "PARAM_EXT_REQUEST_READ", 320 }, { "PARAM_EXT_SET", 323 }, { "PARAM_EXT_VALUE", 322 }, { "PARAM_MAP_RC", 50 }, { "PARAM_REQUEST_LIST", 21 }, { "PARAM_REQUEST_READ", 20 }, { "PARAM_SET", 23 }, { "PARAM_VALUE", 22 }, { "PING", 4 }, { "PLAY_TUNE", 258 }, { "POSITION_TARGET_GLOBAL_INT", 87 }, { "POSITION_TARGET_LOCAL_NED", 85 }, { "POWER_STATUS", 125 }, { "RADIO_STATUS", 109 }, { "RAW_IMU", 27 }, { "RAW_PRESSURE", 28 }, { "RAW_RPM", 339 }, { "RC_CHANNELS", 65 }, { "RC_CHANNELS_OVERRIDE", 70 }, { "RC_CHANNELS_RAW", 35 }, { "RC_CHANNELS_SCALED", 34 }, { "RELAY_STATUS", 376 }, { "REQUEST_DATA_STREAM", 66 }, { "RESOURCE_REQUEST", 142 }, { "SAFETY_ALLOWED_AREA", 55 }, { "SAFETY_SET_ALLOWED_AREA", 54 }, { "SCALED_IMU", 26 }, { "SCALED_IMU2", 116 }, { "SCALED_IMU3", 129 }, { "SCALED_PRESSURE", 29 }, { "SCALED_PRESSURE2", 137 }, { "SCALED_PRESSURE3", 143 }, { "SERIAL_CONTROL", 126 }, { "SERVO_OUTPUT_RAW", 36 }, { "SETUP_SIGNING", 256 }, { "SET_ACTUATOR_CONTROL_TARGET", 139 }, { "SET_ATTITUDE_TARGET", 82 }, { "SET_GPS_GLOBAL_ORIGIN", 48 }, { "SET_HOME_POSITION", 243 }, { "SET_MODE", 11 }, { "SET_POSITION_TARGET_GLOBAL_INT", 86 }, { "SET_POSITION_TARGET_LOCAL_NED", 84 }, { "SIM_STATE", 108 }, { "SMART_BATTERY_INFO", 370 }, { "STATUSTEXT", 253 }, { "STORAGE_INFORMATION", 261 }, { "SYSTEM_TIME", 2 }, { "SYS_STATUS", 1 }, { "TERRAIN_CHECK", 135 }, { "TERRAIN_DATA", 134 }, { "TERRAIN_REPORT", 136 }, { "TERRAIN_REQUEST", 133 }, { "TIMESYNC", 111 }, { "TRAJECTORY_REPRESENTATION_BEZIER", 333 }, { "TRAJECTORY_REPRESENTATION_WAYPOINTS", 332 }, { "TUNNEL", 385 }, { "UAVCAN_NODE_INFO", 311 }, { "UAVCAN_NODE_STATUS", 310 }, { "UTM_GLOBAL_POSITION", 340 }, { "V2_EXTENSION", 248 }, { "VFR_HUD", 74 }, { "VIBRATION", 241 }, { "VICON_POSITION_ESTIMATE", 104 }, { "VIDEO_STREAM_INFORMATION", 269 }, { "VIDEO_STREAM_STATUS", 270 }, { "VISION_POSITION_ESTIMATE", 102 }, { "VISION_SPEED_ESTIMATE", 103 }, { "WHEEL_DISTANCE", 9000 }, { "WIFI_CONFIG_AP", 299 }, { "WINCH_STATUS", 9005 }, { "WIND_COV", 231 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_COMMON_H
