//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream

#pragma once

#include "GCS_config.h"

#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_Terrain/AP_Terrain_config.h>

enum ap_message : uint8_t {
    MSG_HEARTBEAT                      =   0,
#if AP_AHRS_ENABLED
    MSG_AHRS                           =   1,
    MSG_AHRS2                          =   2,
    MSG_ATTITUDE                       =   3,
    MSG_ATTITUDE_QUATERNION            =   4,
    MSG_LOCATION                       =   5,
    MSG_VFR_HUD                        =   6,
#endif
    MSG_SYS_STATUS                     =   7,
    MSG_POWER_STATUS                   =   8,
    MSG_MEMINFO                        =   9,
    MSG_NAV_CONTROLLER_OUTPUT          =  10,
    MSG_CURRENT_WAYPOINT               =  11,
    MSG_SERVO_OUTPUT_RAW               =  12,
    MSG_RC_CHANNELS                    =  13,
    MSG_RC_CHANNELS_RAW                =  14,
    MSG_RAW_IMU                        =  15,
    MSG_SCALED_IMU                     =  16,
    MSG_SCALED_IMU2                    =  17,
    MSG_SCALED_IMU3                    =  18,
    MSG_SCALED_PRESSURE                =  19,
    MSG_SCALED_PRESSURE2               =  20,
    MSG_SCALED_PRESSURE3               =  21,
    MSG_GPS_RAW                        =  22,
    MSG_GPS_RTK                        =  23,
    MSG_GPS2_RAW                       =  24,
    MSG_GPS2_RTK                       =  25,
    MSG_SYSTEM_TIME                    =  26,
    MSG_SERVO_OUT                      =  27,
    MSG_NEXT_MISSION_REQUEST_WAYPOINTS =  28,
    MSG_NEXT_MISSION_REQUEST_RALLY     =  29,
    MSG_NEXT_MISSION_REQUEST_FENCE     =  30,
    MSG_NEXT_PARAM                     =  31,
    MSG_FENCE_STATUS                   =  32,
    MSG_SIMSTATE                       =  33,
    MSG_SIM_STATE                      =  34,
#if AP_MAVLINK_MSG_HWSTATUS_ENABLED
    MSG_HWSTATUS                       =  35,
#endif  // AP_MAVLINK_MSG_HWSTATUS_ENABLED
    MSG_WIND                           =  36,
#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
    MSG_RANGEFINDER                    =  37,
#endif  // AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
    MSG_DISTANCE_SENSOR                =  38,
#if AP_TERRAIN_AVAILABLE
    MSG_TERRAIN_REQUEST                =  39,
    MSG_TERRAIN_REPORT                 =  40,
#endif  // AP_TERRAIN_AVAILABLE
    MSG_BATTERY2_UNUSED                =  41,
    MSG_CAMERA_FEEDBACK                =  42,
    MSG_CAMERA_INFORMATION             =  43,
    MSG_CAMERA_SETTINGS                =  44,
    MSG_CAMERA_FOV_STATUS              =  45,
    MSG_CAMERA_CAPTURE_STATUS          =  46,
    MSG_CAMERA_THERMAL_RANGE           =  47,
    MSG_GIMBAL_DEVICE_ATTITUDE_STATUS  =  48,
    MSG_GIMBAL_MANAGER_INFORMATION     =  49,
    MSG_GIMBAL_MANAGER_STATUS          =  50,
    MSG_VIDEO_STREAM_INFORMATION       =  51,
    MSG_OPTICAL_FLOW                   =  52,
    MSG_MAG_CAL_PROGRESS               =  53,
    MSG_MAG_CAL_REPORT                 =  54,
    MSG_EKF_STATUS_REPORT              =  55,
    MSG_LOCAL_POSITION                 =  56,
    MSG_PID_TUNING                     =  57,
    MSG_VIBRATION                      =  58,
    MSG_RPM                            =  59,
    MSG_WHEEL_DISTANCE                 =  60,
    MSG_MISSION_ITEM_REACHED           =  61,
    MSG_POSITION_TARGET_GLOBAL_INT     =  62,
    MSG_POSITION_TARGET_LOCAL_NED      =  63,
    MSG_ADSB_VEHICLE                   =  64,
    MSG_BATTERY_STATUS                 =  65,
    MSG_AOA_SSA                        =  66,
    MSG_LANDING                        =  67,
    MSG_ESC_TELEMETRY                  =  68,
    MSG_ORIGIN                         =  69,
    MSG_HOME                           =  70,
    MSG_NAMED_FLOAT                    =  71,
    MSG_EXTENDED_SYS_STATE             =  72,
    MSG_AUTOPILOT_VERSION              =  73,
    MSG_EFI_STATUS                     =  74,
    MSG_GENERATOR_STATUS               =  75,
    MSG_WINCH_STATUS                   =  76,
    MSG_WATER_DEPTH                    =  77,
    MSG_HIGH_LATENCY2                  =  78,
    MSG_AIS_VESSEL                     =  79,
    MSG_MCU_STATUS                     =  90,
    MSG_UAVIONIX_ADSB_OUT_STATUS       =  91,
    MSG_ATTITUDE_TARGET                =  92,
    MSG_HYGROMETER                     =  93,
    MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE=94,
    MSG_RELAY_STATUS                   =  95,
#if AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
    MSG_HIGHRES_IMU                    =  96,
#endif
    MSG_AIRSPEED                       =  97,
    MSG_AVAILABLE_MODES                =  98,
    MSG_AVAILABLE_MODES_MONITOR        =  99,
#if AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
    MSG_FLIGHT_INFORMATION             = 100,
#endif
    MSG_LAST // MSG_LAST must be the last entry in this enum
};
