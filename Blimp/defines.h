#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// bit options for DEV_OPTIONS parameter
enum DevOptions {
    DevOptionADSBMAVLink = 1,
    DevOptionVFR_HUDRelativeAlt = 2,
    DevOptionSetAttitudeTarget_ThrustAsThrust = 4,
};

//  Logging parameters - only 32 messages are available to the vehicle here.
enum LoggingParameters {
    LOG_CONTROL_TUNING_MSG,
    LOG_DATA_INT16_MSG,
    LOG_DATA_UINT16_MSG,
    LOG_DATA_INT32_MSG,
    LOG_DATA_UINT32_MSG,
    LOG_DATA_FLOAT_MSG,
    LOG_PARAMTUNE_MSG,
    LOG_HELI_MSG,
    LOG_GUIDEDTARGET_MSG,
    LOG_SYSIDD_MSG,
    LOG_SYSIDS_MSG,
    LOG_FINI_MSG,
    LOG_FINO_MSG,
    LOG_PIDD_MSG,
    LOG_PIVN_MSG,
    LOG_PIVE_MSG,
    LOG_PIVD_MSG,
    LOG_PIVY_MSG,

};

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_RCIN                   (1<<6)
#define MASK_LOG_IMU                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CURRENT                (1<<9)
#define MASK_LOG_RCOUT                  (1<<10)
#define MASK_LOG_OPTFLOW                (1<<11)
#define MASK_LOG_PID                    (1<<12)
#define MASK_LOG_COMPASS                (1<<13)
#define MASK_LOG_INAV                   (1<<14) // deprecated
#define MASK_LOG_CAMERA                 (1<<15)
#define MASK_LOG_MOTBATT                (1UL<<17)
#define MASK_LOG_IMU_FAST               (1UL<<18)
#define MASK_LOG_IMU_RAW                (1UL<<19)
#define MASK_LOG_ANY                    0xFFFF

// Radio failsafe definitions (FS_THR parameter)
#define FS_THR_DISABLED                            0
#define FS_THR_ENABLED_ALWAYS_RTL                  1
#define FS_THR_ENABLED_CONTINUE_MISSION            2    // Removed in 4.0+, now use fs_options
#define FS_THR_ENABLED_ALWAYS_LAND                 3
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL      4
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND     5

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED                        0
#define FS_GCS_ENABLED_ALWAYS_RTL              1
#define FS_GCS_ENABLED_CONTINUE_MISSION        2    // Removed in 4.0+, now use fs_options
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL  3
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND 4
#define FS_GCS_ENABLED_ALWAYS_LAND             5

// EKF failsafe definitions (FS_EKF_ACTION parameter)
#define FS_EKF_ACTION_LAND                  1       // switch to LAND mode on EKF failsafe
#define FS_EKF_ACTION_LAND_EVEN_MANUAL      3       // switch to Land mode on EKF failsafe even if in Manual mode

// for PILOT_THR_BHV parameter
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
