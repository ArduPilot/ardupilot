#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// Frame types
#define UNDEFINED_FRAME 0
#define MULTICOPTER_FRAME 1
#define HELI_FRAME 2

// Tuning enumeration
enum tuning_func {
    TUNING_NONE =                        0, //
    TUNING_STABILIZE_ROLL_PITCH_KP =     1, // stabilize roll/pitch angle controller's P term
    TUNING_STABILIZE_YAW_KP =            3, // stabilize yaw heading controller's P term
    TUNING_RATE_ROLL_PITCH_KP =          4, // body frame roll/pitch rate controller's P term
    TUNING_RATE_ROLL_PITCH_KI =          5, // body frame roll/pitch rate controller's I term
    TUNING_YAW_RATE_KP =                 6, // body frame yaw rate controller's P term
    TUNING_THROTTLE_RATE_KP =            7, // throttle rate controller's P term (desired rate to acceleration or motor output)
    TUNING_WP_SPEED =                   10, // maximum speed to next way point (0 to 10m/s)
    TUNING_LOITER_POSITION_KP =         12, // loiter distance controller's P term (position error to speed)
    TUNING_HELI_EXTERNAL_GYRO =         13, // TradHeli specific external tail gyro gain
    TUNING_ALTITUDE_HOLD_KP =           14, // altitude hold controller's P term (alt error to desired rate)
    TUNING_RATE_ROLL_PITCH_KD =         21, // body frame roll/pitch rate controller's D term
    TUNING_VEL_XY_KP =                  22, // loiter rate controller's P term (speed error to tilt angle)
    TUNING_ACRO_RP_RATE =               25, // acro controller's desired roll and pitch rate in deg/s
    TUNING_YAW_RATE_KD =                26, // body frame yaw rate controller's D term
    TUNING_VEL_XY_KI =                  28, // loiter rate controller's I term (speed error to tilt angle)
    TUNING_AHRS_YAW_KP =                30, // ahrs's compass effect on yaw angle (0 = very low, 1 = very high)
    TUNING_AHRS_KP =                    31, // accelerometer effect on roll/pitch angle (0=low)
    TUNING_ACCEL_Z_KP =                 34, // accel based throttle controller's P term
    TUNING_ACCEL_Z_KI =                 35, // accel based throttle controller's I term
    TUNING_ACCEL_Z_KD =                 36, // accel based throttle controller's D term
    TUNING_DECLINATION =                38, // compass declination in radians
    TUNING_CIRCLE_RATE =                39, // circle turn rate in degrees (hard coded to about 45 degrees in either direction)
    TUNING_ACRO_YAW_RATE =              40, // acro controller's desired yaw rate in deg/s
    TUNING_RANGEFINDER_GAIN =           41, // unused
    TUNING_EKF_VERTICAL_POS =           42, // unused
    TUNING_EKF_HORIZONTAL_POS =         43, // unused
    TUNING_EKF_ACCEL_NOISE =            44, // unused
    TUNING_RC_FEEL_RP =                 45, // roll-pitch input smoothing
    TUNING_RATE_PITCH_KP =              46, // body frame pitch rate controller's P term
    TUNING_RATE_PITCH_KI =              47, // body frame pitch rate controller's I term
    TUNING_RATE_PITCH_KD =              48, // body frame pitch rate controller's D term
    TUNING_RATE_ROLL_KP =               49, // body frame roll rate controller's P term
    TUNING_RATE_ROLL_KI =               50, // body frame roll rate controller's I term
    TUNING_RATE_ROLL_KD =               51, // body frame roll rate controller's D term
    TUNING_RATE_PITCH_FF =              52, // body frame pitch rate controller FF term
    TUNING_RATE_ROLL_FF =               53, // body frame roll rate controller FF term
    TUNING_RATE_YAW_FF =                54, // body frame yaw rate controller FF term
    TUNING_RATE_MOT_YAW_HEADROOM =      55, // motors yaw headroom minimum
    TUNING_RATE_YAW_FILT =              56, // yaw rate input filter
    UNUSED =                            57, // was winch control
    TUNING_SYSTEM_ID_MAGNITUDE =        58, // magnitude of the system ID signal
    TUNING_POS_CONTROL_ANGLE_MAX =      59  // position controller maximum angle
};

// Yaw behaviours during missions - possible values for WP_YAW_BEHAVIOR parameter
#define WP_YAW_BEHAVIOR_NONE                          0   // auto pilot will never control yaw during missions or rtl (except for DO_CONDITIONAL_YAW command received)
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1   // auto pilot will face next waypoint or home during rtl
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2   // auto pilot will face next waypoint except when doing RTL at which time it will stay in it's last
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3   // auto pilot will look ahead during missions and rtl (primarily meant for traditional helicopters)


// Airmode
enum class AirMode {
    AIRMODE_NONE,
    AIRMODE_DISABLED,
    AIRMODE_ENABLED,
};

enum PayloadPlaceStateType {
    PayloadPlaceStateType_FlyToLocation,
    PayloadPlaceStateType_Descent_Start,
    PayloadPlaceStateType_Descent,
    PayloadPlaceStateType_Release,
    PayloadPlaceStateType_Releasing,
    PayloadPlaceStateType_Delay,
    PayloadPlaceStateType_Ascent_Start,
    PayloadPlaceStateType_Ascent,
    PayloadPlaceStateType_Done,
};

// bit options for DEV_OPTIONS parameter
enum DevOptions {
    DevOptionADSBMAVLink = 1,
    DevOptionVFR_HUDRelativeAlt = 2,
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
     LOG_GUIDED_POSITION_TARGET_MSG,
     LOG_SYSIDD_MSG,
     LOG_SYSIDS_MSG,
     LOG_GUIDED_ATTITUDE_TARGET_MSG
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
#define MASK_LOG_VIDEO_STABILISATION    (1UL<<20)
#define MASK_LOG_FTN_FAST               (1UL<<21)
#define MASK_LOG_ANY                    0xFFFF

// Radio failsafe definitions (FS_THR parameter)
#define FS_THR_DISABLED                            0
#define FS_THR_ENABLED_ALWAYS_RTL                  1
#define FS_THR_ENABLED_CONTINUE_MISSION            2    // Removed in 4.0+, now use fs_options
#define FS_THR_ENABLED_ALWAYS_LAND                 3
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL      4
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND     5
#define FS_THR_ENABLED_AUTO_RTL_OR_RTL             6

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED                        0
#define FS_GCS_ENABLED_ALWAYS_RTL              1
#define FS_GCS_ENABLED_CONTINUE_MISSION        2    // Removed in 4.0+, now use fs_options
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL  3
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND 4
#define FS_GCS_ENABLED_ALWAYS_LAND             5
#define FS_GCS_ENABLED_AUTO_RTL_OR_RTL         6

// EKF failsafe definitions (FS_EKF_ACTION parameter)
#define FS_EKF_ACTION_LAND                  1       // switch to LAND mode on EKF failsafe
#define FS_EKF_ACTION_ALTHOLD               2       // switch to ALTHOLD mode on EKF failsafe
#define FS_EKF_ACTION_LAND_EVEN_STABILIZE   3       // switch to Land mode on EKF failsafe even if in a manual flight mode like stabilize

// for PILOT_THR_BHV parameter
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
