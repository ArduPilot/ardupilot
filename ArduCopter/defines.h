#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// Autopilot Yaw Mode enumeration
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =             0,  // pilot controls the heading
    AUTO_YAW_LOOK_AT_NEXT_WP =  1,  // point towards next waypoint (no pilot input accepted)
    AUTO_YAW_ROI =              2,  // point towards a location held in roi (no pilot input accepted)
    AUTO_YAW_FIXED =            3,  // point towards a particular angle (no pilot input accepted)
    AUTO_YAW_LOOK_AHEAD =       4,  // point in the direction the copter is moving
    AUTO_YAW_RESETTOARMEDYAW =  5,  // point towards heading at time motors were armed
    AUTO_YAW_RATE =             6,  // turn at a specified rate (held in auto_yaw_rate)
};

// Frame types
#define UNDEFINED_FRAME 0
#define MULTICOPTER_FRAME 1
#define HELI_FRAME 2

// HIL enumerations
#define HIL_MODE_DISABLED               0
#define HIL_MODE_SENSORS                1

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
    TUNING_ACRO_RP_KP =                 25, // acro controller's P term.  converts pilot input to a desired roll, pitch or yaw rate
    TUNING_YAW_RATE_KD =                26, // body frame yaw rate controller's D term
    TUNING_VEL_XY_KI =                  28, // loiter rate controller's I term (speed error to tilt angle)
    TUNING_AHRS_YAW_KP =                30, // ahrs's compass effect on yaw angle (0 = very low, 1 = very high)
    TUNING_AHRS_KP =                    31, // accelerometer effect on roll/pitch angle (0=low)
    TUNING_ACCEL_Z_KP =                 34, // accel based throttle controller's P term
    TUNING_ACCEL_Z_KI =                 35, // accel based throttle controller's I term
    TUNING_ACCEL_Z_KD =                 36, // accel based throttle controller's D term
    TUNING_DECLINATION =                38, // compass declination in radians
    TUNING_CIRCLE_RATE =                39, // circle turn rate in degrees (hard coded to about 45 degrees in either direction)
    TUNING_ACRO_YAW_KP =                40, // acro controller's P term.  converts pilot input to a desired roll, pitch or yaw rate
    TUNING_RANGEFINDER_GAIN =           41, // rangefinder gain
    TUNING_EKF_VERTICAL_POS =           42, // EKF's baro vs accel (higher rely on accels more, baro impact is reduced).  Range should be 0.2 ~ 4.0?  2.0 is default
    TUNING_EKF_HORIZONTAL_POS =         43, // EKF's gps vs accel (higher rely on accels more, gps impact is reduced).  Range should be 1.0 ~ 3.0?  1.5 is default
    TUNING_EKF_ACCEL_NOISE =            44, // EKF's accel noise (lower means trust accels more, gps & baro less).  Range should be 0.02 ~ 0.5  0.5 is default (but very robust at that level)
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
    TUNING_WINCH =                      57, // winch control (not actually a value to be tuned)
    TUNING_SYSTEM_ID_MAGNITUDE =        58  // magnitude of the system ID signal
};

// Acro Trainer types
#define ACRO_TRAINER_DISABLED   0
#define ACRO_TRAINER_LEVELING   1
#define ACRO_TRAINER_LIMITED    2

// Yaw behaviours during missions - possible values for WP_YAW_BEHAVIOR parameter
#define WP_YAW_BEHAVIOR_NONE                          0   // auto pilot will never control yaw during missions or rtl (except for DO_CONDITIONAL_YAW command received)
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1   // auto pilot will face next waypoint or home during rtl
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2   // auto pilot will face next waypoint except when doing RTL at which time it will stay in it's last
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3   // auto pilot will look ahead during missions and rtl (primarily meant for traditional helicopters)

// Auto modes
enum AutoMode {
    Auto_TakeOff,
    Auto_WP,
    Auto_Land,
    Auto_RTL,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline,
    Auto_NavGuided,
    Auto_Loiter,
    Auto_LoiterToAlt,
    Auto_NavPayloadPlace,
};

// Guided modes
enum GuidedMode {
    Guided_TakeOff,
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel,
    Guided_Angle,
};

// RTL states
enum RTLState {
    RTL_Starting,
    RTL_InitialClimb,
    RTL_ReturnHome,
    RTL_LoiterAtHome,
    RTL_FinalDescent,
    RTL_Land
};

// Safe RTL states
enum SmartRTLState {
    SmartRTL_WaitForPathCleanup,
    SmartRTL_PathFollow,
    SmartRTL_PreLandPosition,
    SmartRTL_Descend,
    SmartRTL_Land
};

enum LandStateType {
    LandStateType_FlyToLocation = 0,
    LandStateType_Descending = 1
};

enum PayloadPlaceStateType {
    PayloadPlaceStateType_FlyToLocation,
    PayloadPlaceStateType_Calibrating_Hover_Start,
    PayloadPlaceStateType_Calibrating_Hover,
    PayloadPlaceStateType_Descending_Start,
    PayloadPlaceStateType_Descending,
    PayloadPlaceStateType_Releasing_Start,
    PayloadPlaceStateType_Releasing,
    PayloadPlaceStateType_Released,
    PayloadPlaceStateType_Ascending_Start,
    PayloadPlaceStateType_Ascending,
    PayloadPlaceStateType_Done,
};

// bit options for DEV_OPTIONS parameter
enum DevOptions {
    DevOptionADSBMAVLink = 1,
    DevOptionVFR_HUDRelativeAlt = 2,
};

//  Logging parameters
enum LoggingParameters {
     LOG_CONTROL_TUNING_MSG,
     LOG_DATA_INT16_MSG,
     LOG_DATA_UINT16_MSG,
     LOG_DATA_INT32_MSG,
     LOG_DATA_UINT32_MSG,
     LOG_DATA_FLOAT_MSG,
     LOG_MOTBATT_MSG,
     LOG_PARAMTUNE_MSG,
     LOG_HELI_MSG,
     LOG_PRECLAND_MSG,
     LOG_GUIDEDTARGET_MSG,
     LOG_SYSIDD_MSG,
     LOG_SYSIDS_MSG,
};

// Harmonic notch update mode
enum HarmonicNotchDynamicMode {
    HarmonicNotch_Fixed,
    HarmonicNotch_UpdateThrottle,
    HarmonicNotch_UpdateRPM,
    HarmonicNotch_UpdateBLHeli,
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
#define FS_THR_ENABLED_CONTINUE_MISSION            2    // Deprecated in 4.0+, now use fs_options
#define FS_THR_ENABLED_ALWAYS_LAND                 3
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL      4
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND     5

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED                        0
#define FS_GCS_ENABLED_ALWAYS_RTL              1
#define FS_GCS_ENABLED_CONTINUE_MISSION        2    // Deprecated in 4.0+, now use fs_options
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL  3
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND 4
#define FS_GCS_ENABLED_ALWAYS_LAND             5

// EKF failsafe definitions (FS_EKF_ACTION parameter)
#define FS_EKF_ACTION_LAND                  1       // switch to LAND mode on EKF failsafe
#define FS_EKF_ACTION_ALTHOLD               2       // switch to ALTHOLD mode on EKF failsafe
#define FS_EKF_ACTION_LAND_EVEN_STABILIZE   3       // switch to Land mode on EKF failsafe even if in a manual flight mode like stabilize

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

// for PILOT_THR_BHV parameter
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
