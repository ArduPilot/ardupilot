#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#define BOTTOM_DETECTOR_TRIGGER_SEC 1.0
#define SURFACE_DETECTOR_TRIGGER_SEC 1.0

enum AutoSurfaceState {
    AUTO_SURFACE_STATE_GO_TO_LOCATION,
    AUTO_SURFACE_STATE_ASCEND
};

// Autopilot Yaw Mode enumeration
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =              0,  // pilot controls the heading
    AUTO_YAW_LOOK_AT_NEXT_WP =   1,  // point towards next waypoint (no pilot input accepted)
    AUTO_YAW_ROI =               2,  // point towards a location held in roi_WP (no pilot input accepted)
    AUTO_YAW_LOOK_AT_HEADING =   3,  // point towards a particular angle (not pilot input accepted)
    AUTO_YAW_LOOK_AHEAD =        4,  // point in the direction the vehicle is moving
    AUTO_YAW_RESETTOARMEDYAW =   5,  // point towards heading at time motors were armed
    AUTO_YAW_CORRECT_XTRACK =    6,  // steer the sub in order to correct for crosstrack error during line following
    AUTO_YAW_RATE =              7   // steer the sub with the desired yaw rate 
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
#define WP_YAW_BEHAVIOR_CORRECT_XTRACK                4   // point towards intermediate position target during line following



//  Logging parameters - only 32 messages are available to the vehicle here.
enum LoggingParameters {
    LOG_CONTROL_TUNING_MSG,
    LOG_DATA_INT16_MSG,
    LOG_DATA_UINT16_MSG,
    LOG_DATA_INT32_MSG,
    LOG_DATA_UINT32_MSG,
    LOG_DATA_FLOAT_MSG,
    LOG_GUIDEDTARGET_MSG
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
#define MASK_LOG_CAMERA                 (1<<15)
#define MASK_LOG_MOTBATT                (1UL<<17)
#define MASK_LOG_IMU_FAST               (1UL<<18)
#define MASK_LOG_IMU_RAW                (1UL<<19)
#define MASK_LOG_ANY                    0xFFFF

// GCS failsafe
#ifndef FS_GCS
# define FS_GCS                        0
#endif
#ifndef FS_GCS_TIMEOUT_S
# define FS_GCS_TIMEOUT_S             5.0    // gcs failsafe triggers after this number of seconds with no GCS heartbeat
#endif

// missing terrain data failsafe
#ifndef FS_TERRAIN_TIMEOUT_MS
#define FS_TERRAIN_TIMEOUT_MS          1000     // 1 second of unhealthy rangefinder and/or missing terrain data will trigger failsafe
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
// EKF failsafe definitions (FS_EKF_ENABLE parameter)
#define FS_EKF_ACTION_DISABLED                0       // Disabled
#define FS_EKF_ACTION_WARN_ONLY               1       // Send warning to gcs
#define FS_EKF_ACTION_DISARM                  2       // Disarm


#ifndef FS_EKF_ACTION_DEFAULT
# define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_DISABLED  // EKF failsafe
#endif

#ifndef FS_EKF_THRESHOLD_DEFAULT
# define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED     0 // Disabled
#define FS_GCS_WARN_ONLY    1 // Only send warning to gcs (only useful with multiple gcs links)
#define FS_GCS_DISARM       2 // Disarm
#define FS_GCS_HOLD         3 // Switch depth hold mode or poshold mode if available
#define FS_GCS_SURFACE      4 // Switch to surface mode

// Leak failsafe definitions (FS_LEAK_ENABLE parameter)
#define FS_LEAK_DISABLED    0 // Disabled
#define FS_LEAK_WARN_ONLY   1 // Only send warning to gcs
#define FS_LEAK_SURFACE     2 // Switch to surface mode

// Internal pressure failsafe threshold (FS_PRESS_MAX parameter)
#define FS_PRESS_MAX_DEFAULT    105000 // Maximum internal pressure in pascal before failsafe is triggered
// Internal pressure failsafe definitions (FS_PRESS_ENABLE parameter)
#define FS_PRESS_DISABLED       0
#define FS_PRESS_WARN_ONLY      1

// Internal temperature failsafe threshold (FS_TEMP_MAX parameter)
#define FS_TEMP_MAX_DEFAULT     62  // Maximum internal pressure in degrees C before failsafe is triggered
// Internal temperature failsafe definitions (FS_TEMP_ENABLE parameter)
#define FS_TEMP_DISABLED        0
#define FS_TEMP_WARN_ONLY       1

// Crash failsafe options
#define FS_CRASH_DISABLED  0
#define FS_CRASH_WARN_ONLY 1
#define FS_CRASH_DISARM    2

// Terrain failsafe actions for AUTO mode
#define FS_TERRAIN_DISARM       0
#define FS_TERRAIN_HOLD         1
#define FS_TERRAIN_SURFACE      2

// Pilot input failsafe actions
#define FS_PILOT_INPUT_DISABLED    0
#define FS_PILOT_INPUT_WARN_ONLY   1
#define FS_PILOT_INPUT_DISARM      2

// Amount of time to attempt recovery of valid rangefinder data before
// initiating terrain failsafe action
#define FS_TERRAIN_RECOVER_TIMEOUT_MS 10000

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_Z_IGNORE        (1<<2)
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

