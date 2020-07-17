#pragma once

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

// Just so that it's completely clear...
#define ENABLED  1
#define DISABLED 0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

#define SERVO_MAX 4500  // This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// HIL enumerations
#define HIL_MODE_DISABLED 0
#define HIL_MODE_SENSORS  1

// types of failsafe events
#define FAILSAFE_EVENT_THROTTLE (1<<0)
#define FAILSAFE_EVENT_GCS      (1<<1)

//  Logging parameters
#define LOG_THR_MSG             0x01
#define LOG_NTUN_MSG            0x02
#define LOG_STARTUP_MSG         0x06
#define LOG_STEERING_MSG        0x0D
#define LOG_GUIDEDTARGET_MSG    0x0E

#define TYPE_GROUNDSTART_MSG    0x01

#define MASK_LOG_ATTITUDE_FAST  (1<<0)
#define MASK_LOG_ATTITUDE_MED   (1<<1)
#define MASK_LOG_GPS            (1<<2)
#define MASK_LOG_PM             (1<<3)
#define MASK_LOG_THR            (1<<4)
#define MASK_LOG_NTUN           (1<<5)
//#define MASK_LOG_MODE         (1<<6) // no longer used
#define MASK_LOG_IMU            (1<<7)
#define MASK_LOG_CMD            (1<<8)
#define MASK_LOG_CURRENT        (1<<9)
#define MASK_LOG_RANGEFINDER    (1<<10)
#define MASK_LOG_COMPASS        (1<<11)
#define MASK_LOG_CAMERA         (1<<12)
#define MASK_LOG_STEERING       (1<<13)
#define MASK_LOG_RC             (1<<14)
// #define MASK_LOG_ARM_DISARM     (1<<15)
#define MASK_LOG_IMU_RAW        (1UL<<19)

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

// for mavlink SET_ATTITUDE_TARGET messages
#define MAVLINK_SET_ATT_TYPE_MASK_ROLL_RATE_IGNORE     (1<<0)
#define MAVLINK_SET_ATT_TYPE_MASK_PITCH_RATE_IGNORE    (1<<1)
#define MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE      (1<<2)
#define MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE      (1<<6)
#define MAVLINK_SET_ATT_TYPE_MASK_ATTITUDE_IGNORE      (1<<7)

// radio failsafe enum (FS_THR_ENABLE parameter)
enum fs_thr_enable {
    FS_THR_DISABLED = 0,
    FS_THR_ENABLED,
    FS_THR_ENABLED_CONTINUE_MISSION,
};

// gcs failsafe enum (FS_GCS_ENABLE parameter)
enum fs_gcs_enable {
    FS_GCS_DISABLED = 0,
    FS_GCS_ENABLED,
    FS_GCS_ENABLED_CONTINUE_MISSION,
};

enum fs_crash_action {
  FS_CRASH_DISABLE = 0,
  FS_CRASH_HOLD = 1,
  FS_CRASH_HOLD_AND_DISARM = 2
};

enum fs_ekf_action {
    FS_EKF_DISABLE = 0,
    FS_EFK_HOLD = 1
};

#define DISTANCE_HOME_MINCHANGE 0.5f  // minimum distance to adjust home location

enum pilot_steer_type_t {
    PILOT_STEER_TYPE_DEFAULT = 0,
    PILOT_STEER_TYPE_TWO_PADDLES = 1,
    PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING = 2,
    PILOT_STEER_TYPE_DIR_UNCHANGED_WHEN_REVERSING = 3,
};

// frame class enum used for FRAME_CLASS parameter
enum frame_class {
    FRAME_UNDEFINED = 0,
    FRAME_ROVER = 1,
    FRAME_BOAT = 2,
    FRAME_BALANCEBOT = 3,
};
