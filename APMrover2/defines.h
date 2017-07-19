#pragma once

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE  1
#define FALSE 0

// Just so that it's completely clear...
#define ENABLED  1
#define DISABLED 0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

#define DEBUG 0
#define SERVO_MAX 4500  // This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// CH 7 control
enum ch7_option {
    CH7_DO_NOTHING = 0,
    CH7_SAVE_WP    = 1
};

// HIL enumerations
#define HIL_MODE_DISABLED 0
#define HIL_MODE_SENSORS  1

// Auto Pilot modes
// ----------------
enum mode {
    MANUAL       = 0,
    LEARNING     = 2,
    STEERING     = 3,
    HOLD         = 4,
    AUTO         = 10,
    RTL          = 11,
    GUIDED       = 15,
    INITIALISING = 16
};

enum GuidedMode {
    Guided_WP,
    Guided_Angle,
    Guided_Velocity
};

// types of failsafe events
#define FAILSAFE_EVENT_THROTTLE (1<<0)
#define FAILSAFE_EVENT_GCS      (1<<1)
#define FAILSAFE_EVENT_RC       (1<<2)

//  Logging parameters
#define LOG_CTUN_MSG            0x01
#define LOG_NTUN_MSG            0x02
#define LOG_PERFORMANCE_MSG     0x03
#define LOG_STARTUP_MSG         0x06
#define LOG_RANGEFINDER_MSG     0x07
#define LOG_ARM_DISARM_MSG      0x08
#define LOG_STEERING_MSG        0x0D
#define LOG_GUIDEDTARGET_MSG    0x0E
#define LOG_WHEELENCODER_MSG    0x0F
#define LOG_ERROR_MSG           0x13

#define TYPE_AIRSTART_MSG       0x00
#define TYPE_GROUNDSTART_MSG    0x01
#define MAX_NUM_LOGS            100

#define MASK_LOG_ATTITUDE_FAST  (1<<0)
#define MASK_LOG_ATTITUDE_MED   (1<<1)
#define MASK_LOG_GPS            (1<<2)
#define MASK_LOG_PM             (1<<3)
#define MASK_LOG_CTUN           (1<<4)
#define MASK_LOG_NTUN           (1<<5)
#define MASK_LOG_MODE           (1<<6)
#define MASK_LOG_IMU            (1<<7)
#define MASK_LOG_CMD            (1<<8)
#define MASK_LOG_CURRENT        (1<<9)
#define MASK_LOG_RANGEFINDER    (1<<10)
#define MASK_LOG_COMPASS        (1<<11)
#define MASK_LOG_CAMERA         (1<<12)
#define MASK_LOG_STEERING       (1<<13)
#define MASK_LOG_RC             (1<<14)
#define MASK_LOG_ARM_DISARM     (1<<15)
#define MASK_LOG_IMU_RAW        (1UL<<19)

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

// for mavlink SET_ATTITUDE_TARGET messages
#define MAVLINK_SET_ATT_TYPE_MASK_ROLL_RATE_IGNORE     (1<<0)
#define MAVLINK_SET_ATT_TYPE_MASK_PITCH_RATE_IGNORE    (1<<1)
#define MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE      (1<<2)
#define MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE      (1<<6)
#define MAVLINK_SET_ATT_TYPE_MASK_ATTITUDE_IGNORE      (1<<7)

// Error message sub systems and error codes
#define ERROR_SUBSYSTEM_CRASH_CHECK  12
// subsystem specific error codes -- crash checker
#define ERROR_CODE_CRASH_CHECK_CRASH 1

enum fs_crash_action {
  FS_CRASH_DISABLE = 0,
  FS_CRASH_HOLD = 1,
  FS_CRASH_HOLD_AND_DISARM = 2
};

#define DISTANCE_HOME_MAX 0.5f  // Distance max to home location before changing it when disarm
