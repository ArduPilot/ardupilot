#pragma once

#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0

#define DEBUG 0
#define SERVO_MAX 4500  // This value represents 45 degrees and is just an
                        // arbitrary representation of servo max travel.

// failsafe
// ----------------------
enum failsafe_state {
    FAILSAFE_NONE=0,
    FAILSAFE_SHORT=1,
    FAILSAFE_LONG=2,
    FAILSAFE_GCS=3
};


// GCS failsafe types for FS_GCS_ENABL parameter
enum gcs_failsafe {
    GCS_FAILSAFE_OFF        = 0, // no GCS failsafe
    GCS_FAILSAFE_HEARTBEAT  = 1, // failsafe if we stop receiving heartbeat
    GCS_FAILSAFE_HB_RSSI    = 2, // failsafe if we stop receiving
                                 // heartbeat or if RADIO.remrssi
                                 // drops to 0
    GCS_FAILSAFE_HB_AUTO    = 3  // failsafe if we stop receiving heartbeat
                                 // while in AUTO mode
};

enum failsafe_action_short {
    FS_ACTION_SHORT_BESTGUESS = 0,      // CIRCLE/no change(if already in AUTO|GUIDED|LOITER)
    FS_ACTION_SHORT_CIRCLE = 1,
    FS_ACTION_SHORT_FBWA = 2,
    FS_ACTION_SHORT_DISABLED = 3,
};

enum failsafe_action_long {
    FS_ACTION_LONG_CONTINUE = 0,
    FS_ACTION_LONG_RTL = 1,
    FS_ACTION_LONG_GLIDE = 2,
    FS_ACTION_LONG_PARACHUTE = 3,
};

// map flight modes to the MAVLink representation
enum FlightMode {
    MANUAL        = PLANE_FLIGHT_MODE_MANUAL,
    CIRCLE        = PLANE_FLIGHT_MODE_CIRCLE,
    STABILIZE     = PLANE_FLIGHT_MODE_STABILIZE,
    TRAINING      = PLANE_FLIGHT_MODE_TRAINING,
    ACRO          = PLANE_FLIGHT_MODE_ACRO,
    FLY_BY_WIRE_A = PLANE_FLIGHT_MODE_FLY_BY_WIRE_A,
    FLY_BY_WIRE_B = PLANE_FLIGHT_MODE_FLY_BY_WIRE_B,
    CRUISE        = PLANE_FLIGHT_MODE_CRUISE,
    AUTOTUNE      = PLANE_FLIGHT_MODE_AUTOTUNE,
    AUTO          = PLANE_FLIGHT_MODE_AUTO,
    RTL           = PLANE_FLIGHT_MODE_RTL,
    LOITER        = PLANE_FLIGHT_MODE_LOITER,
    AVOID_ADSB    = PLANE_FLIGHT_MODE_AVOID_ADSB,
    GUIDED        = PLANE_FLIGHT_MODE_GUIDED,
    INITIALISING  = PLANE_FLIGHT_MODE_INITIALISING,
    QSTABILIZE    = PLANE_FLIGHT_MODE_QSTABILIZE,
    QHOVER        = PLANE_FLIGHT_MODE_QHOVER,
    QLOITER       = PLANE_FLIGHT_MODE_QLOITER,
    QLAND         = PLANE_FLIGHT_MODE_QLAND,
    QRTL          = PLANE_FLIGHT_MODE_QRTL
};

enum mode_reason_t {
    MODE_REASON_UNKNOWN=0,
    MODE_REASON_TX_COMMAND,
    MODE_REASON_GCS_COMMAND,
    MODE_REASON_RADIO_FAILSAFE,
    MODE_REASON_BATTERY_FAILSAFE,
    MODE_REASON_GCS_FAILSAFE,
    MODE_REASON_EKF_FAILSAFE,
    MODE_REASON_GPS_GLITCH,
    MODE_REASON_MISSION_END,
    MODE_REASON_FENCE_BREACH,
    MODE_REASON_AVOIDANCE,
    MODE_REASON_AVOIDANCE_RECOVERY,
    MODE_REASON_SOARING_FBW_B_WITH_MOTOR_RUNNING,
    MODE_REASON_SOARING_THERMAL_DETECTED,
    MODE_REASON_SOARING_IN_THERMAL,
    MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED
};

// type of stick mixing enabled
enum StickMixing {
    STICK_MIXING_DISABLED = 0,
    STICK_MIXING_FBW      = 1,
    STICK_MIXING_DIRECT   = 2
};

enum ChannelMixing {
    MIXING_DISABLED = 0,
    MIXING_UPUP     = 1,
    MIXING_UPDN     = 2,
    MIXING_DNUP     = 3,
    MIXING_DNDN     = 4,
    MIXING_UPUP_SWP = 5,
    MIXING_UPDN_SWP = 6,
    MIXING_DNUP_SWP = 7,
    MIXING_DNDN_SWP = 8,
};

/*
 * The cause for the most recent fence enable
 */
typedef enum GeofenceEnableReason {
    NOT_ENABLED = 0,     //The fence is not enabled
    PWM_TOGGLED,         //Fence enabled/disabled by PWM signal
    AUTO_TOGGLED,        //Fence auto enabled/disabled at takeoff.
    GCS_TOGGLED          //Fence enabled/disabled by the GCS via Mavlink
} GeofenceEnableReason;


// Logging message types
enum log_messages {
    LOG_CTUN_MSG,
    LOG_NTUN_MSG,
    LOG_PERFORMANCE_MSG,
    LOG_STARTUP_MSG,
    TYPE_AIRSTART_MSG,
    TYPE_GROUNDSTART_MSG,
    LOG_RC_MSG,
    LOG_SONAR_MSG,
    LOG_ARM_DISARM_MSG,
    LOG_STATUS_MSG,
    LOG_OPTFLOW_MSG,
    LOG_QTUN_MSG,
    LOG_PARAMTUNE_MSG,
    LOG_THERMAL_MSG,
    LOG_VARIO_MSG,
    LOG_PIQR_MSG,
    LOG_PIQP_MSG,
    LOG_PIQY_MSG,
    LOG_PIQA_MSG,
    LOG_AETR_MSG,
};

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
//#define MASK_LOG_MODE                 (1<<6) // no longer used
#define MASK_LOG_IMU                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CURRENT                (1<<9)
#define MASK_LOG_COMPASS                (1<<10)
#define MASK_LOG_TECS                   (1<<11)
#define MASK_LOG_CAMERA                 (1<<12)
#define MASK_LOG_RC                     (1<<13)
#define MASK_LOG_SONAR                  (1<<14)
#define MASK_LOG_ARM_DISARM             (1<<15)
#define MASK_LOG_IMU_RAW                (1UL<<19)

// altitude control algorithms
enum {
    ALT_CONTROL_DEFAULT      = 0,
    ALT_CONTROL_NON_AIRSPEED = 1,
    ALT_CONTROL_TECS         = 2,
    ALT_CONTROL_AIRSPEED     = 3
};

enum {
    CRASH_DETECT_ACTION_BITMASK_DISABLED = 0,
    CRASH_DETECT_ACTION_BITMASK_DISARM = (1<<0),
    // note: next enum will be (1<<1), then (1<<2), then (1<<3)
};

enum {
    USE_REVERSE_THRUST_NEVER                    = 0,
    USE_REVERSE_THRUST_AUTO_ALWAYS              = (1<<0),
    USE_REVERSE_THRUST_AUTO_LAND_APPROACH       = (1<<1),
    USE_REVERSE_THRUST_AUTO_LOITER_TO_ALT       = (1<<2),
    USE_REVERSE_THRUST_AUTO_LOITER_ALL          = (1<<3),
    USE_REVERSE_THRUST_AUTO_WAYPOINT            = (1<<4),
    USE_REVERSE_THRUST_LOITER                   = (1<<5),
    USE_REVERSE_THRUST_RTL                      = (1<<6),
    USE_REVERSE_THRUST_CIRCLE                   = (1<<7),
    USE_REVERSE_THRUST_CRUISE                   = (1<<8),
    USE_REVERSE_THRUST_FBWB                     = (1<<9),
    USE_REVERSE_THRUST_GUIDED                   = (1<<10),
};
