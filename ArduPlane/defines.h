// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define DEBUG 0
#define LOITER_RANGE 60 // for calculating power outside of loiter radius
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
    GCS_FAILSAFE_HB_RSSI    = 2  // failsafe if we stop receiving
                                 // heartbeat or if RADIO.remrssi
                                 // drops to 0
};


// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

#define PITOT_SOURCE_ADC 1
#define PITOT_SOURCE_ANALOG_PIN 2

#define T6 1000000
#define T7 10000000

// HIL enumerations. Note that HIL_MODE_ATTITUDE and HIL_MODE_SENSORS
// are now the same thing, and are sensors based. The old define is
// kept to allow old APM_Config.h headers to keep working
#define HIL_MODE_DISABLED                       0
#define HIL_MODE_SENSORS                        1

enum FlightMode {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    ACRO          = 4,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    CRUISE        = 7,
    AUTOTUNE      = 8,
    AUTO          = 10,
    RTL           = 11,
    LOITER        = 12,
    GUIDED        = 15,
    INITIALISING  = 16
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
    MIXING_DNDN     = 4
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

//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10


// Logging message types. NOTE: If you change the value of one
// of these then existing logs will break! Only add at the end, and 
// mark unused ones as 'deprecated', but leave them in
enum log_messages {
    LOG_CTUN_MSG,
    LOG_NTUN_MSG,
    LOG_PERFORMANCE_MSG,
    LOG_CMD_MSG_DEPRECATED,     // deprecated
    LOG_CURRENT_MSG,
    LOG_STARTUP_MSG,
    TYPE_AIRSTART_MSG,
    TYPE_GROUNDSTART_MSG,
    LOG_CAMERA_MSG_DEPRECATED,
    LOG_ATTITUDE_MSG,
    LOG_MODE_MSG,
    LOG_COMPASS_MSG,
    LOG_TECS_MSG,
    LOG_RC_MSG,
    LOG_SONAR_MSG,
    LOG_COMPASS2_MSG,
    LOG_ARM_DISARM_MSG,
    LOG_AIRSPEED_MSG,
    LOG_COMPASS3_MSG
#if OPTFLOW == ENABLED
    ,LOG_OPTFLOW_MSG
#endif
};

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_MODE                   (1<<6)
#define MASK_LOG_IMU                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CURRENT                (1<<9)
#define MASK_LOG_COMPASS                (1<<10)
#define MASK_LOG_TECS                   (1<<11)
#define MASK_LOG_CAMERA                 (1<<12)
#define MASK_LOG_RC                     (1<<13)
#define MASK_LOG_SONAR                  (1<<14)
#define MASK_LOG_ARM_DISARM             (1<<15)
#define MASK_LOG_WHEN_DISARMED          (1UL<<16)

// Waypoint Modes
// ----------------
#define ABS_WP 0
#define REL_WP 1

// Command Queues
// ---------------
#define COMMAND_MUST 0
#define COMMAND_MAY 1
#define COMMAND_NOW 2

// Events
// ------
#define EVENT_WILL_REACH_WAYPOINT 1
#define EVENT_SET_NEW_COMMAND_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

// Climb rate calculations
#define ALTITUDE_HISTORY_LENGTH 8       //Number of (time,altitude) points to
                                        // regress a climb rate from

#define AN4                     4
#define AN5                     5

#define SPEEDFILT 400                   // centimeters/second; the speed below
                                        // which a groundstart will be
                                        // triggered

// convert a boolean (0 or 1) to a sign for multiplying (0 maps to 1, 1 maps
// to -1)
#define BOOL_TO_SIGN(bvalue) ((bvalue) ? -1 : 1)

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// altitude control algorithms
enum {
    ALT_CONTROL_DEFAULT      = 0,
    ALT_CONTROL_NON_AIRSPEED = 1,
    ALT_CONTROL_TECS         = 2,
    ALT_CONTROL_AIRSPEED     = 3
};

// attitude controller choice
enum {
    ATT_CONTROL_PID = 0,
    ATT_CONTROL_APMCONTROL = 1
};

enum Serial2Protocol {
    SERIAL2_MAVLINK     = 1,
    SERIAL2_FRSKY_DPORT = 2,
    SERIAL2_FRSKY_SPORT = 3 // not supported yet
};

#endif // _DEFINES_H
