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


// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

#define PITOT_SOURCE_ADC 1
#define PITOT_SOURCE_ANALOG_PIN 2

#define T6 1000000
#define T7 10000000

// GPS type codes - use the names, not the numbers
#define GPS_PROTOCOL_NONE       -1
#define GPS_PROTOCOL_NMEA       0
#define GPS_PROTOCOL_SIRF       1
#define GPS_PROTOCOL_UBLOX      2
#define GPS_PROTOCOL_IMU        3
#define GPS_PROTOCOL_MTK        4
#define GPS_PROTOCOL_HIL        5
#define GPS_PROTOCOL_MTK19      6
#define GPS_PROTOCOL_AUTO       7

// HIL enumerations. Note that HIL_MODE_ATTITUDE and HIL_MODE_SENSORS
// are now the same thing, and are sensors based. The old define is
// kept to allow old APM_Config.h headers to keep working
#define HIL_MODE_DISABLED                       0
#define HIL_MODE_ATTITUDE                       1
#define HIL_MODE_SENSORS                        2

enum FlightMode {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    ACRO          = 4,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    CRUISE        = 7,
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

// Commands - Note that APM now uses a subset of the MAVLink protocol
// commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location
                    // requested
#define NO_COMMAND 0
#define WAIT_COMMAND 255

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT       (1<<0)          // 1 = Relative
                                                        // altitude
#define MASK_OPTIONS_LOITER_DIRECTION   (1<<2)          // 0 = CW
                                                        // 1 = CCW


//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_RADIO_OUT,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU2,
    MSG_RAW_IMU3,
    MSG_GPS_RAW,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_FENCE_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RETRY_DEFERRED // this must be last
};

// Logging message types. NOTE: If you change the value of one
// of these then existing logs will break! Only add at the end, and 
// mark unused ones as 'deprecated', but leave them in
enum log_messages {
    LOG_INDEX_MSG,
    LOG_CTUN_MSG,
    LOG_NTUN_MSG,
    LOG_PERFORMANCE_MSG,
    LOG_CMD_MSG,
    LOG_CURRENT_MSG,
    LOG_STARTUP_MSG,
    TYPE_AIRSTART_MSG,
    TYPE_GROUNDSTART_MSG,
    LOG_CAMERA_MSG,
    LOG_ATTITUDE_MSG,
    LOG_MODE_MSG,
    LOG_COMPASS_MSG,
    LOG_TECS_MSG,
    MAX_NUM_LOGS // always at the end
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


#define BATTERY_VOLTAGE(x) (x->voltage_average()*g.volt_div_ratio)
#define CURRENT_AMPS(x) (x->voltage_average()-g.curr_amp_offset)*g.curr_amp_per_volt

#define AN4                     4
#define AN5                     5

#define SPEEDFILT 400                   // centimeters/second; the speed below
                                        // which a groundstart will be
                                        // triggered


// EEPROM addresses
#define EEPROM_MAX_ADDR         4096
// parameters get the first 1280 bytes of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x500 // where in memory home WP is stored + all other
                            // WP
#define WP_SIZE 15

// fence points are stored at the end of the EEPROM
#define MAX_FENCEPOINTS 20
#define FENCE_WP_SIZE sizeof(Vector2l)
#define FENCE_START_BYTE (EEPROM_MAX_ADDR-(MAX_FENCEPOINTS*FENCE_WP_SIZE))

#define MAX_WAYPOINTS  ((FENCE_START_BYTE - WP_START_BYTE) / WP_SIZE) - 1 // -
                                                                          // 1
                                                                          // to
                                                                          // be
                                                                          // safe

// convert a boolean (0 or 1) to a sign for multiplying (0 maps to 1, 1 maps
// to -1)
#define BOOL_TO_SIGN(bvalue) ((bvalue) ? -1 : 1)

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// InertialSensor driver types
#define CONFIG_INS_OILPAN  1
#define CONFIG_INS_MPU6000 2
#define CONFIG_INS_STUB    3
#define CONFIG_INS_PX4     4

// barometer driver types
#define AP_BARO_BMP085   1
#define AP_BARO_MS5611   2
#define AP_BARO_PX4      3
#define AP_BARO_HIL      4

// compass driver types
#define AP_COMPASS_HMC5843   1
#define AP_COMPASS_PX4       2
#define AP_COMPASS_HIL       3

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

#endif // _DEFINES_H
