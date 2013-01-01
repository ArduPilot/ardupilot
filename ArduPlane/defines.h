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
#define FAILSAFE_NONE   0
#define FAILSAFE_SHORT  1
#define FAILSAFE_LONG   2
#define FAILSAFE_GCS    3
#define FAILSAFE_SHORT_TIME 1500        // Miliiseconds
#define FAILSAFE_LONG_TIME  20000       // Miliiseconds


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

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4
#define CH_YAW CH_4

// HIL enumerations
#define HIL_MODE_DISABLED                       0
#define HIL_MODE_ATTITUDE                       1
#define HIL_MODE_SENSORS                        2

enum FlightMode {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    AUTO          = 10,
    RTL           = 11,
    LOITER        = 12,
    GUIDED        = 15,
    INITIALISING  = 16
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

enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL
};

//  Logging parameters
#define LOG_INDEX_MSG                   0xF0
#define LOG_ATTITUDE_MSG                0x01
#define LOG_GPS_MSG                             0x02
#define LOG_MODE_MSG                    0X03
#define LOG_CONTROL_TUNING_MSG  0X04
#define LOG_NAV_TUNING_MSG              0X05
#define LOG_PERFORMANCE_MSG             0X06
#define LOG_RAW_MSG                             0x07
#define LOG_CMD_MSG                             0x08
#define LOG_CURRENT_MSG                 0x09
#define LOG_STARTUP_MSG                 0x0A
#define TYPE_AIRSTART_MSG               0x00
#define TYPE_GROUNDSTART_MSG    0x01
#define MAX_NUM_LOGS                    100

#define MASK_LOG_ATTITUDE_FAST  (1<<0)
#define MASK_LOG_ATTITUDE_MED   (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_MODE                   (1<<6)
#define MASK_LOG_RAW                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CUR                    (1<<9)

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


#define BATTERY_VOLTAGE(x) (x*(g.input_voltage/1024.0))*g.volt_div_ratio
#define CURRENT_AMPS(x) ((x*(g.input_voltage/1024.0))-g.curr_amp_offset)*g.curr_amp_per_volt

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

#define CONFIG_INS_OILPAN 1
#define CONFIG_INS_MPU6000 2
#define CONFIG_INS_STUB 3

#define AP_BARO_BMP085   1
#define AP_BARO_MS5611   2

// altitude control algorithms
enum {
    ALT_CONTROL_DEFAULT=0,
    ALT_CONTROL_NON_AIRSPEED=1
};


#endif // _DEFINES_H
