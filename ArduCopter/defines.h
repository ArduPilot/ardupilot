// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// Flight modes
// ------------
#define YAW_HOLD                        0
#define YAW_ACRO                        1
#define YAW_AUTO                        2
#define YAW_LOOK_AT_HOME    		    3
#define YAW_TOY                         4       // THOR This is the Yaw mode
#define YAW_LOOK_AHEAD					5		// WARNING!  CODE IN DEVELOPMENT NOT PROVEN


#define ROLL_PITCH_STABLE       0
#define ROLL_PITCH_ACRO         1
#define ROLL_PITCH_AUTO         2
#define ROLL_PITCH_STABLE_OF    3
#define ROLL_PITCH_TOY          4       // THOR This is the Roll and Pitch
                                        // mode

#define THROTTLE_MANUAL                     0   // manual throttle mode - pilot input goes directly to motors
#define THROTTLE_MANUAL_TILT_COMPENSATED    1   // mostly manual throttle but with some tilt compensation
#define THROTTLE_ACCELERATION               2   // pilot inputs the desired acceleration
#define THROTTLE_RATE                       3   // pilot inputs the desired climb rate.  Note: this uses the unstabilized rate controller
#define THROTTLE_STABILIZED_RATE            4   // pilot inputs the desired climb rate.  Uses stabilized rate controller
#define THROTTLE_DIRECT_ALT                 5   // pilot inputs a desired altitude from 0 ~ 10 meters
#define THROTTLE_HOLD                       6   // alt hold plus pilot input of climb rate
#define THROTTLE_AUTO                       7   // auto pilot altitude controller with target altitude held in next_WP.alt
#define THROTTLE_LAND                       8   // landing throttle controller


// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

#define SONAR_SOURCE_ADC 1
#define SONAR_SOURCE_ANALOG_PIN 2

// CH 7 control
#define CH7_PWM_TRIGGER 1800    // pwm value above which the channel 7 option will be invoked
#define CH7_DO_NOTHING 0
#define CH7_SET_HOVER 1         // deprecated
#define CH7_FLIP 2
#define CH7_SIMPLE_MODE 3
#define CH7_RTL 4
#define CH7_SAVE_TRIM 5
#define CH7_ADC_FILTER 6        // deprecated
#define CH7_SAVE_WP 7
#define CH7_MULTI_MODE 8        // deprecated
#define CH7_CAMERA_TRIGGER 9


// Frame types
#define QUAD_FRAME 0
#define TRI_FRAME 1
#define HEXA_FRAME 2
#define Y6_FRAME 3
#define OCTA_FRAME 4
#define HELI_FRAME 5
#define OCTA_QUAD_FRAME 6

#define PLUS_FRAME 0
#define X_FRAME 1
#define V_FRAME 2

// LED output
#define NORMAL_LEDS 0
#define SAVE_TRIM_LEDS 1


// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) (x*0.01745329252)      // *pi/180
#define ToDeg(x) (x*57.2957795131)      // *180/pi

#define DEBUG 0
#define LOITER_RANGE 60 // for calculating power outside of loiter radius

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
#define GPS_PROTOCOL_MTK16      6
#define GPS_PROTOCOL_AUTO       7

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4
#define CH_YAW CH_4

#define RC_CHANNEL_ANGLE 0
#define RC_CHANNEL_RANGE 1
#define RC_CHANNEL_ANGLE_RAW 2

// HIL enumerations
#define HIL_MODE_DISABLED               0
#define HIL_MODE_ATTITUDE               1
#define HIL_MODE_SENSORS                2

#define REACHED_ALT                     0
#define DESCENDING                      1
#define ASCENDING                       2

// Auto Pilot modes
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define POSITION 8                      // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10                    // Hold a single location using optical flow
                                        // sensor
#define TOY_A 11                        // THOR Enum for Toy mode
#define TOY_M 12                        // THOR Enum for Toy mode
#define NUM_MODES 13

#define SIMPLE_1 1
#define SIMPLE_2 2
#define SIMPLE_3 4
#define SIMPLE_4 8
#define SIMPLE_5 16
#define SIMPLE_6 32

// CH_6 Tuning
// -----------
#define CH6_NONE 0
// Attitude
#define CH6_STABILIZE_KP 1
#define CH6_STABILIZE_KI 2
#define CH6_STABILIZE_KD 29             // duplicate with CH6_DAMP
#define CH6_YAW_KP 3
#define CH6_YAW_KI 24
// Rate
#define CH6_ACRO_KP 25
#define CH6_RATE_KP 4
#define CH6_RATE_KI 5
#define CH6_RATE_KD 21
#define CH6_YAW_RATE_KP 6
#define CH6_YAW_RATE_KD 26
// Throttle
#define CH6_THROTTLE_KP 7
#define CH6_THROTTLE_KI 33
#define CH6_THR_ACCEL_KP 34
#define CH6_THR_ACCEL_KI 35
#define CH6_THR_ACCEL_KD 36
// Extras
#define CH6_TOP_BOTTOM_RATIO 8
#define CH6_RELAY 9
// Navigation
#define CH6_TRAVERSE_SPEED 10   // maximum speed to next way point
#define CH6_NAV_KP 11
#define CH6_LOITER_KP 12
#define CH6_LOITER_KI 27

// Trad Heli specific
#define CH6_HELI_EXTERNAL_GYRO 13

// altitude controller
#define CH6_THR_HOLD_KP 14
#define CH6_Z_GAIN 15
#define CH6_DAMP 16             // duplicate with CH6_YAW_RATE_KD

// optical flow controller
#define CH6_OPTFLOW_KP 17
#define CH6_OPTFLOW_KI 18
#define CH6_OPTFLOW_KD 19

#define CH6_NAV_I 20
#define CH6_LOITER_RATE_KP 22
#define CH6_LOITER_RATE_KI 28
#define CH6_LOITER_RATE_KD 23

#define CH6_AHRS_YAW_KP 30
#define CH6_AHRS_KP             31

// Inertial Nav
#define CH6_INAV_TC     32

// nav byte mask
// -------------
#define NAV_LOCATION 1
#define NAV_ALTITUDE 2
#define NAV_DELAY    4


// Commands - Note that APM now uses a subset of the MAVLink protocol
// commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location
                    // requested
#define NO_COMMAND 0


// Navigation modes held in wp_control variable
#define LOITER_MODE 1
#define WP_MODE 2
#define CIRCLE_MODE 3
#define NO_NAV_MODE 4

// TOY mixing options
#define TOY_LOOKUP_TABLE 0
#define TOY_LINEAR_MIXER 1
#define TOY_EXTERNAL_MIXER 2


// Waypoint options
#define MASK_OPTIONS_RELATIVE_ALT               1
#define WP_OPTION_ALT_CHANGE                    2
#define WP_OPTION_YAW                           4
#define WP_OPTION_ALT_REQUIRED                  8
#define WP_OPTION_RELATIVE                      16
//#define WP_OPTION_					32
//#define WP_OPTION_					64
#define WP_OPTION_NEXT_CMD                      128

// RTL state
#define RTL_STATE_RETURNING_HOME    0
#define RTL_STATE_LOITERING_AT_HOME 1
#define RTL_STATE_FINAL_DESCENT     2
#define RTL_STATE_LAND              3

//repeating events
#define RELAY_TOGGLE 5

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
    MSG_GPS_STATUS,
    MSG_GPS_RAW,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_LIMITS_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_RETRY_DEFERRED // this must be last
};

enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL
};

//  Logging parameters
#define TYPE_AIRSTART_MSG               0x00
#define TYPE_GROUNDSTART_MSG            0x01
#define LOG_ATTITUDE_MSG                0x01
#define LOG_GPS_MSG                     0x02
#define LOG_MODE_MSG                    0x03
#define LOG_CONTROL_TUNING_MSG          0x04
#define LOG_NAV_TUNING_MSG              0x05
#define LOG_PERFORMANCE_MSG             0x06
#define LOG_RAW_MSG                     0x07
#define LOG_CMD_MSG                     0x08
#define LOG_CURRENT_MSG                 0x09
#define LOG_STARTUP_MSG                 0x0A
#define LOG_MOTORS_MSG                  0x0B
#define LOG_OPTFLOW_MSG                 0x0C
#define LOG_DATA_MSG                    0x0D
#define LOG_PID_MSG                     0x0E
#define LOG_ITERM_MSG                   0x0F
#define LOG_DMP_MSG                     0x10
#define LOG_INAV_MSG                    0x11
#define LOG_CAMERA_MSG                  0x12
#define LOG_INDEX_MSG                   0xF0
#define MAX_NUM_LOGS                    50

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_MODE                   (1<<6)
#define MASK_LOG_RAW                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CUR                    (1<<9)
#define MASK_LOG_MOTORS                 (1<<10)
#define MASK_LOG_OPTFLOW                (1<<11)
#define MASK_LOG_PID                    (1<<12)
#define MASK_LOG_ITERM                  (1<<13)
#define MASK_LOG_INAV                   (1<<14)
#define MASK_LOG_CAMERA                 (1<<15)

// DATA - event logging
#define DATA_MAVLINK_FLOAT              1
#define DATA_MAVLINK_INT32              2
#define DATA_MAVLINK_INT16              3
#define DATA_MAVLINK_INT8               4
#define DATA_FAST_LOOP                  5
#define DATA_MED_LOOP                   6
#define DATA_AP_STATE                   7
#define DATA_SIMPLE_BEARING             8
#define DATA_INIT_SIMPLE_BEARING        9
#define DATA_ARMED                      10
#define DATA_DISARMED                   11
#define DATA_FAILSAFE_ON                12
#define DATA_FAILSAFE_OFF               13
#define DATA_LOW_BATTERY                14
#define DATA_AUTO_ARMED                 15
#define DATA_TAKEOFF                    16
#define DATA_DID_REACH_ALT              17
#define DATA_LAND_COMPLETE              18
#define DATA_LOST_GPS                   19
#define DATA_LOST_COMPASS               20
#define DATA_BEGIN_FLIP                 21
#define DATA_END_FLIP                   22
#define DATA_EXIT_FLIP                  23
#define DATA_FLIP_ABORTED               24
#define DATA_SET_HOME                   25
#define DATA_SET_SIMPLE_ON              26
#define DATA_SET_SIMPLE_OFF             27
#define DATA_REACHED_ALT                28
#define DATA_ASCENDING                  29
#define DATA_DESCENDING                 30
#define DATA_RTL_REACHED_ALT            31


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
#define EVENT_SET_NEW_WAYPOINT_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

// Climb rate calculations
#define ALTITUDE_HISTORY_LENGTH 8       //Number of (time,altitude) points to
                                        // regress a climb rate from

#define BATTERY_VOLTAGE(x) (x*(g.input_voltage/1024.0))*g.volt_div_ratio
#define CURRENT_AMPS(x) ((x*(g.input_voltage/1024.0))-CURR_AMPS_OFFSET)*g.curr_amp_per_volt

//#define BARO_FILTER_SIZE 8

/* ************************************************************** */
/* Expansion PIN's that people can use for various things. */

// AN0 - 7 are located at edge of IMU PCB "above" pressure sensor and
// Expansion port
// AN0 - 5 are also located next to voltage dividers and sliding SW2 switch
// AN0 - 3 has 10kOhm resistor in serial, include 3.9kOhm to make it as
// voltage divider
// AN4 - 5 are direct GPIO pins from atmega1280 and they are the latest pins
// next to SW2 switch
// Look more ArduCopter Wiki for voltage dividers and other ports
#define AN0  54  // resistor, vdiv use, divider 1 closest to relay
#define AN1  55  // resistor, vdiv use, divider 2
#define AN2  56  // resistor, vdiv use, divider 3
#define AN3  57  // resistor, vdiv use, divider 4 closest to SW2
#define AN4  58  // direct GPIO pin, default as analog input, next to SW2
                 // switch
#define AN5  59  // direct GPIO pin, default as analog input, next to SW2
                 // switch
#define AN6  60  // direct GPIO pin, default as analog input, close to
                 // Pressure sensor, Expansion Ports
#define AN7  61  // direct GPIO pin, default as analog input, close to
                 // Pressure sensor, Expansion Ports

// AN8 - 15 are located at edge of IMU PCB "above" pressure sensor and
// Expansion port
// AN8 - 15 PINs are not connected anywhere, they are located as last 8 pins
// on edge of the board above Expansion Ports
// even pins (8,10,12,14) are at edge of board, Odd pins (9,11,13,15) are on
// inner row
#define AN8  62  // NC
#define AN9  63  // NC
#define AN10  64 // NC
#define AN11  65 // NC
#define AN12  66 // NC
#define AN13  67 // NC
#define AN14  68 // NC
#define AN15  69 // NC

#define RELAY_PIN 47

#define PIEZO_PIN AN5           //Last pin on the back ADC connector



// sonar
//#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters

// RADIANS
#define RADX100 0.000174532925
#define DEGX100 5729.57795


// EEPROM addresses
#define EEPROM_MAX_ADDR         4096
// parameters get the first 1536 bytes of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x600 // where in memory home WP is stored + all other
                            // WP
#define WP_SIZE 15

// fence points are stored at the end of the EEPROM
#define MAX_FENCEPOINTS 6
#define FENCE_WP_SIZE sizeof(Vector2l)
#define FENCE_START_BYTE (EEPROM_MAX_ADDR-(MAX_FENCEPOINTS*FENCE_WP_SIZE))

#define MAX_WAYPOINTS  ((FENCE_START_BYTE - WP_START_BYTE) / WP_SIZE) - 1 // -
                                                                          // 1
                                                                          // to
                                                                          // be
                                                                          // safe

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// IMU selection
#define CONFIG_IMU_OILPAN 1
#define CONFIG_IMU_MPU6000 2

// APM Hardware selection
#define APM_HARDWARE_APM1 1
#define APM_HARDWARE_APM2 2

#define AP_BARO_BMP085    1
#define AP_BARO_MS5611    2

#endif // _DEFINES_H
