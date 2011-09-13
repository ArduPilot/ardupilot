// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define DEBUG 0
#define LOITER_RANGE 60 // for calculating power outside of loiter radius
#define SERVO_MAX 4500	// This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// failsafe
// ----------------------
#define FAILSAFE_NONE	0
#define FAILSAFE_SHORT	1
#define FAILSAFE_LONG	2
#define FAILSAFE_GCS	3
#define	FAILSAFE_SHORT_TIME 1500	// Miliiseconds
#define	FAILSAFE_LONG_TIME  20000	// Miliiseconds


// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

#define T6 1000000
#define T7 10000000

// GPS type codes - use the names, not the numbers
#define GPS_PROTOCOL_NONE	-1
#define GPS_PROTOCOL_NMEA	0
#define GPS_PROTOCOL_SIRF	1
#define GPS_PROTOCOL_UBLOX	2
#define GPS_PROTOCOL_IMU	3
#define GPS_PROTOCOL_MTK	4
#define GPS_PROTOCOL_HIL	5
#define GPS_PROTOCOL_MTK16	6
#define GPS_PROTOCOL_AUTO	7

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4
#define CH_YAW CH_4

// HIL enumerations
#define HIL_PROTOCOL_XPLANE			1
#define HIL_PROTOCOL_MAVLINK		2

#define HIL_MODE_DISABLED			0
#define HIL_MODE_ATTITUDE			1
#define HIL_MODE_SENSORS			2

// GCS enumeration
#define GCS_PROTOCOL_STANDARD		0	// standard APM protocol
#define GCS_PROTOCOL_LEGACY			1	// legacy ArduPilot protocol
#define GCS_PROTOCOL_XPLANE			2	// X-Plane HIL simulation
#define GCS_PROTOCOL_DEBUGTERMINAL	3	//Human-readable debug interface for use with a dumb terminal
#define GCS_PROTOCOL_MAVLINK	        4	// binary protocol for qgroundcontrol
#define GCS_PROTOCOL_NONE			-1	// No GCS output

// Auto Pilot modes
// ----------------
#define MANUAL 0
#define CIRCLE 1			 // When flying sans GPS, and we loose the radio, just circle
#define STABILIZE 2

#define FLY_BY_WIRE_A 5		// Fly By Wire A has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical = manual throttle
#define FLY_BY_WIRE_B 6		// Fly By Wire B has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical => desired airspeed
#define FLY_BY_WIRE_C 7		// Fly By Wire C has left stick horizontal => desired roll angle, left stick vertical => desired climb rate, right stick vertical => desired airspeed
							// Fly By Wire B and Fly By Wire C require airspeed sensor
#define AUTO 10
#define RTL 11
#define LOITER 12
//#define TAKEOFF 13			// This is not used by APM.  It appears here for consistency with ACM
//#define LAND 14			// This is not used by APM.  It appears here for consistency with ACM
#define GUIDED 15
#define INITIALISING 16     // in startup routines


// Commands - Note that APM now uses a subset of the MAVLink protocol commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location requested
#define NO_COMMAND 0

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT	(1<<0)		// 1 = Relative altitude

//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

#define MAV_CMD_CONDITION_YAW 23

//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
#define MSG_ACKNOWLEDGE 0x00
#define MSG_HEARTBEAT 0x01
#define MSG_ATTITUDE 0x02
#define MSG_LOCATION 0x03
#define MSG_PRESSURE 0x04
#define MSG_STATUS_TEXT 0x05
#define MSG_PERF_REPORT 0x06
#define MSG_MODE_CHANGE 0x07 //This is different than HEARTBEAT because it occurs only when the mode is actually changed
#define MSG_VERSION_REQUEST 0x08
#define MSG_VERSION 0x09
#define MSG_EXTENDED_STATUS1 0x0a
#define MSG_EXTENDED_STATUS2 0x0b
#define MSG_CPU_LOAD 0x0c
#define MSG_NAV_CONTROLLER_OUTPUT 0x0d

#define MSG_COMMAND_REQUEST 0x20
#define MSG_COMMAND_UPLOAD 0x21
#define MSG_COMMAND_LIST 0x22
#define MSG_COMMAND_MODE_CHANGE 0x23
#define MSG_CURRENT_WAYPOINT 0x24

#define MSG_VALUE_REQUEST 0x30
#define MSG_VALUE_SET 0x31
#define MSG_VALUE 0x32

#define MSG_PID_REQUEST 0x40
#define MSG_PID_SET 0x41
#define MSG_PID 0x42
#define MSG_VFR_HUD 0x4a

#define MSG_TRIM_STARTUP 0x50
#define MSG_TRIM_MIN 0x51
#define MSG_TRIM_MAX 0x52
#define MSG_RADIO_OUT 0x53
#define MSG_RADIO_IN  0x54

#define MSG_RAW_IMU1 0x60
#define MSG_RAW_IMU2 0x61
#define MSG_RAW_IMU3 0x62
#define MSG_GPS_STATUS 0x63
#define MSG_GPS_RAW 0x64

#define MSG_SERVO_OUT 0x70

#define MSG_PIN_REQUEST 0x80
#define MSG_PIN_SET 0x81

#define MSG_DATAFLASH_REQUEST 0x90
#define MSG_DATAFLASH_SET 0x91

#define MSG_EEPROM_REQUEST 0xa0
#define MSG_EEPROM_SET 0xa1

#define MSG_POSITION_CORRECT 0xb0
#define MSG_ATTITUDE_CORRECT 0xb1
#define MSG_POSITION_SET 0xb2
#define MSG_ATTITUDE_SET 0xb3
#define MSG_LOCAL_LOCATION 0xb4
#define MSG_RETRY_DEFERRED 0xff

#define SEVERITY_LOW 1
#define SEVERITY_MEDIUM 2
#define SEVERITY_HIGH 3
#define SEVERITY_CRITICAL 4

//  Logging parameters
#define LOG_INDEX_MSG			0xF0
#define LOG_ATTITUDE_MSG		0x01
#define LOG_GPS_MSG				0x02
#define LOG_MODE_MSG			0X03
#define LOG_CONTROL_TUNING_MSG	0X04
#define LOG_NAV_TUNING_MSG		0X05
#define LOG_PERFORMANCE_MSG		0X06
#define LOG_RAW_MSG				0x07
#define LOG_CMD_MSG				0x08
#define LOG_CURRENT_MSG 		0x09
#define LOG_STARTUP_MSG 		0x0A
#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01
#define MAX_NUM_LOGS			50

#define MASK_LOG_ATTITUDE_FAST 	(1<<0)
#define MASK_LOG_ATTITUDE_MED 	(1<<1)
#define MASK_LOG_GPS 			(1<<2)
#define MASK_LOG_PM 			(1<<3)
#define MASK_LOG_CTUN 			(1<<4)
#define MASK_LOG_NTUN			(1<<5)
#define MASK_LOG_MODE			(1<<6)
#define MASK_LOG_RAW			(1<<7)
#define MASK_LOG_CMD			(1<<8)
#define MASK_LOG_CUR			(1<<9)

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
#define	ALTITUDE_HISTORY_LENGTH 8	//Number of (time,altitude) points to regress a climb rate from


#define BATTERY_VOLTAGE(x) (x*(g.input_voltage/1024.0))*g.volt_div_ratio

#define CURRENT_AMPS(x) ((x*(g.input_voltage/1024.0))-CURR_AMPS_OFFSET)*g.curr_amp_per_volt

#define	AIRSPEED_CH 7			// The external ADC channel for the airspeed sensor
#define BATTERY_PIN1 0		        // These are the pins for the voltage dividers
#define BATTERY_PIN2 1
#define BATTERY_PIN3 2
#define BATTERY_PIN4 3

#define VOLTAGE_PIN_0 0 // These are the pins for current sensor: voltage
#define CURRENT_PIN_1 1 // and current

#define RELAY_PIN 47


// sonar
#define MAX_SONAR_XL 0
#define MAX_SONAR_LV 1
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters
#define AN4			4
#define AN5			5

// Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41

#define A_LED_PIN 37			//36 = B,	37 = A,	35 = C
#define B_LED_PIN 36
#define C_LED_PIN 35

#define SPEEDFILT 400			// centimeters/second; the speed below which a groundstart will be triggered


// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
// parameters get the first 1KiB of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x400 // where in memory home WP is stored + all other WP
#define WP_SIZE 15

#define ONBOARD_PARAM_NAME_LENGTH 15
#define MAX_WAYPOINTS  ((EEPROM_MAX_ADDR - WP_START_BYTE) / WP_SIZE) - 1 // - 1 to be safe

// convert a boolean (0 or 1) to a sign for multiplying (0 maps to 1, 1 maps to -1)
#define BOOL_TO_SIGN(bvalue) ((bvalue)?-1:1)
