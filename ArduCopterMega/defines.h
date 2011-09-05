// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Just so that it's completely clear...
#define ENABLED			1
#define DISABLED		0


// Flight modes
// ------------
#define YAW_HOLD 			0
#define YAW_ACRO 			1
#define YAW_AUTO 			2
#define YAW_LOOK_AT_HOME 	3

#define ROLL_PITCH_STABLE 	0
#define ROLL_PITCH_ACRO 	1
#define ROLL_PITCH_SIMPLE	2
#define ROLL_PITCH_AUTO		3

#define THROTTLE_MANUAL 	0
#define THROTTLE_HOLD 		1
#define THROTTLE_AUTO		2


// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

// CH 7 control
#define DO_SET_HOVER 0
#define DO_FLIP 1

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
#define AUTO_TRIM_LEDS 1

// motor LEDs
#define FR_LED AN12  // Mega PE4 pin, OUT7
#define RE_LED AN14  // Mega PE5 pin, OUT6
#define RI_LED AN10  // Mega PH4 pin, OUT5
#define LE_LED AN8  // Mega PH5 pin, OUT4

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define DEBUG 0
#define LOITER_RANGE 60 // for calculating power outside of loiter radius

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

// SONAR types:
#define MAX_SONAR_UNKNOWN	0
#define MAX_SONAR_XL		1

// Radio channels
// Note channels are from 0!
//
// XXX these should be CH_n defines from RC.h at some point.
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_10 9    //PB5
#define CH_11 10   //PE3

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4
#define CH_YAW CH_4


#define RC_CHANNEL_ANGLE 0
#define RC_CHANNEL_RANGE 1
#define RC_CHANNEL_ANGLE_RAW 2

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
#define STABILIZE 0			// hold level position
#define ACRO 1				// rate control
#define SIMPLE 2			//
#define ALT_HOLD 3			// AUTO control
#define AUTO 4				// AUTO control
#define GUIDED 5			// AUTO control
#define LOITER 6			// Hold a single location
#define RTL 7				// AUTO control
#define CIRCLE 8				// AUTO control
#define NUM_MODES 9

// YAW debug
// ---------
#define YAW_HOLD 0
#define YAW_BRAKE 1
#define YAW_RATE 2


// CH_6 Tuning
// -----------
#define CH6_NONE 0
// Attitude
#define CH6_STABILIZE_KP 1
#define CH6_STABILIZE_KI 2
#define	CH6_YAW_KP 3
#define	CH6_YAW_KD 4
// Rate
#define CH6_RATE_KP 5
#define CH6_RATE_KI 6
#define	CH6_YAW_RATE_KP 7
#define	CH6_YAW_RATE_KD 8
// Altitude
#define CH6_THROTTLE_KP 9
#define CH6_THROTTLE_KD 10
// Extras
#define CH6_TOP_BOTTOM_RATIO 11
#define CH6_PMAX 12
#define CH6_RELAY 13
#define CH6_TRAVERSE_SPEED 14

// nav byte mask
// -------------
#define NAV_LOCATION 1
#define NAV_ALTITUDE 2
#define NAV_DELAY    4


// Commands - Note that APM now uses a subset of the MAVLink protocol commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location requested
#define NO_COMMAND 0


#define LOITER_MODE 1
#define WP_MODE 2

// Waypoint options
#define WP_OPTION_ALT_RELATIVE 		1
#define WP_OPTION_ALT_CHANGE 		2
#define WP_OPTION_YAW 				4
#define WP_OPTION_ALT_REQUIRED		8
#define WP_OPTION_RELATIVE			16
//#define WP_OPTION_					32
//#define WP_OPTION_					64
#define WP_OPTION_NEXT_CMD			128

//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

//#define MAV_CMD_CONDITION_YAW 23

//  GCS Message ID's
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
#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01
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
#define LOG_MOTORS_MSG 			0x0B
#define LOG_OPTFLOW_MSG 		0x0C
#define LOG_INDEX_MSG			0xF0
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
#define MASK_LOG_MOTORS			(1<<10)
#define MASK_LOG_OPTFLOW		(1<<11)

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


#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO
#define CURRENT_AMPS(x) ((x*(INPUT_VOLTAGE/1024.0))-CURR_AMPS_OFFSET)*CURR_AMP_PER_VOLT
//#define BARO_FILTER_SIZE 8

/* ************************************************************** */
/* Expansion PIN's that people can use for various things. */

// AN0 - 7 are located at edge of IMU PCB "above" pressure sensor and Expansion port
// AN0 - 5 are also located next to voltage dividers and sliding SW2 switch
// AN0 - 3 has 10kOhm resistor in serial, include 3.9kOhm to make it as voltage divider
// AN4 - 5 are direct GPIO pins from atmega1280 and they are the latest pins next to SW2 switch
// Look more ArduCopter Wiki for voltage dividers and other ports
#define AN0  54  // resistor, vdiv use, divider 1 closest to relay
#define AN1  55  // resistor, vdiv use, divider 2
#define AN2  56  // resistor, vdiv use, divider 3
#define AN3  57  // resistor, vdiv use, divider 4 closest to SW2
#define AN4  58  // direct GPIO pin, default as analog input, next to SW2 switch
#define AN5  59  // direct GPIO pin, default as analog input, next to SW2 switch
#define AN6  60  // direct GPIO pin, default as analog input, close to Pressure sensor, Expansion Ports
#define AN7  61  // direct GPIO pin, default as analog input, close to Pressure sensor, Expansion Ports

// AN8 - 15 are located at edge of IMU PCB "above" pressure sensor and Expansion port
// AN8 - 15 PINs are not connected anywhere, they are located as last 8 pins on edge of the board above Expansion Ports
// even pins (8,10,12,14) are at edge of board, Odd pins (9,11,13,15) are on inner row
#define AN8  62  // NC
#define AN9  63  // NC
#define AN10  64 // NC
#define AN11  65 // NC
#define AN12  66 // NC
#define AN13  67 // NC
#define AN14  68 // NC
#define AN15  69 // NC

#define VOLTAGE_PIN_0 0 // These are the pins for current sensor: voltage
#define CURRENT_PIN_1 1 // and current

#define RELAY_PIN 47

#define	AIRSPEED_CH 7			// The external ADC channel for the airspeed sensor
#define BATTERY_PIN1 0		        // These are the pins for the voltage dividers
#define BATTERY_PIN2 1
#define BATTERY_PIN3 2
#define BATTERY_PIN4 3
#define PIEZO_PIN AN5           //Last pin on the back ADC connector



// sonar
//#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters

// Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41

#define A_LED_PIN 37			//36 = B,	37 = A,	35 = C
#define B_LED_PIN 36
#define C_LED_PIN 35


// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
// parameters get the first 1KiB of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x400 // where in memory home WP is stored + all other WP
#define WP_SIZE 15

#define ONBOARD_PARAM_NAME_LENGTH 15
#define MAX_WAYPOINTS  ((EEPROM_MAX_ADDR - WP_START_BYTE) / WP_SIZE) - 1 // - 1 to be safe
