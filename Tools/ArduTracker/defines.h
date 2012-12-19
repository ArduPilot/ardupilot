// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define DEBUG 0
#define LOITER_RANGE 30 // for calculating power outside of loiter radius

// GPS baud rates
// --------------
#define NO_GPS		38400
#define NMEA_GPS	38400
#define EM406_GPS	57600
#define UBLOX_GPS	38400
#define ARDU_IMU	38400
#define MTK_GPS		38400
#define SIM_GPS		38400

// GPS type codes - use the names, not the numbers
#define GPS_PROTOCOL_NONE	-1
#define GPS_PROTOCOL_NMEA	0
#define GPS_PROTOCOL_SIRF	1
#define GPS_PROTOCOL_UBLOX	2
#define GPS_PROTOCOL_IMU	3
#define GPS_PROTOCOL_MTK	4
#define GPS_PROTOCOL_HIL	5

// Radio channels
// Note channels are from 0!
//
// XXX these should be CH_n defines from RC.h at some point.
#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_RUDDER 3
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#define WP_SIZE 14

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

// PID enumeration
// ---------------
#define CASE_SERVO_ROLL 0
#define CASE_SERVO_PITCH 1
#define CASE_SERVO_RUDDER 2
#define CASE_NAV_ROLL 3
#define CASE_NAV_PITCH_ASP 4
#define CASE_NAV_PITCH_ALT 5
#define CASE_TE_THROTTLE 6
#define CASE_ALT_THROTTLE 7

// Feedforward cases
// ----------------
#define CASE_PITCH_COMP 0
#define CASE_RUDDER_MIX 1
#define CASE_P_TO_T 2

// Auto Pilot modes
// ----------------
#define MANUAL 0
#define CIRCLE 1			 // When flying sans GPS, and we loose the radio, just circle
#define STABILIZE 2

#define FLY_BY_WIRE_A 5		// Fly By Wire A has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical = manual throttle
#define FLY_BY_WIRE_B 6		// Fly By Wire B has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical => desired airspeed
							// Fly By Wire B = Fly By Wire A if you have AIRSPEED_SENSOR 0
#define AUTO 10
#define RTL 11
#define LOITER 12
#define TAKEOFF 13
#define LAND 14


// Command IDs - Must
#define CMD_BLANK 0x00 // there is no command stored in the mem location requested
#define CMD_WAYPOINT 0x10
#define CMD_LOITER 0x11
#define CMD_LOITER_N_TURNS 0x12
#define CMD_LOITER_TIME 0x13
#define CMD_RTL 0x14
#define CMD_LAND 0x15
#define CMD_TAKEOFF 0x16

// Command IDs - May
#define CMD_DELAY 0x20
#define CMD_CLIMB 0x21 // NOT IMPLEMENTED
#define CMD_LAND_OPTIONS 0x22			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0

// Command IDs - Now
//#define CMD_AP_MODE 0x30
#define CMD_RESET_INDEX 0x31
#define CMD_GOTO_INDEX 0x32	// NOT IMPLEMENTED
#define CMD_GETVAR_INDEX 0x33
#define CMD_SENDVAR_INDEX 0x34
#define CMD_TELEMETRY 0x35

#define CMD_THROTTLE_CRUISE 0x40
#define CMD_AIRSPEED_CRUISE 0x41
#define CMD_RESET_HOME 0x44

#define CMD_KP_GAIN 0x60
#define CMD_KI_GAIN 0x61
#define CMD_KD_GAIN 0x62
#define CMD_KI_MAX 0x63
#define CMD_KFF_GAIN 0x64

#define CMD_RADIO_TRIM 0x70
#define CMD_RADIO_MAX 0x71
#define CMD_RADIO_MIN 0x72
#define CMD_RADIO_MIN 0x72
#define CMD_ELEVON_TRIM 0x73

#define CMD_INDEX 0x75   // sets the current Must index
#define CMD_REPEAT 0x80
#define CMD_RELAY 0x81
#define CMD_SERVO 0x82	// move servo N to PWM value

//repeating events
#define NO_REPEAT 0
#define CH_4_TOGGLE 1
#define CH_5_TOGGLE 2
#define CH_6_TOGGLE 3
#define CH_7_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

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
#define MSG_EXTENDED_STATUS 0x0a
#define MSG_CPU_LOAD 0x0b

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

#define MSG_TRIM_STARTUP 0x50
#define MSG_TRIM_MIN 0x51
#define MSG_TRIM_MAX 0x52
#define MSG_RADIO_OUT 0x53

#define MSG_RAW_IMU 0x60
#define MSG_GPS_STATUS 0x61
#define MSG_GPS_RAW 0x62
#define MSG_AIRSPEED 0x63

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

#define SEVERITY_LOW 1
#define SEVERITY_MEDIUM 2
#define SEVERITY_HIGH 3
#define SEVERITY_CRITICAL 4

//  Logging parameters
#define LOG_ATTITUDE_MSG		0x01
#define LOG_GPS_MSG				0x02
#define LOG_MODE_MSG			0X03
#define LOG_CONTROL_TUNING_MSG	0X04
#define LOG_NAV_TUNING_MSG		0X05
#define LOG_PERFORMANCE_MSG		0X06
#define LOG_RAW_MSG				0x07
#define LOG_CMD_MSG				0x08
#define LOG_STARTUP_MSG			0x09
#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01

#define MASK_LOG_ATTITUDE_FAST 0
#define MASK_LOG_ATTITUDE_MED 2
#define MASK_LOG_GPS 4
#define MASK_LOG_PM 8
#define MASK_LOG_CTUN 16
#define MASK_LOG_NTUN 32
#define MASK_LOG_MODE 64
#define MASK_LOG_RAW 128
#define MASK_LOG_CMD 256

// Yaw modes
#define YAW_MODE_COORDINATE_TURNS 0
#define YAW_MODE_HOLD_HEADING 1
#define YAW_MODE_SLIP 2

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

//GPS_fix
#define VALID_GPS 0x00
#define BAD_GPS 0x01
#define FAILED_GPS 0x03

// Climb rate calculations
#define	ALTITUDE_HISTORY_LENGTH 8	//Number of (time,altitude) points to regress a climb rate from


#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO

#define	AIRSPEED_CH 7			// The external ADC channel for the airspeed sensor
#define BATTERY_PIN1 0		        // These are the pins for the voltage dividers
#define BATTERY_PIN2 1
#define BATTERY_PIN3 2
#define BATTERY_PIN4 3
#define RELAY_PIN 47

// Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41

#define A_LED_PIN 37			//36 = B,	37 = A,	35 = C
#define B_LED_PIN 36
#define C_LED_PIN 35

#define HOLD_ALT_ABOVE_HOME 8 // bitmask value

#define SPEEDFILT 400			// centimeters/second; the speed below which a groundstart will be triggered

#define EEPROM_MAX_ADDR		4096

// log
#define EE_END_OF_PARAMS getAddress(PARAM_LAST_UINT32)
#define EE_LAST_LOG_PAGE EE_END_OF_PARAMS+1
#define EE_LAST_LOG_NUM EE_END_OF_PARAMS+3
#define EE_LOG_1_START EE_END_OF_PARAMS+5
#define WP_START_BYTE EE_END_OF_PARAMS+45 // where in memory home WP is stored + all other WP
										  // add 19 to account for log files

// bits in log_bitmask
#define LOGBIT_ATTITUDE_FAST	(1<<0)
#define LOGBIT_ATTITUDE_MED		(1<<1)
#define LOGBIT_GPS				(1<<2)
#define LOGBIT_PM				(1<<3)
#define LOGBIT_CTUN				(1<<4)
#define LOGBIT_NTUN				(1<<5)
#define LOGBIT_MODE				(1<<6)
#define LOGBIT_RAW				(1<<7)
#define LOGBIT_CMD				(1<<8)

