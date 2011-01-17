// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

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

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4
#define CH_YAW CH_4
#define CH_AUX CH_5
#define CH_AUX2 CH_6

// motors
#define RIGHT 0
#define LEFT 1
#define FRONT 2
#define BACK 3

#define MAX_SERVO_OUTPUT 2700

#define SONAR 0
#define BARO 1

#define WP_START_BYTE 0x130	// where in memory home WP is stored + all other WP
#define WP_SIZE 14

// GCS enumeration
#define GCS_PROTOCOL_STANDARD	0	// standard APM protocol
#define GCS_PROTOCOL_SPECIAL	1	// special test protocol (?)
#define GCS_PROTOCOL_LEGACY		2	// legacy ArduPilot protocol
#define GCS_PROTOCOL_XPLANE		3	// X-Plane HIL simulation
#define GCS_PROTOCOL_IMU		4	// ArdiPilot IMU output
#define GCS_PROTOCOL_JASON		5	// Jason's special secret GCS protocol
#define GCS_PROTOCOL_NONE		-1	// No GCS output

#define PLUS_FRAME 0
#define X_FRAME 1
#define TRI_FRAME 2

// Auto Pilot modes
// ----------------
#define ACRO 0				// rate control
#define STABILIZE 1			// hold level position
#define ALT_HOLD 2				// AUTO control
#define FBW 3				// AUTO control
#define AUTO 4				// AUTO control
#define POSITION_HOLD 5		// Hold a single location
#define RTL 6				// AUTO control
#define TAKEOFF 7			// controlled decent rate
#define LAND 8				// controlled decent rate
#define NUM_MODES 9

// Command IDs - Must
#define CMD_BLANK 0x00 // there is no command stored in the mem location requested
#define CMD_WAYPOINT 0x10
#define CMD_LOITER 0x11
#define CMD_LOITER_N_TURNS 0x12
#define CMD_LOITER_TIME 0x13
#define CMD_RTL 0x14
#define CMD_LAND 0x15
#define CMD_TAKEOFF 0x16
#define CMD_ALTITUDE 0x17
#define CMD_R_WAYPOINT 0x18

// Command IDs - May
#define CMD_DELAY 0x20
#define CMD_CLIMB 0x21 // NOT IMPLEMENTED
#define CMD_LAND_OPTIONS 0x22			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0 
#define CMD_ANGLE 0x23	// move servo N to PWM value

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
#define MSG_COMMAND 0x22
#define MSG_VALUE 0x32
#define MSG_PID 0x42
#define MSG_TRIMS 0x50
#define MSG_MINS 0x51
#define MSG_MAXS 0x52
#define MSG_IMU_OUT 0x53

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
#define LOG_CURRENT_MSG 		0x09
#define LOG_STARTUP_MSG 		0x0A
#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01

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

#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO

#define CURRENT_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*CURR_VOLT_DIV_RATIO
#define CURRENT_AMPS(x) (x*(INPUT_VOLTAGE/1024.0))*CURR_AMP_DIV_RATIO

#define	AIRSPEED_CH 7			// The external ADC channel for the airspeed sensor
#define BATTERY_PIN1 0		        // These are the pins for the voltage dividers
#define BATTERY_PIN2 1
#define BATTERY_PIN3 2
#define BATTERY_PIN4 3
#define CURRENT_PIN1 4 // These are the pins for current sensor: voltage
#define CURRENT_PIN2 5 // and current

#define RELAY_PIN 47


// sonar
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters

// Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41

#define A_LED_PIN 37			//36 = B,	37 = A,	35 = C
#define B_LED_PIN 36
#define C_LED_PIN 35

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 418

#define ToRad(x) (x * 0.01745329252)	// *pi/180
//#define ToDeg(x) (x * 57.2957795131)	// *180/pi


// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
#define PID_SIZE 8
#define RADIO_SIZE 6

// Radio setup
#define EE_RADIO_1 0x00	// all gains stored from here
#define EE_RADIO_2 0x06	// all gains stored from here
#define EE_RADIO_3 0x0C	// all gains stored from here
#define EE_RADIO_4 0x12	// all gains stored from here
#define EE_RADIO_5 0x18	// all gains stored from here
#define EE_RADIO_6 0x1E	// all gains stored from here
#define EE_RADIO_7 0x24	// all gains stored from here
#define EE_RADIO_8 0x2A	// all gains stored from here
#define EE_RADIO_9 0xD2		// camera pitch
#define EE_RADIO_10 0xD8	// camera roll

// user gains
#define EE_XTRACK_GAIN 0x30
#define EE_XTRACK_ANGLE 0x32
#define EE_PITCH_MAX 0x34
#define EE_DISTANCE_GAIN 0x36
#define EE_ALTITUDE_GAIN 0x38

#define EE_GAIN_1 0x40	// all gains stored from here
#define EE_GAIN_2 0x48	// all gains stored from here
#define EE_GAIN_3 0x50	// all gains stored from here
#define EE_GAIN_4 0x58	// all gains stored from here
#define EE_GAIN_5 0x60	// all gains stored from here
#define EE_GAIN_6 0x68	// all gains stored from here
#define EE_GAIN_7 0x70	// all gains stored from here
#define EE_GAIN_8 0x78	// all gains stored from here
#define EE_GAIN_9 0x80	// all gains stored from here
#define EE_GAIN_10 0x88	// all gains stored from here

#define EE_STAB_DAMPENER 0xA0
#define EE_HOLD_YAW_DAMPENER 0xA2

#define EE_MAG_DECLINATION 0xA8
#define EE_MAG_X 0xAA
#define EE_MAG_Y 0xAC
#define EE_MAG_Z 0xAE
#define EE_COMPASS 0xAF
#define EE_FRAME 0xB1

#define EE_IMU_OFFSET 0xE0

//mission specific
#define EE_CONFIG 0X0F8
#define EE_WP_TOTAL 0x0FB
#define EE_WP_INDEX 0x0FC
#define EE_WP_RADIUS 0x0FD
#define EE_LOITER_RADIUS 0x0FE
#define EE_ALT_HOLD_HOME 0x0FF

// user configs
#define EE_THROTTLE_MIN 0x103
#define EE_THROTTLE_CRUISE 0x105
#define EE_THROTTLE_MAX 0x107
#define EE_THROTTLE_FAILSAFE 0x10D
#define EE_THROTTLE_FS_VALUE 0x10E
#define EE_THROTTLE_FAILSAFE_ACTION 0x110
#define EE_LOG_BITMASK 0x114
#define EE_FLIGHT_MODES 0x121

// sensors
#define EE_ABS_PRESS_GND 0x116
#define EE_GND_TEMP 0x11A
#define EE_GND_ALT 0x11C
#define EE_AP_OFFSET 0x11E
#define EE_CURRENT_SENSOR 0x127

// log
#define EE_LAST_LOG_PAGE 0xE00
#define EE_LAST_LOG_NUM 0xE02
#define EE_LOG_1_START 0xE04

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
#define LOGBIT_CURRENT			(1<<9)

