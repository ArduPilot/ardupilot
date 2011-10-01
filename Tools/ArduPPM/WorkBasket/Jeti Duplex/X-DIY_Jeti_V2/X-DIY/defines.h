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

#define WP_START_BYTE 0x130	// where in memory home WP is stored + all other WP
#define WP_SIZE 14

// GCS enumeration
#define GCS_PROTOCOL_STANDARD	0	// standard APM protocol
#define GCS_PROTOCOL_SPECIAL	1	// special test protocol (?)
#define GCS_PROTOCOL_LEGACY		2	// legacy ArduPilot protocol
#define GCS_PROTOCOL_XPLANE		3	// X-Plane HIL simulation
#define GCS_PROTOCOL_IMU		4	// ArdiPilot IMU output
#define GCS_PROTOCOL_JASON		5	// Jason's special secret GCS protocol
#define GCS_PROTOCOL_DEBUGTERMINAL	6	//Human-readable debug interface for use with a dumb terminal
#define GCS_PROTOCOL_XDIY		7	// X-DIY custom protocol
#define GCS_PROTOCOL_NONE		-1	// No GCS output

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

// IMU Parameters

#define ADC_CONSTRAINT 900
#define TRUE 1
#define FALSE 0
#define ADC_WARM_CYCLES 200
#define SPEEDFILT 400			// centimeters/second

#define GYRO_TEMP_CH 3			// The ADC channel reading the gyro temperature

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 418
#define GRAVITY 418 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

// IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
#define Gyro_Gain_X 0.4 //X axis Gyro gain
#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
#define Gyro_Gain_Z 0.41 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.0014	 		// Pitch&Roll Drift Correction Proportional Gain
#define Ki_ROLLPITCH 0.0000003 		// Pitch&Roll Drift Correction Integrator Gain
#define Kp_YAW 0.8		 			// Yaw Drift Correction Porportional Gain	
#define Ki_YAW 0.00004 				// Yaw Drift CorrectionIntegrator Gain

/*For debugging purposes*/
#define OUTPUTMODE 1	//If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 Accel only data


#define EEPROM_MAX_ADDR		4096

// Radio setup
#define EE_TRIM 0x00
#define EE_MIN 0x10
#define EE_MAX 0x20
#define EE_ELEVON1_TRIM 0x30
#define EE_ELEVON2_TRIM 0x32

// user gains
#define EE_XTRACK_GAIN 0x34
#define EE_XTRACK_ANGLE 0x36
#define EE_ALT_MIX 0x3B
#define EE_HEAD_MAX 0x38
#define EE_PITCH_MAX 0x39
#define EE_PITCH_MIN 0x3A
#define EE_KP 0x40
#define EE_KI 0x60
#define EE_KD 0x80
#define EE_IMAX 0xA0
#define EE_KFF 0xC0
#define EE_AN_OFFSET 0xE0

//mission specific
#define EE_CONFIG 0X0F8
#define EE_WP_MODE 0x0F9
#define EE_YAW_MODE 0x0FA	// not used
#define EE_WP_TOTAL 0x0FB
#define EE_WP_INDEX 0x0FC
#define EE_WP_RADIUS 0x0FD
#define EE_LOITER_RADIUS 0x0FE
#define EE_ALT_HOLD_HOME 0x0FF

// user configs
#define EE_AIRSPEED_CRUISE 0x103
#define EE_AIRSPEED_RATIO 0x104
#define EE_AIRSPEED_FBW_MIN 0x108
#define EE_AIRSPEED_FBW_MAX 0x109
#define EE_THROTTLE_MIN 0x10A
#define EE_THROTTLE_CRUISE 0x10B
#define EE_THROTTLE_MAX 0x10C
#define EE_THROTTLE_FAILSAFE 0x10D
#define EE_THROTTLE_FS_VALUE 0x10E
#define EE_THROTTLE_FAILSAFE_ACTION 0x110
#define EE_FLIGHT_MODE_CHANNEL 0x112
#define EE_AUTO_TRIM 0x113
#define EE_LOG_BITMASK 0x114
#define EE_REVERSE_SWITCH 0x120
#define EE_FLIGHT_MODES 0x121

// sensors
#define EE_ABS_PRESS_GND 0x116
#define EE_GND_TEMP 0x11A
#define EE_GND_ALT 0x11C
#define EE_AP_OFFSET 0x11E

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

