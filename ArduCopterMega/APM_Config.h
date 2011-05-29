// Example config file. Use APM_Config.h.reference and the wiki to find additional defines to setup your plane.
// Once you upload the code, run the factory "reset" to save all config values to EEPROM.
// After reset, use the setup mode to set your radio limits for CH1-4, and to set your flight modes.

// GPS is auto-selected

//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

#define NAV_TEST 1	// 0 = traditional, 1 = rate controlled
#define YAW_OPTION 1	// 0 = hybrid rate approach, 1 = offset Yaw approach
#define AUTO_RESET_LOITER 0 // enables Loiter to reset it's current location based on stick input.

#define FRAME_CONFIG QUAD_FRAME
	/*
	options:
	QUAD_FRAME
	TRI_FRAME
	HEXA_FRAME
	Y6_FRAME
	OCTA_FRAME
	*/

#define FRAME_ORIENTATION X_FRAME
	/*
	PLUS_FRAME
	X_FRAME
	*/


#define CHANNEL_6_TUNING CH6_NONE
	/*
	CH6_NONE
	CH6_STABLIZE_KP
	CH6_STABLIZE_KD
	CH6_BARO_KP
	CH6_BARO_KD
	CH6_SONAR_KP
	CH6_SONAR_KD
	CH6_Y6_SCALING
	*/

//#define ACRO_RATE_TRIGGER 4200
// if you want full ACRO mode, set value to 0
// if you want mostly stabilize with flips, set value to 4200

//#define STABILIZE_ROLL_D 		0.11
//#define STABILIZE_PITCH_D 		0.11


// Logging
//#define LOG_CURRENT ENABLED

# define LOG_ATTITUDE_FAST		DISABLED
# define LOG_ATTITUDE_MED 		DISABLED
# define LOG_GPS 				ENABLED
# define LOG_PM 				ENABLED
# define LOG_CTUN				ENABLED
# define LOG_NTUN				ENABLED
# define LOG_MODE				ENABLED
# define LOG_RAW				DISABLED
# define LOG_CMD				ENABLED
# define LOG_CURRENT			DISABLED



#define MOTOR_LEDS 1	// 0 = off, 1 = on

#define FR_LED AN12  // Mega PE4 pin, OUT7
#define RE_LED AN14  // Mega PE5 pin, OUT6
#define RI_LED AN10  // Mega PH4 pin, OUT5
#define LE_LED AN8  // Mega PH5 pin, OUT4
