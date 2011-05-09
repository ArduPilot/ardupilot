// Example config file. Use APM_Config.h.reference and the wiki to find additional defines to setup your plane.
// Once you upload the code, run the factory "reset" to save all config values to EEPROM.
// After reset, use the setup mode to set your radio limits for CH1-4, and to set your flight modes.

// GPS is auto-selected


#define NAV_TEST 0	// 0 = traditional, 1 = rate controlled

#define CHANNEL_6_TUNING CH6_NONE
//#define CHANNEL_6_TUNING CH6_PMAX

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

