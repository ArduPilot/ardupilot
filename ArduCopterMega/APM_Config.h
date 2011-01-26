// Example config file. Use APM_Config.h.reference and the wiki to find additional defines to setup your plane.
// Once you upload the code, run the factory "reset" to save all config values to EEPROM.
// After reset, use the setup mode to set your radio limits for CH1-4, and to set your flight modes.

#define GPS_PROTOCOL  	    GPS_PROTOCOL_MTK
#define GCS_PROTOCOL        GCS_PROTOCOL_NONE

//#define MAGORIENTATION	AP_COMPASS_COMPONENTS_UP_PINS_BACK
//#define MAGORIENTATION	AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
#define MAGORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD


//#define ACRO_RATE_TRIGGER 4200
// if you want full ACRO mode, set value to 0
// if you want safe ACRO mode, set value to 100
// if you want mostly stabilize with flips, set value to 4200



// For future development, don't enable unless you know them
// These are all experimental and underwork, jp 23-12-10
//#define ENABLE_EXTRAS     ENABLED
//#define ENABLE_EXTRAINIT  ENABLED
//#define ENABLE_CAM        ENABLED
//#define ENABLE_AM         ENABLED
//#define ENABLE_xx         ENABLED

 
// Logging
//#define LOG_CURRENT ENABLED


