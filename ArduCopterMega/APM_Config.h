// Example config file. Use APM_Config.h.reference and the wiki to find additional defines to setup your plane.
// Once you upload the code, run the factory "reset" to save all config values to EEPROM.
// After reset, use the setup mode to set your radio limits for CH1-4, and to set your flight modes.

#define GPS_PROTOCOL  		GPS_PROTOCOL_MTK
#define GCS_PROTOCOL        GCS_PROTOCOL_NONE

#define MAGORIENTATION	AP_COMPASS_COMPONENTS_UP_PINS_BACK
#define ARM_AT_STARTUP	0
//#define MAGOFFSET		30.50,15.00,47.00
//#define DECLINATION		14.2


// For future development, don't enable unless you know them
// These are all experimental and underwork, jp 23-12-10
//#define ENABLE_EXTRAS     ENABLED
//#define ENABLE_EXTRAINIT  ENABLED
#define ENABLE_CAM        ENABLED
//#define ENABLE_AM         ENABLED
//#define ENABLE_xx         ENABLED

