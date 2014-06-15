// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// User specific config file.  Any items listed in config.h can be overridden here.
// Select Megapirate board type:
#define MPNG_BOARD_TYPE   RCTIMER_CRIUS_V2
/*
  RCTIMER_CRIUS_V2    -- (DEFAULT) Use ONLY for RCTimer CRIUS V2 board
  CRIUS_V1            -- RCTimer CRIUS V1(1.1) board and all HobbyKing AIOP boards
  HK_RED_MULTIWII_PRO -- HobbyKing MultiWii Pro RED board with ITG3205 and BMA180, BMP085 sensors
  BLACK_VORTEX
  MULTIWII_PRO_EZ3_BLACK  -- ReadyToFlyQuads - MultiWii PRO Ez3.0 Blacked MAG Editon Flight Controller w/ GPS Option (NO COMPASS)
 */
//# define CONFIG_MS5611_SERIAL AP_BARO_MS5611_I2C
 
 // experimental serial sonar support on Serial 3
//#define CONFIG_SONAR ENABLED
//#define CONFIG_SONAR_SOURCE SONAR_SOURCE_SERIAL
//#define SERIAL3_BAUD 9600 

//else
#define SERIAL3_BAUD 57600 

#define SERIAL0_BAUD 115200
#define SERIAL1_BAUD 57600

// GPS port speed (Serial2) 38400 by default
#define SERIAL2_BAUD 38400

// GPS driver selection
#define GPS_PROTOCOL GPS_PROTOCOL_UBLOX
/*
	GPS_PROTOCOL_AUTO   (Default)
	GPS_PROTOCOL_NONE
	GPS_PROTOCOL_NMEA
	GPS_PROTOCOL_SIRF
	GPS_PROTOCOL_UBLOX
	GPS_PROTOCOL_IMU
	GPS_PROTOCOL_MTK
	GPS_PROTOCOL_HIL
	GPS_PROTOCOL_MTK19
*/
//disable serial PPM  --IF YOU HAVE PROBLEMS WITH below 2 lines - THEN SET IN ./libraries/AP_HAL_MPNG/RCInput_MPNG.cpp
# define SERIAL_PPM SERIAL_PPM_DISABLED
#define RC_MAPPING RC_MAP_MULTIWII

// QuadCopter selected by default
#define FRAME_CONFIG QUAD_FRAME
/*
 *  options:
 *  QUAD_FRAME
 *  TRI_FRAME
 *  HEXA_FRAME
 *  Y6_FRAME
 *  OCTA_FRAME
 *  OCTA_QUAD_FRAME
 *  HELI_FRAME
 *  SINGLE_FRAME
 */

 /*  NOTE This project is called Bleeducopter for a reason! We are seriously pushing what an 8-bit CPU can do. Enabling too many options below will cause slow scheduler loops. 
	When that happens the best case scenario is a FAILSAFE situation. Worst case is a crash or fly-away. This means - 


 				DISABLE ALL OPTIONS BELOW WHICH YOU ARE NOT USING TO SAVE CPU CYCLES!!!!!
 				ONLY ENABLE AUTOTUNE TEMPORARILY! SWITCH BACK TO A SAFER BULD WHEN DONE.
				DISABLE SONAR, LOGGING AND CLI FOR AUTOTUNE BUILDS!
				TEST YOUR BUILD EXTENSIVELY WITH PROPS OFF!
				BE CAREFUL WITH SONAR!
				DISABLE LOGGING UNLESS NEEDED!
				IF USING LOGGING, LOG ONLY THE DATA YOU NEED!

THIS FIRMWARE IS EXTREMELY EXPERIMENTAL! YOU HAVE BEEN WARNED.

*/				
 
//#define CLI_ENABLED           DISABLED            // disable the CLI (command-line-interface) to save 21K of flash space
#define LOGGING_ENABLED       DISABLED           // disable dataflash logging to save 11K of flash space
//#define GPS_PROTOCOL          GPS_PROTOCOL_UBLOX  // hard code GPS to Ublox to save 8k of flash
//#define GPS_PROTOCOL          GPS_PROTOCOL_MTK19  // hard cdoe GPS to Mediatek to save 10k of flash
//#define MOUNT                 DISABLED            // disable the camera gimbal to save 8K of flash space
#define AUTOTUNE              DISABLED            // disable the auto tune functionality to save 7k of flash
#define OPTFLOW               DISABLED            // disable optical flow sensor to save 5K of flash space
//#define AC_FENCE              DISABLED            // disable fence to save 2k of flash
//#define CAMERA                DISABLED            // disable camera trigger to save 1k of flash
#define CONFIG_SONAR          DISABLED            // disable sonar to save 1k of flash

// features below are disabled by default
//#define SPRAYER               ENABLED             // enable the crop sprayer feature (two ESC controlled pumps the speed of which depends upon the vehicle's horizontal velocity)
//#define EPM_ENABLED           ENABLED             // enable epm cargo gripper costs 500bytes of flash

// other settings
//#define THROTTLE_IN_DEADBAND   100                // redefine size of throttle deadband in pwm (0 ~ 1000)
//#define LAND_REQUIRE_MIN_THROTTLE_TO_DISARM   DISABLED    // when set to DISABLED vehicle will disarm after landing (in LAND mode or RTL) even if pilot has not put throttle to zero

//#define HIL_MODE              HIL_MODE_SENSORS    // build for hardware-in-the-loop simulation

// User Hooks : For User Developed code that you wish to run
// Put your variable definitions into the UserVariables.h file (or another file name and then change the #define below).
//#define USERHOOK_VARIABLES "UserVariables.h"
// Put your custom code into the UserCode.pde with function names matching those listed below and ensure the appropriate #define below is uncommented below
//#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
