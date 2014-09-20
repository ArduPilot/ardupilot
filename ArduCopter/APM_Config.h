// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// User specific config file.  Any items listed in config.h can be overridden here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no longer
// valid! You should switch to using a HAL_BOARD flag in your local config.mk.

//#define FRAME_CONFIG QUAD_FRAME
/*  options:
 *  QUAD_FRAME
 *  TRI_FRAME
 *  HEXA_FRAME
 *  Y6_FRAME
 *  OCTA_FRAME
 *  OCTA_QUAD_FRAME
 *  HELI_FRAME
 *  SINGLE_FRAME
 *  COAX_FRAME
 */

// uncomment the lines below to save on flash space if compiling for the APM using Arduino IDE.  Top items save the most flash space
//#define LOGGING_ENABLED       DISABLED            // disable dataflash logging to save 11K of flash space
//#define MOUNT                 DISABLED            // disable the camera gimbal to save 8K of flash space
//#define AUTOTUNE_ENABLED      DISABLED            // disable the auto tune functionality to save 7k of flash
//#define AC_FENCE              DISABLED            // disable fence to save 2k of flash
//#define CAMERA                DISABLED            // disable camera trigger to save 1k of flash
//#define CONFIG_SONAR          DISABLED            // disable sonar to save 1k of flash
//#define POSHOLD_ENABLED       DISABLED            // disable PosHold flight mode to save 4.5k of flash

// features below are disabled by default on APM (but enabled on Pixhawk)
//#define AC_RALLY              ENABLED             // disable rally points to save 2k of flash, and also frees rally point EEPROM for more mission commands
//#define PARACHUTE             ENABLED             // enable parachute release at a cost of 1k of flash
//#define EPM_ENABLED           ENABLED             // enable epm cargo gripper costs 500bytes of flash
//#define CLI_ENABLED           ENABLED             // enable the CLI (command-line-interface) at a cost of 21K of flash space

// features below are disabled by default on all boards
//#define OPTFLOW               ENABLED             // enable optical flow sensor and OF_LOITER flight mode at a cost of 5K of flash space
//#define SPRAYER               ENABLED             // enable the crop sprayer feature (two ESC controlled pumps the speed of which depends upon the vehicle's horizontal velocity)
//#define NAV_GUIDED            ENABLED             // enable external navigation computer to control vehicle through MAV_CMD_NAV_GUIDED mission commands

// other settings
//#define THROTTLE_IN_DEADBAND   100                // redefine size of throttle deadband in pwm (0 ~ 1000)
//#define LAND_REQUIRE_MIN_THROTTLE_TO_DISARM   DISABLED    // when set to DISABLED vehicle will disarm after landing (in LAND mode or RTL) even if pilot has not put throttle to zero
//#define LOG_FROM_STARTUP                          // start logging as soon as the board starts up instead of waiting for the vehicle to be armed

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
