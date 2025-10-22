// User specific config file.  Any items listed in config.h can be overridden here.

// uncomment the lines below to disable features (flash sizes listed are for APM2 boards and will underestimate savings on Pixhawk and other boards)
//#define LOGGING_ENABLED       0            // disable logging to save 11K of flash space
//#define MOUNT                 0            // disable the camera gimbal to save 8K of flash space
//#define AUTOTUNE_ENABLED      0            // disable the auto tune functionality to save 7k of flash
//#define NAV_GUIDED            0            // disable external navigation computer ability to control vehicle through MAV_CMD_NAV_GUIDED mission commands
//#define MODE_ACRO_ENABLED     0            // disable acrobatic mode support
//#define MODE_AUTO_ENABLED     0            // disable auto mode support
//#define MODE_BRAKE_ENABLED    0            // disable brake mode support
//#define MODE_CIRCLE_ENABLED   0            // disable circle mode support
//#define MODE_DRIFT_ENABLED    0            // disable drift mode support
//#define MODE_FLIP_ENABLED     0            // disable flip mode support
//#define MODE_FOLLOW_ENABLED   0            // disable follow mode support
//#define MODE_GUIDED_ENABLED   0            // disable guided mode support
//#define MODE_GUIDED_NOGPS_ENABLED   0      // disable guided/nogps mode support
//#define MODE_LOITER_ENABLED   0            // disable loiter mode support
//#define MODE_POSHOLD_ENABLED  0            // disable poshold mode support
//#define MODE_RTL_ENABLED      0            // disable rtl mode support
//#define MODE_SMARTRTL_ENABLED 0            // disable smartrtl mode support
//#define MODE_SPORT_ENABLED    0            // disable sport mode support
//#define MODE_SYSTEMID_ENABLED 0            // disable system ID mode support
//#define MODE_THROW_ENABLED    0            // disable throw mode support
//#define MODE_ZIGZAG_ENABLED   0            // disable zigzag mode support
//#define OSD_ENABLED           0            // disable on-screen-display support

// features below are disabled by default on all boards
//#define CAL_ALWAYS_REBOOT                         // flight controller will reboot after compass or accelerometer calibration completes
//#define DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE   // disable mode changes from GCS during Radio failsafes.  Avoids a race condition for vehicle like Solo in which the RC and telemetry travel along the same link
//#define AP_COPTER_ADVANCED_FAILSAFE_ENABLED     1             // enabled advanced failsafe which allows running a portion of the mission in failsafe events

// other settings
//#define THROTTLE_IN_DEADBAND   100                // redefine size of throttle deadband in pwm (0 ~ 1000)

// User Hooks : For User Developed code that you wish to run
// Put your variable definitions into the UserVariables.h file (or another file name and then change the #define below).
//#define USERHOOK_VARIABLES "UserVariables.h"
// Put your custom code into the UserCode.cpp with function names matching those listed below and ensure the appropriate #define below is uncommented below
//#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
//#define USERHOOK_AUXSWITCH 1                        // for code to handle user aux switches
//#define USER_PARAMS_ENABLED 1                       // to enable user parameters
