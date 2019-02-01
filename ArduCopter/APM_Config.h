// User specific config file.  Any items listed in config.h can be overridden here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no longer
// valid! You should switch to using a HAL_BOARD flag in your local config.mk.

// uncomment the lines below to disable features (flash sizes listed are for APM2 boards and will underestimate savings on Pixhawk and other boards)
//#define LOGGING_ENABLED       DISABLED            // disable dataflash logging to save 11K of flash space
//#define MOUNT                 DISABLED            // disable the camera gimbal to save 8K of flash space
//#define AUTOTUNE_ENABLED      DISABLED            // disable the auto tune functionality to save 7k of flash
//#define AC_FENCE              DISABLED            // disable fence to save 2k of flash
//#define CAMERA                DISABLED            // disable camera trigger to save 1k of flash
//#define RANGEFINDER_ENABLED   DISABLED            // disable rangefinder to save 1k of flash
//#define PROXIMITY_ENABLED     DISABLED            // disable proximity sensors
//#define AC_RALLY              DISABLED            // disable rally points library (must also disable terrain which relies on rally)
//#define AC_AVOID_ENABLED      DISABLED            // disable stop-at-fence library
//#define AC_TERRAIN            DISABLED            // disable terrain library
//#define PARACHUTE             DISABLED            // disable parachute release to save 1k of flash
//#define NAV_GUIDED            DISABLED            // disable external navigation computer ability to control vehicle through MAV_CMD_NAV_GUIDED mission commands
//#define OPTFLOW               DISABLED            // disable optical flow sensor to save 5K of flash space
//#define VISUAL_ODOMETRY_ENABLED DISABLED          // disable visual odometry to save 2K of flash space
//#define FRSKY_TELEM_ENABLED   DISABLED            // disable FRSky telemetry
//#define ADSB_ENABLED          DISABLED            // disable ADSB support
//#define PRECISION_LANDING     DISABLED            // disable precision landing using companion computer or IRLock sensor
//#define BEACON_ENABLED        DISABLED            // disable beacon support
//#define SPRAYER_ENABLED       DISABLED            // disable the crop sprayer feature (two ESC controlled pumps the speed of which depends upon the vehicle's horizontal velocity)
//#define WINCH_ENABLED         DISABLED            // disable winch support
//#define GRIPPER_ENABLED       DISABLED            // disable gripper support
//#define RPM_ENABLED           DISABLED            // disable rotations per minute sensor support
//#define MAGNETOMETER          DISABLED            // disable magnetometer support
//#define STATS_ENABLED         DISABLED            // disable statistics support
//#define MODE_ACRO_ENABLED     DISABLED            // disable acrobatic mode support
//#define MODE_AUTO_ENABLED     DISABLED            // disable auto mode support
//#define MODE_BRAKE_ENABLED    DISABLED            // disable brake mode support
//#define MODE_CIRCLE_ENABLED   DISABLED            // disable circle mode support
//#define MODE_DRIFT_ENABLED    DISABLED            // disable drift mode support
//#define MODE_FLIP_ENABLED     DISABLED            // disable flip mode support
//#define MODE_FOLLOW_ENABLED   DISABLED            // disable follow mode support
//#define MODE_GUIDED_ENABLED   DISABLED            // disable guided mode support
//#define MODE_GUIDED_NOGPS_ENABLED   DISABLED      // disable guided/nogps mode support
//#define MODE_LOITER_ENABLED   DISABLED            // disable loiter mode support
//#define MODE_POSHOLD_ENABLED  DISABLED            // disable poshold mode support
//#define MODE_RTL_ENABLED DISABLED                 // disable rtl mode support
//#define MODE_SMARTRTL_ENABLED DISABLED            // disable smartrtl mode support
//#define MODE_SPORT_ENABLED DISABLED               // disable sport mode support
//#define MODE_THROW_ENABLED    DISABLED            // disable throw mode support
//#define MODE_ZIGZAG_ENABLED   DISABLED            // disable zigzag mode support
//#define DEVO_TELEM_ENABLED DISABLED               // disable DEVO telemetry, if you don't use Walkera RX-707 (or newer) receivers
//#define MODE_HOLD_ULTRA_ENABLED DISABLED          // disable indoor hold with ultrasonic


// features below are disabled by default on all boards
//#define CAL_ALWAYS_REBOOT                         // flight controller will reboot after compass or accelerometer calibration completes
//#define DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE   // disable mode changes from GCS during Radio failsafes.  Avoids a race condition for vehicle like Solo in which the RC and telemetry travel along the same link
//#define ADVANCED_FAILSAFE     ENABLED             // enabled advanced failsafe which allows running a portion of the mission in failsafe events

// other settings
//#define THROTTLE_IN_DEADBAND   100                // redefine size of throttle deadband in pwm (0 ~ 1000)

//#define HIL_MODE              HIL_MODE_SENSORS    // build for hardware-in-the-loop simulation

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
//#define USERHOOK_AUXSWITCH ENABLED                        // for code to handle user aux switches
//#define USER_PARAMS_ENABLED ENABLED                       // to enable user parameters
