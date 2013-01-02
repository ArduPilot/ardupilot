// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Example config file. Take a look at config.h. Any term define there can be
// overridden by defining it here.


// If you used to define your CONFIG_APM_HARDWARE setting here, it is no longer
// valid! You should switch to using a HAL_BOARD flag in your local config.mk.

//#define MAG_ORIENTATION		AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
//#define HIL_MODE				HIL_MODE_ATTITUDE
//#define DMP_ENABLED ENABLED
//#define SECONDARY_DMP_ENABLED ENABLED       // allows running DMP in parallel with DCM for testing purposes

//#define FRAME_CONFIG QUAD_FRAME
/*
 *  options:
 *  QUAD_FRAME
 *  TRI_FRAME
 *  HEXA_FRAME
 *  Y6_FRAME
 *  OCTA_FRAME
 *  OCTA_QUAD_FRAME
 *  HELI_FRAME
 */

//#define FRAME_ORIENTATION X_FRAME
/*
 *  PLUS_FRAME
 *  X_FRAME
 *  V_FRAME
 */

//#define CH7_OPTION		CH7_SAVE_WP
/*
 *  CH7_DO_NOTHING
 *  CH7_FLIP
 *  CH7_SIMPLE_MODE
 *  CH7_RTL
 *  CH7_SAVE_TRIM
 *  CH7_SAVE_WP
 *  CH7_CAMERA_TRIGGER
 */

// Inertia based contollers
//#define INERTIAL_NAV_XY ENABLED
#define INERTIAL_NAV_Z ENABLED

//#define MOTORS_JD880
//#define MOTORS_JD850


// the choice of function names is up to the user and does not have to match these
// uncomment these hooks and ensure there is a matching function on your "UserCode.pde" file
//#define USERHOOK_FASTLOOP userhook_FastLoop();
#define USERHOOK_50HZLOOP userhook_50Hz();
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();
#define USERHOOK_INIT userhook_init();

// the choice of included variables file (*.h) is up to the user and does not have to match this one
// Ensure the defined file exists and is in the arducopter directory
#define USERHOOK_VARIABLES "UserVariables.h"

//#define LOGGING_ENABLED		DISABLED

#define LOITER_REPOSITIONING    DISABLED                          // Experimental Do Not Use
// #define LOITER_RP               ROLL_PITCH_LOITER_PR

