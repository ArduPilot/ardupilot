// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

//Ya bla,bla,bla.. HAL, open the pod bay door.... Sorry Dave, I am afraid cannot do that.
#define CONFIG_HAL_BOARD  HAL_BOARD_APM1
////////////////////////////////////////

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

//#define HIL_MODE            HIL_MODE_DISABLED

/*
 *  // HIL_MODE SELECTION
 *  //
 *  // Mavlink supports
 *  // 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
 *  // 2. HIL_MODE_SENSORS: full sensor simulation
 *  //#define HIL_MODE            HIL_MODE_ATTITUDE
 *
 */
#define HIL_MODE            HIL_MODE_ATTITUDE
//#define HIL_MODE  HIL_MODE_SENSORS

# define CLI_ENABLED  DISABLED
# define MOUNT	DISABLED
# define LOGGING_ENABLED  DISABLED
//**************these are on uart3, DO NOT USE FOR FLIGHT! WILL HOSE YOUR TELEM
# define DEBUG_NAV  DISABLED  
# define DEBUG_NAV_A ENABLED
