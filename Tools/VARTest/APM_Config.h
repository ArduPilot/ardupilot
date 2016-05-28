// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If you wish to change any of the setup parameters from
// their default values, place the appropriate #define statements here.

// For example if you wanted the Port 3 baud rate to be 38400 you would add a statement like the one below (uncommented)
//#define SERIAL3_BAUD        38400

// You may also put an include statement here to point at another configuration file.  This is convenient if you maintain
// different configuration files for different aircraft or HIL simulation.  See the examples below
//#include "APM_Config_mavlink_hil.h"
//#include "Skywalker.h"

// The following are the recommended settings for Xplane simulation. Remove the leading "/* and trailing "*/" to enable:

/*
#define HIL_MODE            HIL_MODE_ATTITUDE
*/

/*
// HIL_MODE SELECTION
//
// Mavlink supports
// 1. HIL_MODE_SENSORS: full sensor simulation
#define HIL_MODE            HIL_MODE_ATTITUDE

// Sensors
// All sensors are supported in all modes.
#define AIRSPEED_SENSOR     ENABLED
#define MAGNETOMETER        ENABLED
#define AIRSPEED_CRUISE     25
#define THROTTLE_FAILSAFE   ENABLED
*/
