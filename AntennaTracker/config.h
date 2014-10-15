// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
#include "defines.h"

#include "APM_Config.h" // <== THIS INCLUDE, DO NOT EDIT IT. EVER.

///
/// DO NOT EDIT THIS INCLUDE - if you want to make a local change, make that
/// change in your local copy of APM_Config.h.
///

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

#define CONFIG_BARO     HAL_BARO_DEFAULT
#define CONFIG_COMPASS  HAL_COMPASS_DEFAULT

#ifdef HAL_SERIAL0_BAUD_DEFAULT
# define SERIAL0_BAUD HAL_SERIAL0_BAUD_DEFAULT
#endif

#ifndef MAV_SYSTEM_ID
 // use 2 for antenna tracker by default
 # define MAV_SYSTEM_ID          2
#endif

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
#ifndef SERIAL0_BAUD
 # define SERIAL0_BAUD                   115200
#endif
#ifndef SERIAL1_BAUD
 # define SERIAL1_BAUD                    57600
#endif
#ifndef SERIAL2_BAUD
 # define SERIAL2_BAUD                    57600
#endif

#ifndef SERIAL_BUFSIZE
 # define SERIAL_BUFSIZE 512
#endif

#ifndef SERIAL1_BUFSIZE
 # define SERIAL1_BUFSIZE 256
#endif

#ifndef SERIAL2_BUFSIZE
 # define SERIAL2_BUFSIZE 256
#endif

//////////////////////////////////////////////////////////////////////////////
// RC Channel definitions
//
#ifndef CH_YAW
 # define CH_YAW        CH_1    // RC input/output for yaw on channel 1
#endif
#ifndef CH_PITCH
 # define CH_PITCH      CH_2    // RC input/output for pitch on channel 2
#endif


//////////////////////////////////////////////////////////////////////////////
// yaw and pitch axis angle range defaults
//
#ifndef YAW_RANGE_DEFAULT
 # define YAW_RANGE_DEFAULT 360
#endif
#ifndef PITCH_RANGE_DEFAULT
 # define PITCH_RANGE_DEFAULT 180
#endif

//////////////////////////////////////////////////////////////////////////////
// Tracking definitions
//
#ifndef TRACKING_TIMEOUT_MS
 # define TRACKING_TIMEOUT_MS               5000    // consider we've lost track of vehicle after 5 seconds with no position update.  Used to update armed/disarmed status leds
#endif
#ifndef TRACKING_TIMEOUT_SEC
 # define TRACKING_TIMEOUT_SEC              5.0f    // consider we've lost track of vehicle after 5 seconds with no position update.
#endif

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

// use this to completely disable the CLI
#ifndef CLI_ENABLED
 # define CLI_ENABLED ENABLED
#endif

