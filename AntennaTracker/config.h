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

#define CONFIG_INS_TYPE HAL_INS_DEFAULT
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
// Developer Items
//

// use this to completely disable the CLI
#ifndef CLI_ENABLED
 # define CLI_ENABLED ENABLED
#endif

