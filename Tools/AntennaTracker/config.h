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

//////////////////////////////////////////////////////////////////////////////
// main board differences
//
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 # define CONFIG_INS_TYPE CONFIG_INS_OILPAN
 # define CONFIG_BARO     AP_BARO_BMP085
 # define CONFIG_COMPASS  AP_COMPASS_HMC5843
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define CONFIG_INS_TYPE CONFIG_INS_MPU6000
 # define CONFIG_BARO          AP_BARO_MS5611
 # define CONFIG_MS5611_SERIAL AP_BARO_MS5611_SPI
 # define CONFIG_COMPASS  AP_COMPASS_HMC5843
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define CONFIG_INS_TYPE CONFIG_INS_HIL
 # define CONFIG_BARO     AP_BARO_HIL
 # define CONFIG_COMPASS  AP_COMPASS_HIL
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 # define CONFIG_INS_TYPE CONFIG_INS_PX4
 # define CONFIG_BARO AP_BARO_PX4
 # define CONFIG_COMPASS  AP_COMPASS_PX4
 # define SERIAL0_BAUD 115200
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
 # define CONFIG_INS_TYPE CONFIG_INS_FLYMAPLE
 # define CONFIG_BARO AP_BARO_BMP085
 # define CONFIG_COMPASS  AP_COMPASS_HMC5843
 # define SERIAL0_BAUD 115200
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
 # define CONFIG_INS_TYPE CONFIG_INS_L3G4200D
 # define CONFIG_BARO     AP_BARO_BMP085
 # define CONFIG_COMPASS  AP_COMPASS_HMC5843
#endif


#ifndef CONFIG_BARO
 # error "CONFIG_BARO not set"
#endif

#ifndef CONFIG_COMPASS
 # error "CONFIG_COMPASS not set"
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
#ifndef SERIAL3_BAUD
 # define SERIAL3_BAUD                    57600
#endif


#ifndef SERIAL_BUFSIZE
 # define SERIAL_BUFSIZE 512
#endif

#ifndef SERIAL2_BUFSIZE
 # define SERIAL2_BUFSIZE 256
#endif
