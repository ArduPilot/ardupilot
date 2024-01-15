#pragma once

#include "defines.h"

#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID    1
#endif

#ifndef ARM_DELAY_MS
  #define ARM_DELAY_MS  2000
#endif

//////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
//

#ifndef CH7_OPTION
  #define CH7_OPTION CH7_SAVE_WP
#endif

#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
//#define USERHOOK_AUXSWITCH ENABLED                        // for code to handle user aux switches
//#define USER_PARAMS_ENABLED ENABLED                       // to enable user parameters



//////////////////////////////////////////////////////////////////////////////
// MODE
// MODE_CHANNEL
//
#ifndef MODE_CHANNEL
  #define MODE_CHANNEL    8
#endif
#if (MODE_CHANNEL != 5) && (MODE_CHANNEL != 6) && (MODE_CHANNEL != 7) && (MODE_CHANNEL != 8)
  #error XXX
  #error XXX You must set MODE_CHANNEL to 5, 6, 7 or 8
  #error XXX
#endif

//////////////////////////////////////////////////////////////////////////////
// RALLY POINTS
//
#ifndef AP_RALLY
 #define AP_RALLY ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Precision Landing with companion computer or IRLock sensor
#ifndef PRECISION_LANDING
 # define PRECISION_LANDING ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// NAVL1
//
#ifndef NAVL1
  #define NAVL1_PERIOD    8
#endif

//////////////////////////////////////////////////////////////////////////////
// CRUISE_SPEED default
//
#ifndef CRUISE_SPEED
  #define CRUISE_SPEED    2  // in m/s
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//
#ifndef LOGGING_ENABLED
  #define LOGGING_ENABLED ENABLED
#endif

#define DEFAULT_LOG_BITMASK    0xffff

//////////////////////////////////////////////////////////////////////////////
// Dock mode - allows vehicle to dock to a docking target
#ifndef MODE_DOCK_ENABLED
# define MODE_DOCK_ENABLED PRECISION_LANDING
#endif


//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

// if RESET_SWITCH_CH is not zero, then this is the PWM value on
// that channel where we reset the control mode to the current switch
// position (to for example return to switched mode after failsafe or
// fence breach)
#ifndef RESET_SWITCH_CHAN_PWM
  #define RESET_SWITCH_CHAN_PWM    1750
#endif

#ifndef ADVANCED_FAILSAFE
  #define ADVANCED_FAILSAFE DISABLED
#endif

#ifndef STATS_ENABLED
 # define STATS_ENABLED ENABLED
#endif

#ifndef OSD_ENABLED
 #define OSD_ENABLED DISABLED
#endif

