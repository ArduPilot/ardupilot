#pragma once

#include "defines.h"

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_APM_HARDWARE
#error CONFIG_APM_HARDWARE option is depreated! use CONFIG_HAL_BOARD instead.
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// Advanced Failsafe support
//

#ifndef ADVANCED_FAILSAFE
 # define ADVANCED_FAILSAFE ENABLED
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


#ifndef FLAP_1_PERCENT
 # define FLAP_1_PERCENT 0
#endif
#ifndef FLAP_1_SPEED
 # define FLAP_1_SPEED 0
#endif
#ifndef FLAP_2_PERCENT
 # define FLAP_2_PERCENT 0
#endif
#ifndef FLAP_2_SPEED
 # define FLAP_2_SPEED 0
#endif
//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
// FLIGHT_MODE_CHANNEL
//
#ifndef FLIGHT_MODE_CHANNEL
 # define FLIGHT_MODE_CHANNEL    8
#endif
#if (FLIGHT_MODE_CHANNEL != 5) && (FLIGHT_MODE_CHANNEL != 6) && (FLIGHT_MODE_CHANNEL != 7) && (FLIGHT_MODE_CHANNEL != 8)
 # error XXX
 # error XXX You must set FLIGHT_MODE_CHANNEL to 5, 6, 7 or 8
 # error XXX
#endif

#if !defined(FLIGHT_MODE_1)
 # define FLIGHT_MODE_1                  Mode::Number::RTL
#endif
#if !defined(FLIGHT_MODE_2)
 # define FLIGHT_MODE_2                  Mode::Number::RTL
#endif
#if !defined(FLIGHT_MODE_3)
 # define FLIGHT_MODE_3                  Mode::Number::FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_4)
 # define FLIGHT_MODE_4                  Mode::Number::FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_5)
 # define FLIGHT_MODE_5                  Mode::Number::MANUAL
#endif
#if !defined(FLIGHT_MODE_6)
 # define FLIGHT_MODE_6                  Mode::Number::MANUAL
#endif


//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM
//
#ifndef AUTO_TRIM
 # define AUTO_TRIM                              DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY
//
#ifndef GROUND_START_DELAY
 # define GROUND_START_DELAY             0
#endif

#ifndef DSPOILR_RUD_RATE_DEFAULT
 #define DSPOILR_RUD_RATE_DEFAULT 100
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE
//
#ifndef AIRSPEED_CRUISE
 # define AIRSPEED_CRUISE                12 // 12 m/s
#endif
#define AIRSPEED_CRUISE_CM AIRSPEED_CRUISE*100


//////////////////////////////////////////////////////////////////////////////
// MIN_GNDSPEED
//
#ifndef MIN_GNDSPEED
 # define MIN_GNDSPEED                   0 // m/s (0 disables)
#endif
#define MIN_GNDSPEED_CM MIN_GNDSPEED*100


//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control
//
#ifndef AIRSPEED_FBW_MIN
 # define AIRSPEED_FBW_MIN               9
#endif
#ifndef AIRSPEED_FBW_MAX
 # define AIRSPEED_FBW_MAX               22
#endif

#ifndef ALT_HOLD_FBW
 # define ALT_HOLD_FBW 0
#endif
#define ALT_HOLD_FBW_CM ALT_HOLD_FBW*100


//////////////////////////////////////////////////////////////////////////////
// Servo Mapping
//
#ifndef THROTTLE_MIN
 # define THROTTLE_MIN                   0 // percent
#endif
#ifndef THROTTLE_CRUISE
 # define THROTTLE_CRUISE                45
#endif
#ifndef THROTTLE_MAX
 # define THROTTLE_MAX                   100
#endif

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
#ifndef HEAD_MAX
 # define HEAD_MAX                               45
#endif
#ifndef PITCH_MAX
 # define PITCH_MAX                              20
#endif
#ifndef PITCH_MIN
 # define PITCH_MIN                              -25
#endif
#define HEAD_MAX_CENTIDEGREE HEAD_MAX * 100
#define PITCH_MAX_CENTIDEGREE PITCH_MAX * 100
#define PITCH_MIN_CENTIDEGREE PITCH_MIN * 100

#ifndef RUDDER_MIX
 # define RUDDER_MIX           0.5f
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Logging control
//

#ifndef LOGGING_ENABLED
 # define LOGGING_ENABLED                ENABLED
#endif

#define DEFAULT_LOG_BITMASK   0xffff


//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
#ifndef WP_RADIUS_DEFAULT
 # define WP_RADIUS_DEFAULT              90
#endif

#ifndef LOITER_RADIUS_DEFAULT
 # define LOITER_RADIUS_DEFAULT 60
#endif

#ifndef ALT_HOLD_HOME
 # define ALT_HOLD_HOME 100
#endif
#define ALT_HOLD_HOME_CM ALT_HOLD_HOME*100

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

#ifndef SCALING_SPEED
 # define SCALING_SPEED          15.0
#endif

// a digital pin to set high when the geo-fence triggers. Defaults
// to -1, which means don't activate a pin
#ifndef FENCE_TRIGGERED_PIN
 # define FENCE_TRIGGERED_PIN -1
#endif

//////////////////////////////////////////////////////////////////////////////
// Parachute release
#ifndef PARACHUTE
#define PARACHUTE HAL_PARACHUTE_ENABLED
#endif

#ifndef STATS_ENABLED
 # define STATS_ENABLED ENABLED
#endif

#ifndef OSD_ENABLED
 #define OSD_ENABLED DISABLED
#endif

#ifndef OFFBOARD_GUIDED
 #define OFFBOARD_GUIDED !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
#ifndef FS_EKF_THRESHOLD_DEFAULT
 # define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

/////////////////////////////////////////////////////////////////////////////
//  Landing Throttle Control Trigger Threshold
#ifndef THR_CTRL_LAND_THRESH
 #define THR_CTRL_LAND_THRESH 0.7
#endif
