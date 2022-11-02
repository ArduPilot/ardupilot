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
// CAMERA TRIGGER AND CONTROL
//
#ifndef CAMERA
 # define CAMERA         ENABLED
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

// use this to disable geo-fencing
#ifndef AC_FENCE
 # define AC_FENCE ENABLED
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

//////////////////////////////////////////////////////////////////////////////
// Payload Gripper
#ifndef GRIPPER_ENABLED
  #define GRIPPER_ENABLED !HAL_MINIMIZE_FEATURES
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

#ifndef LANDING_GEAR_ENABLED
 #define LANDING_GEAR_ENABLED !HAL_MINIMIZE_FEATURES
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

#if HAL_V22T_ENABLED
# define WP_YAW_BEHAVIOR_DEFAULT       3
# define GPS_HDOP_GOOD_DEFAULT         140     // minimum hdop that represents a good position.  used during pre-arm checks if fence is enabled
# define POSHOLD_BRAKE_RATE_DEFAULT    8       // default POSHOLD_BRAKE_RATE param value.  Rotation rate during braking in deg/sec
# define POSHOLD_BRAKE_ANGLE_DEFAULT   3000    // default POSHOLD_BRAKE_ANGLE param value.  Max lean angle during braking in centi-degrees
# define PILOT_ACCEL_Z_DEFAULT 250 // vertical acceleration in cm/s/s while altitude is under pilot control
# define LAND_REPOSITION_DEFAULT   1   // by default the pilot can override roll/pitch during landing
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
# define PILOT_TKOFF_ALT_DEFAULT           0     // default final alt above home for pilot initiated takeoff
# define AUTO_DISARMING_DELAY  10
# define DEFAULT_ANGLE_MAX         3000            // ANGLE_MAX parameters default value
# define RTL_CONE_SLOPE_DEFAULT    3.0f    // slope of RTL cone (height / distance). 0 = No cone
#define HAL_FRAME_TYPE_DEFAULT AP_Motors::MOTOR_FRAME_TYPE_X
# define RTL_ALT                   1500    // default alt to return to home in cm, 0 = Maintain current altitude
# define RTL_LOITER_TIME           5000    // Time (in milliseconds) to loiter above home before beginning final descent
# define RTL_ALT_FINAL             0       // the altitude, in cm, the vehicle will move to as the final stage of Returning to Launch.  Set to zero to land.
#define FS_THR_ENABLED_ALWAYS_RTL                  1
# define FS_THR_VALUE_DEFAULT             975
# define RC_FAST_SPEED                        125
#define FS_GCS_DISABLED                        0
# define CH_MODE_DEFAULT   5
# define LAND_SPEED    50          // the descent speed for the final stage of landing in cm/s
#define ACRO_BALANCE_ROLL          1.0f
#define ACRO_BALANCE_PITCH         1.0f
# define FS_EKF_ACTION_DEFAULT         1  // EKF failsafe triggers land by default
# define RTL_CLIMB_MIN_DEFAULT     0       // vehicle will always climb this many cm as first stage of RTL
# define PILOT_VELZ_MAX    250     // maximum vertical velocity in cm/s
#define FS_GCS_ENABLED_CONTINUE_MISSION        2
#define FS_THR_ENABLED_CONTINUE_MISSION            2    // Removed in 4.0+, now use fs_options
#define FS_GCS_ENABLED_ALWAYS_RTL              1

// helicopter
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#if FRAME_CONFIG == HELI_FRAME
#ifndef MODE_AUTOROTATE_ENABLED
# define MODE_AUTOROTATE_ENABLED !HAL_MINIMIZE_FEATURES
#endif
#else
# define MODE_AUTOROTATE_ENABLED DISABLED
#endif
#else
# define MODE_AUTOROTATE_ENABLED DISABLED
#endif

#endif