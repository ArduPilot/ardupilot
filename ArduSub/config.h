#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduSub
#endif

// run at 400Hz on all systems
# define MAIN_LOOP_RATE    400

#ifndef SURFACE_DEPTH_DEFAULT
# define SURFACE_DEPTH_DEFAULT -10.0f // pressure sensor reading 10cm depth means craft is considered surfaced
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
// default RC speed in Hz
#ifndef RC_SPEED_DEFAULT
#   define RC_SPEED_DEFAULT 200
#endif

//////////////////////////////////////////////////////////////////////////////
// Circle Nav parameters
//

#ifndef CIRCLE_NAV_ENABLED
# define CIRCLE_NAV_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// RC
//

#ifndef AP_SUB_RC_ENABLED
# define AP_SUB_RC_ENABLED 1
#endif
#ifndef RCMAP_ENABLED
# define RCMAP_ENABLED AP_SUB_RC_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975
#endif


//////////////////////////////////////////////////////////////////////////////
// Rangefinder
//

#ifndef RANGEFINDER_HEALTH_MAX
# define RANGEFINDER_HEALTH_MAX 3          // number of good reads that indicates a healthy rangefinder
#endif

#ifndef RANGEFINDER_TIMEOUT_MS
# define RANGEFINDER_TIMEOUT_MS  1000      // desired rangefinder alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

#ifndef RANGEFINDER_WPNAV_FILT_HZ
# define RANGEFINDER_WPNAV_FILT_HZ   0.25f // filter frequency for rangefinder altitude provided to waypoint navigation class
#endif

#ifndef RANGEFINDER_TILT_CORRECTION        // by disable tilt correction for use of range finder data by EKF
# define RANGEFINDER_TILT_CORRECTION 0
#endif

#ifndef RANGEFINDER_SIGNAL_MIN_DEFAULT
# define RANGEFINDER_SIGNAL_MIN_DEFAULT 90 // rangefinder readings with signal quality below this value are ignored
#endif

#ifndef SURFTRAK_DEPTH_DEFAULT
# define SURFTRAK_DEPTH_DEFAULT -50.0f     // surftrak will try to keep the sub below this depth
#endif

// Avoidance (relies on Proximity and Fence)
#ifndef AVOIDANCE_ENABLED
# define AVOIDANCE_ENABLED 0
#endif

#if AVOIDANCE_ENABLED // Avoidance Library relies on Fence
# define FENCE_ENABLED 1
#endif

#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// Nav-Guided - allows external nav computer to control vehicle
#ifndef NAV_GUIDED
# define NAV_GUIDED    1
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight mode definitions
//

// Acro Mode
#ifndef ACRO_RP_P
# define ACRO_RP_P                 4.5f
#endif

#ifndef ACRO_YAW_P
# define ACRO_YAW_P                3.375f
#endif

#ifndef ACRO_LEVEL_MAX_ANGLE
# define ACRO_LEVEL_MAX_ANGLE      3000
#endif

#ifndef ACRO_BALANCE_ROLL
#define ACRO_BALANCE_ROLL          1.0f
#endif

#ifndef ACRO_BALANCE_PITCH
#define ACRO_BALANCE_PITCH         1.0f
#endif

#ifndef ACRO_EXPO_DEFAULT
#define ACRO_EXPO_DEFAULT          0.3f
#endif

// AUTO Mode
#ifndef WP_YAW_BEHAVIOR_DEFAULT
# define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_CORRECT_XTRACK
#endif

#ifndef AUTO_YAW_SLEW_RATE
# define AUTO_YAW_SLEW_RATE    60              // degrees/sec
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED
# define YAW_LOOK_AHEAD_MIN_SPEED  100             // minimum ground speed in cm/s required before vehicle is aimed at ground course
#endif

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#ifndef ROLL_PITCH_INPUT_MAX
# define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
#endif
#ifndef DEFAULT_ANGLE_MAX
# define DEFAULT_ANGLE_MAX         4500            // ANGLE_MAX parameters default value
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter position control gains
//
#ifndef POS_XY_P
# define POS_XY_P               1.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// PosHold parameter defaults
//
#ifndef POSHOLD_ENABLED
# define POSHOLD_ENABLED               1 // PosHold flight mode enabled by default
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//

#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

// default maximum velocities and acceleration the pilot may request
#ifndef PILOT_VELZ_MAX
# define PILOT_VELZ_MAX    500     // maximum vertical velocity in cm/s
#endif
#ifndef PILOT_SPEED_DEFAULT
# define PILOT_SPEED_DEFAULT 200 // maximum horizontal velocity in cm/s while under pilot control
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
# define PILOT_ACCEL_Z_DEFAULT 100 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  0
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//

// Default logging bitmask
#ifndef DEFAULT_LOG_BITMASK
# define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//Default flight modes
#ifndef FLIGHT_MODE_1
# define FLIGHT_MODE_1 Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_2
# define FLIGHT_MODE_2 Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_3
# define FLIGHT_MODE_3 Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_4
# define FLIGHT_MODE_4 Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_5
# define FLIGHT_MODE_5 Mode::Number::SURFACE
#endif
#ifndef FLIGHT_MODE_6
# define FLIGHT_MODE_6 Mode::Number::SURFACE
#endif

