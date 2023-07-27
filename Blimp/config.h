//
#pragma once

//////////////////////////////////////////////////////////////////////////////
//
// Default and automatic configuration details.
//
//
#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build Blimp
#endif

#ifndef ARMING_DELAY_SEC
# define ARMING_DELAY_SEC 2.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
// default RC speed in Hz
#ifndef RC_FAST_SPEED
#   define RC_FAST_SPEED 490
#endif

#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif

// prearm GPS hdop check
#ifndef GPS_HDOP_GOOD_DEFAULT
# define GPS_HDOP_GOOD_DEFAULT         140     // minimum hdop that represents a good position.  used during pre-arm checks if fence is enabled
#endif

// Radio failsafe while using RC_override
#ifndef FS_RADIO_RC_OVERRIDE_TIMEOUT_MS
# define FS_RADIO_RC_OVERRIDE_TIMEOUT_MS  1000    // RC Radio failsafe triggers after 1 second while using RC_override from ground station
#endif

// Radio failsafe
#ifndef FS_RADIO_TIMEOUT_MS
#define FS_RADIO_TIMEOUT_MS            500     // RC Radio Failsafe triggers after 500 milliseconds with No RC Input
#endif

// missing terrain data failsafe
#ifndef FS_TERRAIN_TIMEOUT_MS
#define FS_TERRAIN_TIMEOUT_MS          5000     // 5 seconds of missing terrain data will trigger failsafe (RTL)
#endif

// pre-arm baro vs inertial nav max alt disparity
#ifndef PREARM_MAX_ALT_DISPARITY_CM
# define PREARM_MAX_ALT_DISPARITY_CM       100     // barometer and inertial nav altitude must be within this many centimeters
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
#ifndef FS_EKF_ACTION_DEFAULT
# define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_LAND  // EKF failsafe triggers land by default
#endif
#ifndef FS_EKF_THRESHOLD_DEFAULT
# define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

#ifndef EKF_ORIGIN_MAX_DIST_M
# define EKF_ORIGIN_MAX_DIST_M         50000   // EKF origin and waypoints (including home) must be within 50km
#endif

//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
//
#ifndef FLIGHT_MODE_1
# define FLIGHT_MODE_1                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_2
# define FLIGHT_MODE_2                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_3
# define FLIGHT_MODE_3                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_4
# define FLIGHT_MODE_4                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_5
# define FLIGHT_MODE_5                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_6
# define FLIGHT_MODE_6                  Mode::Number::MANUAL
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
#ifndef FS_THR_VALUE_DEFAULT
# define FS_THR_VALUE_DEFAULT             975
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle control defaults
//
#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  10
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//
#ifndef LOGGING_ENABLED
# define LOGGING_ENABLED                ENABLED
#endif

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

#ifndef CH_MODE_DEFAULT
# define CH_MODE_DEFAULT   5
#endif

#ifndef STATS_ENABLED
# define STATS_ENABLED ENABLED
#endif

#ifndef HAL_FRAME_TYPE_DEFAULT
#define HAL_FRAME_TYPE_DEFAULT Fins::MOTOR_FRAME_TYPE_AIRFISH
#endif
