// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _DEFINES_H
#define _DEFINES_H

#include <AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// Flight modes
// ------------
#define AUTO_YAW_HOLD                   0       // pilot controls the heading
#define AUTO_YAW_LOOK_AT_NEXT_WP        1       // point towards next waypoint (no pilot input accepted)
#define AUTO_YAW_ROI                    2       // point towards a location held in roi_WP (no pilot input accepted)
#define AUTO_YAW_LOOK_AT_HEADING        3       // point towards a particular angle (not pilot input accepted)
#define AUTO_YAW_LOOK_AHEAD             4       // point in the direction the copter is moving
#define AUTO_YAW_RESETTOARMEDYAW        5       // point towards heading at time motors were armed

// Ch6, Ch7 and Ch8 aux switch control
#define AUX_SWITCH_PWM_TRIGGER_HIGH 1800   // pwm value above which the ch7 or ch8 option will be invoked
#define AUX_SWITCH_PWM_TRIGGER_LOW  1200   // pwm value below which the ch7 or ch8 option will be disabled
#define CH6_PWM_TRIGGER_HIGH    1800
#define CH6_PWM_TRIGGER_LOW     1200

// values used by the ap.ch7_opt and ap.ch8_opt flags
#define AUX_SWITCH_LOW              0       // indicates auxiliar switch is in the low position (pwm <1200)
#define AUX_SWITCH_MIDDLE           1       // indicates auxiliar switch is in the middle position (pwm >1200, <1800)
#define AUX_SWITCH_HIGH             2       // indicates auxiliar switch is in the high position (pwm >1800)

// Frame types
#define UNDEFINED_FRAME 0
#define QUAD_FRAME 1
#define TRI_FRAME 2
#define HEXA_FRAME 3
#define Y6_FRAME 4
#define OCTA_FRAME 5
#define HELI_FRAME 6
#define OCTA_QUAD_FRAME 7
#define SINGLE_FRAME 8
#define COAX_FRAME 9

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define ToRad(x) radians(x) // *pi/180
#define ToDeg(x) degrees(x) // *180/pi

// HIL enumerations
#define HIL_MODE_DISABLED               0
#define HIL_MODE_SENSORS                1

// Auto Pilot modes
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2         //Remove             // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10      //Remove              // Hold a single location using optical flow sensor
#define DRIFT 11          //Remove              // DRIFT mode (Note: 12 is no longer used)
#define SPORT 13          //REmove              // earth frame rate control
#define FLIP        14    //REmove              // flip the vehicle on the roll axis
#define AUTOTUNE    15    //REmove              // autotune the vehicle's roll and pitch gains
#define POSHOLD     16    //Remove              // position hold with manual override
#define NUM_MODES   17

// Acro Trainer types
#define ACRO_TRAINER_DISABLED   0
#define ACRO_TRAINER_LEVELING   1
#define ACRO_TRAINER_LIMITED    2

// RC Feel roll/pitch definitions
#define RC_FEEL_RP_VERY_SOFT        0
#define RC_FEEL_RP_SOFT             25
#define RC_FEEL_RP_MEDIUM           50
#define RC_FEEL_RP_CRISP            75
#define RC_FEEL_RP_VERY_CRISP       100

// Yaw behaviours during missions - possible values for WP_YAW_BEHAVIOR parameter
#define WP_YAW_BEHAVIOR_NONE                          0   // auto pilot will never control yaw during missions or rtl (except for DO_CONDITIONAL_YAW command received)
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1   // auto pilot will face next waypoint or home during rtl
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2   // auto pilot will face next waypoint except when doing RTL at which time it will stay in it's last
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3   // auto pilot will look ahead during missions and rtl (primarily meant for traditional helicotpers)


// Waypoint options
#define WP_OPTION_ALT_CHANGE                    2
#define WP_OPTION_YAW                           4
#define WP_OPTION_ALT_REQUIRED                  8
#define WP_OPTION_RELATIVE                      16
//#define WP_OPTION_                    32
//#define WP_OPTION_                    64
#define WP_OPTION_NEXT_CMD                      128

// Auto modes
enum AutoMode {
    Auto_TakeOff,
    Auto_WP,
    Auto_Land,
    Auto_RTL,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline,
    Auto_NavGuided
};

// RTL states
enum RTLState {
    InitialClimb,
    ReturnHome,
    Land
};

// Flip states
enum FlipState {
    Flip_Start,
    Flip_Roll,
    Flip_Recover,
    Flip_Abandon
};

// LAND state
#define LAND_STATE_FLY_TO_LOCATION  0
#define LAND_STATE_DESCENDING       1

//  Logging parameters
#define TYPE_AIRSTART_MSG               0x00
#define TYPE_GROUNDSTART_MSG            0x01
#define LOG_ATTITUDE_MSG                0x01
#define LOG_MODE_MSG                    0x03
#define LOG_CONTROL_TUNING_MSG          0x04
#define LOG_NAV_TUNING_MSG              0x05
#define LOG_PERFORMANCE_MSG             0x06
#define LOG_CURRENT_MSG                 0x09
#define LOG_STARTUP_MSG                 0x0A
#define LOG_OPTFLOW_MSG                 0x0C
#define LOG_EVENT_MSG                   0x0D
#define LOG_PID_MSG                     0x0E    // deprecated
#define LOG_COMPASS_MSG                 0x0F
#define LOG_INAV_MSG                    0x11    // deprecated
#define LOG_CAMERA_MSG_DEPRECATED       0x12    // deprecated
#define LOG_ERROR_MSG                   0x13
#define LOG_DATA_INT16_MSG              0x14
#define LOG_DATA_UINT16_MSG             0x15
#define LOG_DATA_INT32_MSG              0x16
#define LOG_DATA_UINT32_MSG             0x17
#define LOG_DATA_FLOAT_MSG              0x18
#define LOG_AUTOTUNE_MSG                0x19
#define LOG_AUTOTUNEDETAILS_MSG         0x1A
#define LOG_COMPASS2_MSG                0x1B
#define LOG_COMPASS3_MSG                0x1C
//BEV begin custom logging
#define LOG_TRANSITION_MSG              0x1D
#define LOG_LANDING_MSG                 0x1E
//BEV end custom logging

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_RCIN                   (1<<6)
#define MASK_LOG_IMU                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CURRENT                (1<<9)
#define MASK_LOG_RCOUT                  (1<<10)
#define MASK_LOG_OPTFLOW                (1<<11)
#define MASK_LOG_PID                    (1<<12) // deprecated
#define MASK_LOG_COMPASS                (1<<13)
#define MASK_LOG_INAV                   (1<<14) // deprecated
#define MASK_LOG_CAMERA                 (1<<15)
#define MASK_LOG_WHEN_DISARMED          (1UL<<16)
#define MASK_LOG_ANY                    0xFFFF

// DATA - event logging
#define DATA_MAVLINK_FLOAT              1
#define DATA_MAVLINK_INT32              2
#define DATA_MAVLINK_INT16              3
#define DATA_MAVLINK_INT8               4
#define DATA_AP_STATE                   7
#define DATA_INIT_SIMPLE_BEARING        9
#define DATA_ARMED                      10
#define DATA_DISARMED                   11
#define DATA_AUTO_ARMED                 15
#define DATA_TAKEOFF                    16
#define DATA_LAND_COMPLETE_MAYBE        17
#define DATA_LAND_COMPLETE              18
#define DATA_NOT_LANDED                 28
#define DATA_LOST_GPS                   19
#define DATA_FLIP_START                 21
#define DATA_FLIP_END                   22
#define DATA_SET_HOME                   25
#define DATA_SET_SIMPLE_ON              26
#define DATA_SET_SIMPLE_OFF             27
#define DATA_SET_SUPERSIMPLE_ON         29
#define DATA_AUTOTUNE_INITIALISED       30
#define DATA_AUTOTUNE_OFF               31
#define DATA_AUTOTUNE_RESTART           32
#define DATA_AUTOTUNE_SUCCESS           33
#define DATA_AUTOTUNE_FAILED            34
#define DATA_AUTOTUNE_REACHED_LIMIT     35
#define DATA_AUTOTUNE_PILOT_TESTING     36
#define DATA_AUTOTUNE_SAVEDGAINS        37
#define DATA_SAVE_TRIM                  38
#define DATA_SAVEWP_ADD_WP              39
#define DATA_SAVEWP_CLEAR_MISSION_RTL   40
#define DATA_FENCE_ENABLE               41
#define DATA_FENCE_DISABLE              42
#define DATA_ACRO_TRAINER_DISABLED      43
#define DATA_ACRO_TRAINER_LEVELING      44
#define DATA_ACRO_TRAINER_LIMITED       45
#define DATA_EPM_ON                     46
#define DATA_EPM_OFF                    47
#define DATA_EPM_NEUTRAL                48
#define DATA_PARACHUTE_DISABLED         49
#define DATA_PARACHUTE_ENABLED          50
#define DATA_PARACHUTE_RELEASED         51

// Centi-degrees to radians
#define DEGX100 5729.57795f

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// Error message sub systems and error codes
#define ERROR_SUBSYSTEM_MAIN                1
#define ERROR_SUBSYSTEM_RADIO               2
#define ERROR_SUBSYSTEM_COMPASS             3
#define ERROR_SUBSYSTEM_OPTFLOW             4
#define ERROR_SUBSYSTEM_FAILSAFE_RADIO      5
#define ERROR_SUBSYSTEM_FAILSAFE_BATT       6
#define ERROR_SUBSYSTEM_FAILSAFE_GPS        7
#define ERROR_SUBSYSTEM_FAILSAFE_GCS        8
#define ERROR_SUBSYSTEM_FAILSAFE_FENCE      9
#define ERROR_SUBSYSTEM_FLIGHT_MODE         10
#define ERROR_SUBSYSTEM_GPS                 11
#define ERROR_SUBSYSTEM_CRASH_CHECK         12
#define ERROR_SUBSYSTEM_FLIP                13
#define ERROR_SUBSYSTEM_AUTOTUNE            14
#define ERROR_SUBSYSTEM_PARACHUTE           15
#define ERROR_SUBSYSTEM_EKFINAV_CHECK       16
#define ERROR_SUBSYSTEM_FAILSAFE_EKFINAV    17
#define ERROR_SUBSYSTEM_BARO                18
//BEV added this one
#define ERROR_SUBSYSTEM_FAILSAFE_RC_OVERRIDE         19

// general error codes
#define ERROR_CODE_ERROR_RESOLVED           0
#define ERROR_CODE_FAILED_TO_INITIALISE     1
// subsystem specific error codes -- radio
#define ERROR_CODE_RADIO_LATE_FRAME         2
// subsystem specific error codes -- failsafe_thr, batt, gps
#define ERROR_CODE_FAILSAFE_RESOLVED        0
#define ERROR_CODE_FAILSAFE_OCCURRED        1
// subsystem specific error codes -- compass
#define ERROR_CODE_COMPASS_FAILED_TO_READ   2
// subsystem specific error codes -- gps
#define ERROR_CODE_GPS_GLITCH               2
// subsystem specific error codes -- main
#define ERROR_CODE_MAIN_INS_DELAY           1
// subsystem specific error codes -- crash checker
#define ERROR_CODE_CRASH_CHECK_CRASH        1
#define ERROR_CODE_CRASH_CHECK_LOSS_OF_CONTROL 2
// subsystem specific error codes -- flip
#define ERROR_CODE_FLIP_ABANDONED           2
// subsystem specific error codes -- autotune
#define ERROR_CODE_AUTOTUNE_BAD_GAINS       2
// parachute failed to deploy because of low altitude
#define ERROR_CODE_PARACHUTE_TOO_LOW        2
// EKF check definitions
#define ERROR_CODE_EKFINAV_CHECK_BAD_VARIANCE       2
#define ERROR_CODE_EKFINAV_CHECK_VARIANCE_CLEARED   0
// Baro specific error codes
#define ERROR_CODE_BARO_GLITCH              2

// Arming Check Enable/Disable bits
#define ARMING_CHECK_NONE                   0x00
#define ARMING_CHECK_ALL                    0x01
#define ARMING_CHECK_BARO                   0x02
#define ARMING_CHECK_COMPASS                0x04
#define ARMING_CHECK_GPS                    0x08
#define ARMING_CHECK_INS                    0x10
#define ARMING_CHECK_PARAMETERS             0x20
#define ARMING_CHECK_RC                     0x40
#define ARMING_CHECK_VOLTAGE                0x80

// Radio failsafe definitions (FS_THR parameter)
#define FS_THR_DISABLED                    0
#define FS_THR_ENABLED_ALWAYS_RTL          1
#define FS_THR_ENABLED_CONTINUE_MISSION    2
#define FS_THR_ENABLED_ALWAYS_LAND         3

// Battery failsafe definitions (FS_BATT_ENABLE parameter)
#define FS_BATT_DISABLED                    0       // battery failsafe disabled
#define FS_BATT_LAND                        1       // switch to LAND mode on battery failsafe
#define FS_BATT_RTL                         2       // switch to RTL mode on battery failsafe

// GPS Failsafe definitions (FS_GPS_ENABLE parameter)
#define FS_GPS_DISABLED                     0       // GPS failsafe disabled
#define FS_GPS_LAND                         1       // switch to LAND mode on GPS Failsafe
#define FS_GPS_ALTHOLD                      2       // switch to ALTHOLD mode on GPS failsafe
#define FS_GPS_LAND_EVEN_STABILIZE          3       // switch to LAND mode on GPS failsafe even if in a manual flight mode like Stabilize


enum Serial2Protocol {
    SERIAL2_MAVLINK     = 1,
    SERIAL2_FRSKY_DPORT = 2,
    SERIAL2_FRSKY_SPORT = 3 // not supported yet
};

//BEV begin plane defines
// type of stick mixing enabled
enum StickMixing {
    STICK_MIXING_DISABLED = 0,
    STICK_MIXING_FBW      = 1,
    STICK_MIXING_DIRECT   = 2
};

// altitude control algorithms
enum {
    ALT_CONTROL_DEFAULT      = 0,
    ALT_CONTROL_NON_AIRSPEED = 1,
    ALT_CONTROL_TECS         = 2,
    ALT_CONTROL_AIRSPEED     = 3
};

// attitude controller choice
enum {
    ATT_CONTROL_PID = 0,
    ATT_CONTROL_APMCONTROL = 1
};

//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
#ifndef PLANE_WP_RADIUS_DEFAULT
 # define PLANE_WP_RADIUS_DEFAULT              30
#endif

#endif // _DEFINES_H
