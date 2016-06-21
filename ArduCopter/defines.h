// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// Autopilot Yaw Mode enumeration
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =             0,  // pilot controls the heading
    AUTO_YAW_LOOK_AT_NEXT_WP =  1,  // point towards next waypoint (no pilot input accepted)
    AUTO_YAW_ROI =              2,  // point towards a location held in roi_WP (no pilot input accepted)
    AUTO_YAW_LOOK_AT_HEADING =  3,  // point towards a particular angle (not pilot input accepted)
    AUTO_YAW_LOOK_AHEAD =       4,  // point in the direction the copter is moving
    AUTO_YAW_RESETTOARMEDYAW =  5,  // point towards heading at time motors were armed
};

// Ch6... Ch12 aux switch control
#define AUX_SWITCH_PWM_TRIGGER_HIGH 1800   // pwm value above which the ch7 or ch8 option will be invoked
#define AUX_SWITCH_PWM_TRIGGER_LOW  1200   // pwm value below which the ch7 or ch8 option will be disabled
#define CH6_PWM_TRIGGER_HIGH    1800
#define CH6_PWM_TRIGGER_LOW     1200

// values used by the ap.ch7_opt and ap.ch8_opt flags
#define AUX_SWITCH_LOW              0       // indicates auxiliary switch is in the low position (pwm <1200)
#define AUX_SWITCH_MIDDLE           1       // indicates auxiliary switch is in the middle position (pwm >1200, <1800)
#define AUX_SWITCH_HIGH             2       // indicates auxiliary switch is in the high position (pwm >1800)

// Aux Switch enumeration
enum aux_sw_func {
    AUXSW_DO_NOTHING =           0, // aux switch disabled
    AUXSW_FLIP =                 2, // flip
    AUXSW_SIMPLE_MODE =          3, // change to simple mode
    AUXSW_RTL =                  4, // change to RTL flight mode
    AUXSW_SAVE_TRIM =            5, // save current position as level
    AUXSW_SAVE_WP =              7, // save mission waypoint or RTL if in auto mode
    AUXSW_CAMERA_TRIGGER =       9, // trigger camera servo or relay
    AUXSW_RANGEFINDER =         10, // allow enabling or disabling rangefinder in flight which helps avoid surface tracking when you are far above the ground
    AUXSW_FENCE =               11, // allow enabling or disabling fence in flight
    AUXSW_RESETTOARMEDYAW =     12, // changes yaw to be same as when quad was armed
    AUXSW_SUPERSIMPLE_MODE =    13, // change to simple mode in middle, super simple at top
    AUXSW_ACRO_TRAINER =        14, // low = disabled, middle = leveled, high = leveled and limited
    AUXSW_SPRAYER =             15, // enable/disable the crop sprayer
    AUXSW_AUTO =                16, // change to auto flight mode
    AUXSW_AUTOTUNE =            17, // auto tune
    AUXSW_LAND =                18, // change to LAND flight mode
    AUXSW_EPM =                 19, // Operate the EPM cargo gripper low=off, middle=neutral, high=on
    AUXSW_PARACHUTE_ENABLE  =   21, // Parachute enable/disable
    AUXSW_PARACHUTE_RELEASE =   22, // Parachute release
    AUXSW_PARACHUTE_3POS =      23, // Parachute disable, enable, release with 3 position switch
    AUXSW_MISSION_RESET =       24, // Reset auto mission to start from first command
    AUXSW_ATTCON_FEEDFWD =      25, // enable/disable the roll and pitch rate feed forward
    AUXSW_ATTCON_ACCEL_LIM =    26, // enable/disable the roll, pitch and yaw accel limiting
    AUXSW_RETRACT_MOUNT =       27, // Retract Mount
    AUXSW_RELAY =               28, // Relay pin on/off (only supports first relay)
    AUXSW_LANDING_GEAR =        29, // Landing gear controller
    AUXSW_LOST_COPTER_SOUND =   30, // Play lost copter sound
    AUXSW_MOTOR_ESTOP =         31, // Emergency Stop Switch
    AUXSW_MOTOR_INTERLOCK =     32, // Motor On/Off switch
    AUXSW_BRAKE =               33, // Brake flight mode
	AUXSW_RELAY2 =              34, // Relay2 pin on/off (in Mission planner set CH8_OPT  = 34)
    AUXSW_RELAY3 =              35, // Relay3 pin on/off (in Mission planner set CH9_OPT  = 35)
    AUXSW_RELAY4 =              36, // Relay4 pin on/off (in Mission planner set CH10_OPT = 36)
    AUXSW_THROW =               37  // change to THROW flight mode
};

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

// HIL enumerations
#define HIL_MODE_DISABLED               0
#define HIL_MODE_SENSORS                1

// Auto Pilot Modes enumeration
enum control_mode_t {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18   // throw to launch mode using inertial/GPS system, no pilot input
};

enum mode_reason_t {
    MODE_REASON_UNKNOWN=0,
    MODE_REASON_TX_COMMAND,
    MODE_REASON_GCS_COMMAND,
    MODE_REASON_RADIO_FAILSAFE,
    MODE_REASON_BATTERY_FAILSAFE,
    MODE_REASON_GCS_FAILSAFE,
    MODE_REASON_EKF_FAILSAFE,
    MODE_REASON_GPS_GLITCH,
    MODE_REASON_MISSION_END,
    MODE_REASON_THROTTLE_LAND_ESCAPE,
    MODE_REASON_FENCE_BREACH,
    MODE_REASON_TERRAIN_FAILSAFE,
    MODE_REASON_BRAKE_TIMEOUT,
    MODE_REASON_FLIP_COMPLETE
};

// Tuning enumeration
enum tuning_func {
    TUNING_NONE =                        0, //
    TUNING_STABILIZE_ROLL_PITCH_KP =     1, // stabilize roll/pitch angle controller's P term
    TUNING_STABILIZE_YAW_KP =            3, // stabilize yaw heading controller's P term
    TUNING_RATE_ROLL_PITCH_KP =          4, // body frame roll/pitch rate controller's P term
    TUNING_RATE_ROLL_PITCH_KI =          5, // body frame roll/pitch rate controller's I term
    TUNING_YAW_RATE_KP =                 6, // body frame yaw rate controller's P term
    TUNING_THROTTLE_RATE_KP =            7, // throttle rate controller's P term (desired rate to acceleration or motor output)
    TUNING_WP_SPEED =                   10, // maximum speed to next way point (0 to 10m/s)
    TUNING_LOITER_POSITION_KP =         12, // loiter distance controller's P term (position error to speed)
    TUNING_HELI_EXTERNAL_GYRO =         13, // TradHeli specific external tail gyro gain
    TUNING_ALTITUDE_HOLD_KP =           14, // altitude hold controller's P term (alt error to desired rate)
    TUNING_RATE_ROLL_PITCH_KD =         21, // body frame roll/pitch rate controller's D term
    TUNING_VEL_XY_KP =                  22, // loiter rate controller's P term (speed error to tilt angle)
    TUNING_ACRO_RP_KP =                 25, // acro controller's P term.  converts pilot input to a desired roll, pitch or yaw rate
    TUNING_YAW_RATE_KD =                26, // body frame yaw rate controller's D term
    TUNING_VEL_XY_KI =                  28, // loiter rate controller's I term (speed error to tilt angle)
    TUNING_AHRS_YAW_KP =                30, // ahrs's compass effect on yaw angle (0 = very low, 1 = very high)
    TUNING_AHRS_KP =                    31, // accelerometer effect on roll/pitch angle (0=low)
    TUNING_ACCEL_Z_KP =                 34, // accel based throttle controller's P term
    TUNING_ACCEL_Z_KI =                 35, // accel based throttle controller's I term
    TUNING_ACCEL_Z_KD =                 36, // accel based throttle controller's D term
    TUNING_DECLINATION =                38, // compass declination in radians
    TUNING_CIRCLE_RATE =                39, // circle turn rate in degrees (hard coded to about 45 degrees in either direction)
    TUNING_ACRO_YAW_KP =                40, // acro controller's P term.  converts pilot input to a desired roll, pitch or yaw rate
    TUNING_RANGEFINDER_GAIN =           41, // rangefinder gain
    TUNING_EKF_VERTICAL_POS =           42, // EKF's baro vs accel (higher rely on accels more, baro impact is reduced).  Range should be 0.2 ~ 4.0?  2.0 is default
    TUNING_EKF_HORIZONTAL_POS =         43, // EKF's gps vs accel (higher rely on accels more, gps impact is reduced).  Range should be 1.0 ~ 3.0?  1.5 is default
    TUNING_EKF_ACCEL_NOISE =            44, // EKF's accel noise (lower means trust accels more, gps & baro less).  Range should be 0.02 ~ 0.5  0.5 is default (but very robust at that level)
    TUNING_RC_FEEL_RP =                 45, // roll-pitch input smoothing
    TUNING_RATE_PITCH_KP =              46, // body frame pitch rate controller's P term
    TUNING_RATE_PITCH_KI =              47, // body frame pitch rate controller's I term
    TUNING_RATE_PITCH_KD =              48, // body frame pitch rate controller's D term
    TUNING_RATE_ROLL_KP =               49, // body frame roll rate controller's P term
    TUNING_RATE_ROLL_KI =               50, // body frame roll rate controller's I term
    TUNING_RATE_ROLL_KD =               51, // body frame roll rate controller's D term
    TUNING_RATE_PITCH_FF =              52, // body frame pitch rate controller FF term
    TUNING_RATE_ROLL_FF =               53, // body frame roll rate controller FF term
    TUNING_RATE_YAW_FF =                54, // body frame yaw rate controller FF term
    TUNING_RATE_MOT_YAW_HEADROOM =      55, // motors yaw headroom minimum
    TUNING_RATE_YAW_FILT =              56  // yaw rate input filter
};

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

// Auto modes
enum AutoMode {
    Auto_TakeOff,
    Auto_WP,
    Auto_Land,
    Auto_RTL,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline,
    Auto_NavGuided,
    Auto_Loiter
};

// Guided modes
enum GuidedMode {
    Guided_TakeOff,
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel,
    Guided_Angle,
};

// RTL states
enum RTLState {
    RTL_InitialClimb,
    RTL_ReturnHome,
    RTL_LoiterAtHome,
    RTL_FinalDescent,
    RTL_Land
};

// Alt_Hold states
enum AltHoldModeState {
    AltHold_MotorStopped,
    AltHold_NotAutoArmed,
    AltHold_Takeoff,
    AltHold_Flying,
    AltHold_Landed
};

// Loiter states
enum LoiterModeState {
    Loiter_MotorStopped,
    Loiter_NotAutoArmed,
    Loiter_Takeoff,
    Loiter_Flying,
    Loiter_Landed
};

// Flip states
enum FlipState {
    Flip_Start,
    Flip_Roll,
    Flip_Pitch_A,
    Flip_Pitch_B,
    Flip_Recover,
    Flip_Abandon
};

// Throw states
enum ThrowModeState {
    Throw_Disarmed,
    Throw_Detecting,
    Throw_Uprighting,
    Throw_HgtStabilise,
    Throw_PosHold
};

// LAND state
#define LAND_STATE_FLY_TO_LOCATION  0
#define LAND_STATE_DESCENDING       1

//  Logging parameters
#define TYPE_AIRSTART_MSG               0x00
#define TYPE_GROUNDSTART_MSG            0x01
#define LOG_CONTROL_TUNING_MSG          0x04
#define LOG_NAV_TUNING_MSG              0x05
#define LOG_PERFORMANCE_MSG             0x06
#define LOG_OPTFLOW_MSG                 0x0C
#define LOG_EVENT_MSG                   0x0D
#define LOG_PID_MSG                     0x0E    // deprecated
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
#define LOG_MOTBATT_MSG                 0x1E
#define LOG_PARAMTUNE_MSG               0x1F
#define LOG_HELI_MSG                    0x20
#define LOG_PRECLAND_MSG                0x21
#define LOG_GUIDEDTARGET_MSG            0x22

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
#define MASK_LOG_PID                    (1<<12)
#define MASK_LOG_COMPASS                (1<<13)
#define MASK_LOG_INAV                   (1<<14) // deprecated
#define MASK_LOG_CAMERA                 (1<<15)
#define MASK_LOG_MOTBATT                (1UL<<17)
#define MASK_LOG_IMU_FAST               (1UL<<18)
#define MASK_LOG_IMU_RAW                (1UL<<19)
#define MASK_LOG_ANY                    0xFFFF

// DATA - event logging
#define DATA_AP_STATE                       7
#define DATA_SYSTEM_TIME_SET                8
#define DATA_INIT_SIMPLE_BEARING            9
#define DATA_ARMED                          10
#define DATA_DISARMED                       11
#define DATA_AUTO_ARMED                     15
#define DATA_LAND_COMPLETE_MAYBE            17
#define DATA_LAND_COMPLETE                  18
#define DATA_NOT_LANDED                     28
#define DATA_LOST_GPS                       19
#define DATA_FLIP_START                     21
#define DATA_FLIP_END                       22
#define DATA_SET_HOME                       25
#define DATA_SET_SIMPLE_ON                  26
#define DATA_SET_SIMPLE_OFF                 27
#define DATA_SET_SUPERSIMPLE_ON             29
#define DATA_AUTOTUNE_INITIALISED           30
#define DATA_AUTOTUNE_OFF                   31
#define DATA_AUTOTUNE_RESTART               32
#define DATA_AUTOTUNE_SUCCESS               33
#define DATA_AUTOTUNE_FAILED                34
#define DATA_AUTOTUNE_REACHED_LIMIT         35
#define DATA_AUTOTUNE_PILOT_TESTING         36
#define DATA_AUTOTUNE_SAVEDGAINS            37
#define DATA_SAVE_TRIM                      38
#define DATA_SAVEWP_ADD_WP                  39
#define DATA_FENCE_ENABLE                   41
#define DATA_FENCE_DISABLE                  42
#define DATA_ACRO_TRAINER_DISABLED          43
#define DATA_ACRO_TRAINER_LEVELING          44
#define DATA_ACRO_TRAINER_LIMITED           45
#define DATA_EPM_GRAB                       46
#define DATA_EPM_RELEASE                    47
#define DATA_PARACHUTE_DISABLED             49
#define DATA_PARACHUTE_ENABLED              50
#define DATA_PARACHUTE_RELEASED             51
#define DATA_LANDING_GEAR_DEPLOYED          52
#define DATA_LANDING_GEAR_RETRACTED         53
#define DATA_MOTORS_EMERGENCY_STOPPED       54
#define DATA_MOTORS_EMERGENCY_STOP_CLEARED  55
#define DATA_MOTORS_INTERLOCK_DISABLED      56
#define DATA_MOTORS_INTERLOCK_ENABLED       57
#define DATA_ROTOR_RUNUP_COMPLETE           58  // Heli only
#define DATA_ROTOR_SPEED_BELOW_CRITICAL     59  // Heli only
#define DATA_EKF_ALT_RESET                  60
#define DATA_LAND_CANCELLED_BY_PILOT        61

// Centi-degrees to radians
#define DEGX100 5729.57795f

// Error message sub systems and error codes
#define ERROR_SUBSYSTEM_MAIN                1
#define ERROR_SUBSYSTEM_RADIO               2
#define ERROR_SUBSYSTEM_COMPASS             3
#define ERROR_SUBSYSTEM_OPTFLOW             4
#define ERROR_SUBSYSTEM_FAILSAFE_RADIO      5
#define ERROR_SUBSYSTEM_FAILSAFE_BATT       6
#define ERROR_SUBSYSTEM_FAILSAFE_GPS        7   // not used
#define ERROR_SUBSYSTEM_FAILSAFE_GCS        8
#define ERROR_SUBSYSTEM_FAILSAFE_FENCE      9
#define ERROR_SUBSYSTEM_FLIGHT_MODE         10
#define ERROR_SUBSYSTEM_GPS                 11  // not used
#define ERROR_SUBSYSTEM_CRASH_CHECK         12
#define ERROR_SUBSYSTEM_FLIP                13
#define ERROR_SUBSYSTEM_AUTOTUNE            14
#define ERROR_SUBSYSTEM_PARACHUTE           15
#define ERROR_SUBSYSTEM_EKFCHECK            16
#define ERROR_SUBSYSTEM_FAILSAFE_EKFINAV    17
#define ERROR_SUBSYSTEM_BARO                18
#define ERROR_SUBSYSTEM_CPU                 19
#define ERROR_SUBSYSTEM_FAILSAFE_ADSB       20
#define ERROR_SUBSYSTEM_TERRAIN             21
#define ERROR_SUBSYSTEM_NAVIGATION          22
#define ERROR_SUBSYSTEM_FAILSAFE_TERRAIN    23
// general error codes
#define ERROR_CODE_ERROR_RESOLVED           0
#define ERROR_CODE_FAILED_TO_INITIALISE     1
#define ERROR_CODE_UNHEALTHY                4
// subsystem specific error codes -- radio
#define ERROR_CODE_RADIO_LATE_FRAME         2
// subsystem specific error codes -- failsafe_thr, batt, gps
#define ERROR_CODE_FAILSAFE_RESOLVED        0
#define ERROR_CODE_FAILSAFE_OCCURRED        1
// subsystem specific error codes -- compass
#define ERROR_CODE_COMPASS_FAILED_TO_READ   2
// subsystem specific error codes -- main
#define ERROR_CODE_MAIN_INS_DELAY           1
// subsystem specific error codes -- crash checker
#define ERROR_CODE_CRASH_CHECK_CRASH        1
#define ERROR_CODE_CRASH_CHECK_LOSS_OF_CONTROL 2
// subsystem specific error codes -- flip
#define ERROR_CODE_FLIP_ABANDONED           2
// subsystem specific error codes -- terrain
#define ERROR_CODE_MISSING_TERRAIN_DATA     2
// subsystem specific error codes -- navigation
#define ERROR_CODE_FAILED_TO_SET_DESTINATION    2
#define ERROR_CODE_RESTARTED_RTL            3
#define ERROR_CODE_FAILED_CIRCLE_INIT       4
#define ERROR_CODE_DEST_OUTSIDE_FENCE       5

// parachute failed to deploy because of low altitude or landed
#define ERROR_CODE_PARACHUTE_TOO_LOW        2
#define ERROR_CODE_PARACHUTE_LANDED         3
// EKF check definitions
#define ERROR_CODE_EKFCHECK_BAD_VARIANCE       2
#define ERROR_CODE_EKFCHECK_VARIANCE_CLEARED   0
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

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED                     0
#define FS_GCS_ENABLED_ALWAYS_RTL           1
#define FS_GCS_ENABLED_CONTINUE_MISSION     2

// EKF failsafe definitions (FS_EKF_ACTION parameter)
#define FS_EKF_ACTION_LAND                  1       // switch to LAND mode on EKF failsafe
#define FS_EKF_ACTION_ALTHOLD               2       // switch to ALTHOLD mode on EKF failsafe
#define FS_EKF_ACTION_LAND_EVEN_STABILIZE   3       // switch to Land mode on EKF failsafe even if in a manual flight mode like stabilize

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

// for PILOT_THR_BHV parameter
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
