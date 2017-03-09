#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

#define BOTTOM_DETECTOR_TRIGGER_SEC 1.0
#define SURFACE_DETECTOR_TRIGGER_SEC 1.0

enum AutoSurfaceState {
    AUTO_SURFACE_STATE_GO_TO_LOCATION,
    AUTO_SURFACE_STATE_ASCEND
};

// Autopilot Yaw Mode enumeration
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =              0,  // pilot controls the heading
    AUTO_YAW_LOOK_AT_NEXT_WP =   1,  // point towards next waypoint (no pilot input accepted)
    AUTO_YAW_ROI =               2,  // point towards a location held in roi_WP (no pilot input accepted)
    AUTO_YAW_LOOK_AT_HEADING =   3,  // point towards a particular angle (not pilot input accepted)
    AUTO_YAW_LOOK_AHEAD =        4,  // point in the direction the vehicle is moving
    AUTO_YAW_RESETTOARMEDYAW =   5,  // point towards heading at time motors were armed
    AUTO_YAW_CORRECT_XTRACK =    6   // steer the sub in order to correct for crosstrack error during line following
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
    //    AUXSW_FLIP =                 2, // flip
    AUXSW_SIMPLE_MODE =          3, // change to simple mode

    // No RTL mode for Sub
    //    AUXSW_RTL =                  4, // change to RTL flight mode

    AUXSW_SAVE_TRIM =            5, // save current position as level
    AUXSW_SAVE_WP =              7, // save mission waypoint or RTL if in auto mode
    AUXSW_CAMERA_TRIGGER =       9, // trigger camera servo or relay
    AUXSW_RANGEFINDER =         10, // allow enabling or disabling rangefinder in flight which helps avoid surface tracking when you are far above the ground
    AUXSW_FENCE =               11, // allow enabling or disabling fence in flight
    AUXSW_RESETTOARMEDYAW =     12, // changes yaw to be same as when quad was armed
    AUXSW_SUPERSIMPLE_MODE =    13, // change to simple mode in middle, super simple at top
    AUXSW_ACRO_TRAINER =        14, // low = disabled, middle = leveled, high = leveled and limited

    // No sprayer for Sub, remove
    //    AUXSW_SPRAYER =             15, // enable/disable the crop sprayer

    AUXSW_AUTO =                16, // change to auto flight mode
    AUXSW_LAND =                18, // change to LAND flight mode
    AUXSW_GRIPPER =             19, // Operate cargo grippers low=off, middle=neutral, high=on

    // No parachute for Sub, remove
    //    AUXSW_PARACHUTE_ENABLE  =   21, // Parachute enable/disable
    //    AUXSW_PARACHUTE_RELEASE =   22, // Parachute release
    //    AUXSW_PARACHUTE_3POS =      23, // Parachute disable, enable, release with 3 position switch

    AUXSW_MISSION_RESET =       24, // Reset auto mission to start from first command
    AUXSW_ATTCON_FEEDFWD =      25, // enable/disable the roll and pitch rate feed forward
    AUXSW_ATTCON_ACCEL_LIM =    26, // enable/disable the roll, pitch and yaw accel limiting
    AUXSW_RETRACT_MOUNT =       27, // Retract Mount
    AUXSW_RELAY =               28, // Relay pin on/off (only supports first relay)

    // No landing gear for sub, remove
    //    AUXSW_LANDING_GEAR =        29, // Landing gear controller

    AUXSW_LOST_VEHICLE_SOUND =   30, // Play lost vehicle sound
    AUXSW_MOTOR_ESTOP =         31, // Emergency Stop Switch
    AUXSW_MOTOR_INTERLOCK =     32, // Motor On/Off switch
    //    AUXSW_BRAKE =               33, // Brake flight mode
    AUXSW_RELAY2 =              34, // Relay2 pin on/off (in Mission planner set CH8_OPT  = 34)
    AUXSW_RELAY3 =              35, // Relay3 pin on/off (in Mission planner set CH9_OPT  = 35)
    AUXSW_RELAY4 =              36, // Relay4 pin on/off (in Mission planner set CH10_OPT = 36)
};

// HIL enumerations
#define HIL_MODE_DISABLED               0
#define HIL_MODE_SENSORS                1

// Auto Pilot Modes enumeration
enum control_mode_t {
    STABILIZE =     0,  // manual angle with manual depth/throttle
    ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
    ALT_HOLD =      2,  // manual angle with automatic depth/throttle
    AUTO =          3,  // not implemented in sub // fully automatic waypoint control using mission commands
    GUIDED =        4,  // not implemented in sub // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    VELHOLD =       5,  // automatic x/y velocity control and automatic depth/throttle
    CIRCLE =        7,  // not implemented in sub // automatic circular flight with automatic throttle
    SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    MANUAL =       19   // Pass-through input with no stabilization
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
    MODE_REASON_SURFACE_COMPLETE,
    MODE_REASON_LEAK_FAILSAFE
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
#define WP_YAW_BEHAVIOR_CORRECT_XTRACK                4   // point towards intermediate position target during line following

// Auto modes
enum AutoMode {
    Auto_WP,
    Auto_Land,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline,
    Auto_NavGuided,
    Auto_Loiter,
    Auto_TerrainRecover
};

// Guided modes
enum GuidedMode {
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

//  Logging parameters
#define TYPE_AIRSTART_MSG               0x00
#define TYPE_GROUNDSTART_MSG            0x01
#define LOG_CONTROL_TUNING_MSG          0x04
#define LOG_NAV_TUNING_MSG              0x05
#define LOG_PERFORMANCE_MSG             0x06
#define LOG_OPTFLOW_MSG                 0x0C
#define LOG_EVENT_MSG                   0x0D
#define LOG_ERROR_MSG                   0x13
#define LOG_DATA_INT16_MSG              0x14
#define LOG_DATA_UINT16_MSG             0x15
#define LOG_DATA_INT32_MSG              0x16
#define LOG_DATA_UINT32_MSG             0x17
#define LOG_DATA_FLOAT_MSG              0x18
#define LOG_MOTBATT_MSG                 0x1E
#define LOG_PARAMTUNE_MSG               0x1F
#define LOG_HELI_MSG                    0x20
#define LOG_GUIDEDTARGET_MSG            0x22
#define LOG_PROXIMITY_MSG               0x24

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
#define DATA_NOT_LANDED                     28
#define DATA_LOST_GPS                       19
#define DATA_SET_HOME                       25
#define DATA_SET_SIMPLE_ON                  26
#define DATA_SET_SIMPLE_OFF                 27
#define DATA_SET_SUPERSIMPLE_ON             29
#define DATA_SAVE_TRIM                      38
#define DATA_SAVEWP_ADD_WP                  39
#define DATA_FENCE_ENABLE                   41
#define DATA_FENCE_DISABLE                  42
#define DATA_ACRO_TRAINER_DISABLED          43
#define DATA_ACRO_TRAINER_LEVELING          44
#define DATA_ACRO_TRAINER_LIMITED           45
#define DATA_GRIPPER_GRAB                   46
#define DATA_GRIPPER_RELEASE                47
#define DATA_MOTORS_EMERGENCY_STOPPED       54
#define DATA_MOTORS_EMERGENCY_STOP_CLEARED  55
#define DATA_MOTORS_INTERLOCK_DISABLED      56
#define DATA_MOTORS_INTERLOCK_ENABLED       57
#define DATA_EKF_ALT_RESET                  60
#define DATA_LAND_CANCELLED_BY_PILOT        61
#define DATA_EKF_YAW_RESET                  62
#define DATA_SURFACED                       63
#define DATA_NOT_SURFACED                   64
#define DATA_BOTTOMED                       65
#define DATA_NOT_BOTTOMED                   66

// Centi-degrees to radians
#define DEGX100 5729.57795f

// Error message sub systems and error codes
#define ERROR_SUBSYSTEM_MAIN                1
#define ERROR_SUBSYSTEM_INPUT               2
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
#define ERROR_SUBSYSTEM_EKFCHECK            16
#define ERROR_SUBSYSTEM_FAILSAFE_EKFINAV    17
#define ERROR_SUBSYSTEM_BARO                18
#define ERROR_SUBSYSTEM_CPU                 19
#define ERROR_SUBSYSTEM_TERRAIN             21
#define ERROR_SUBSYSTEM_NAVIGATION          22
#define ERROR_SUBSYSTEM_FAILSAFE_TERRAIN    23
#define ERROR_SUBSYSTEM_FAILSAFE_LEAK       24
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
// subsystem specific error codes -- terrain
#define ERROR_CODE_MISSING_TERRAIN_DATA     2
// subsystem specific error codes -- navigation
#define ERROR_CODE_FAILED_TO_SET_DESTINATION    2
#define ERROR_CODE_RESTARTED_RTL            3
#define ERROR_CODE_FAILED_CIRCLE_INIT       4
#define ERROR_CODE_DEST_OUTSIDE_FENCE       5

// EKF check definitions
#define ERROR_CODE_EKFCHECK_BAD_VARIANCE       2
#define ERROR_CODE_EKFCHECK_VARIANCE_CLEARED   0
// Baro specific error codes
#define ERROR_CODE_BARO_GLITCH              2

// Battery failsafe definitions (FS_BATT_ENABLE parameter)
#define FS_BATT_DISABLED                    0       // battery failsafe disabled
#define FS_BATT_LAND                        1       // switch to LAND mode on battery failsafe
#define FS_BATT_RTL                         2       // switch to RTL mode on battery failsafe

// GCS failsafe definitions (FS_GCS_ENABLE parameter)
#define FS_GCS_DISABLED     0 // Disabled
#define FS_GCS_WARN_ONLY    1 // Only send warning to gcs (only useful with multiple gcs links)
#define FS_GCS_DISARM       2 // Disarm
#define FS_GCS_HOLD         3 // Switch depth hold mode or poshold mode if available
#define FS_GCS_SURFACE      4 // Switch to surface mode

// Leak failsafe definitions (FS_LEAK_ENABLE parameter)
#define FS_LEAK_DISABLED    0 // Disabled
#define FS_LEAK_WARN_ONLY   1 // Only send waring to gcs
#define FS_LEAK_SURFACE     2 // Switch to surface mode

// Internal pressure failsafe threshold (FS_PRESS_MAX parameter)
#define FS_PRESS_MAX_DEFAULT    105000 // Maximum internal pressure in pascal before failsafe is triggered
// Internal pressure failsafe definitions (FS_PRESS_ENABLE parameter)
#define FS_PRESS_DISABLED       0
#define FS_PRESS_WARN_ONLY      1

// Internal temperature failsafe threshold (FS_TEMP_MAX parameter)
#define FS_TEMP_MAX_DEFAULT     62  // Maximum internal pressure in degrees C before failsafe is triggered
// Internal temperature failsafe definitions (FS_TEMP_ENABLE parameter)
#define FS_TEMP_DISABLED        0
#define FS_TEMP_WARN_ONLY       1

// Terrain failsafe actions for AUTO mode
#define FS_TERRAIN_DISARM       0
#define FS_TERRAIN_HOLD         1
#define FS_TERRAIN_SURFACE      2

// Amount of time to attempt recovery of valid rangefinder data before
// initiating terrain failsafe action
#define FS_TERRAIN_RECOVER_TIMEOUT_MS 10000

// EKF failsafe definitions (FS_EKF_ENABLE parameter)
#define FS_EKF_ACTION_DISABLED                  1       // Disabled, not implemented yet in Sub

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

