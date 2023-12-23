#pragma once

#define AP_PARAM_VEHICLE_NAME tracker

#include <AC_PID/AC_PID.h>
#include <AP_Param/AP_Param.h>

// Global parameter class.
//
class Parameters {
public:

    /*
     *  The value of k_format_version determines whether the existing
     *  eeprom data is considered valid. You should only change this
     *  value under the following circumstances:
     *
     *  1) the meaning of an existing eeprom parameter changes
     *
     *  2) the value of an existing k_param_* enum value changes
     *
     *  Adding a new parameter should _not_ require a change to
     *  k_format_version except under special circumstances. If you
     *  change it anyway then all ArduPlane users will need to reload all
     *  their parameters. We want that to be an extremely rare
     *  thing. Please do not just change it "just in case".
     *
     *  To determine if a k_param_* value has changed, use the rules of
     *  C++ enums to work out the value of the neighboring enum
     *  values. If you don't know the C++ enum rules then please ask for
     *  help.
     */

    //////////////////////////////////////////////////////////////////
    // STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
    // COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
    static const uint16_t k_format_version = 1;
    //////////////////////////////////////////////////////////////////


    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,      // deprecated

        k_param_gcs0 = 100,         // stream rates for SERIAL0
        k_param_gcs1,               // stream rates for SERIAL1
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial0_baud,       // deprecated
        k_param_serial1_baud,       // deprecated
        k_param_imu,
        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_ahrs,  // AHRS group
        k_param_barometer,
        k_param_scheduler,
        k_param_ins,
        k_param_sitl,
        k_param_pidPitch_old,   // deprecated
        k_param_pidYaw_old,     // deprecated
        k_param_gcs2,               // stream rates for SERIAL2
        k_param_serial2_baud,       // deprecated

        k_param_yaw_slew_time,
        k_param_pitch_slew_time,
        k_param_min_reverse_time,

        k_param_start_latitude,
        k_param_start_longitude,
        k_param_startup_delay,
        k_param_BoardConfig,
        k_param_gps,
        k_param_scan_speed_unused, // deprecated
        k_param_proxy_mode_unused, // deprecated
        k_param_servo_pitch_type,
        k_param_onoff_yaw_rate,
        k_param_onoff_pitch_rate,
        k_param_onoff_yaw_mintime,
        k_param_onoff_pitch_mintime,
        k_param_yaw_trim,
        k_param_pitch_trim,
        k_param_yaw_range,
        k_param_pitch_range,	//deprecated
        k_param_distance_min,
        k_param_sysid_target,       // 138
        k_param_gcs3,               // stream rates for fourth MAVLink port
        k_param_log_bitmask,        // 140
        k_param_notify,
        k_param_can_mgr,
        k_param_battery,

        //
        // 150: Telemetry control
        //
        k_param_serial_manager,     // serial manager library
        k_param_servo_yaw_type,
        k_param_alt_source,
        k_param_mavlink_update_rate,
        k_param_pitch_min,
        k_param_pitch_max,
        k_param_gcs4,
        k_param_gcs5,
        k_param_gcs6,

        //
        // 200 : Radio settings
        //
        k_param_channel_yaw_old = 200,
        k_param_channel_pitch_old,
        k_param_pidPitch2Srv,
        k_param_pidYaw2Srv,
        k_param_rc_channels,
        k_param_servo_channels,

        k_param_stats = 218,
        k_param_scripting = 219,

        //
        // 220: Waypoint data
        //
        k_param_command_total = 220,

        // 254,255: reserved
        k_param_gcs_pid_mask = 225,
        k_param_scan_speed_yaw,
        k_param_scan_speed_pitch,
        k_param_initial_mode,
        k_param_disarm_pwm,

        k_param_auto_opts,
        k_param_NavEKF2,
        k_param_NavEKF3,

        k_param_logger = 253, // 253 - Logging Group

        k_param_vehicle = 257, // vehicle common block of parameters
    };

    AP_Int16 format_version;

    // Telemetry control
    //
    AP_Int16 sysid_this_mav;
    AP_Int16 sysid_my_gcs;
    AP_Int16 sysid_target;

    AP_Float yaw_slew_time;
    AP_Float pitch_slew_time;
    AP_Float min_reverse_time;
    AP_Int16 scan_speed_yaw;
    AP_Int16 scan_speed_pitch;

    AP_Float start_latitude;
    AP_Float start_longitude;

    AP_Float startup_delay;
    AP_Int8  servo_pitch_type;
    AP_Int8  servo_yaw_type;
    AP_Int8  alt_source;
    AP_Int8  mavlink_update_rate;
    AP_Float onoff_yaw_rate;
    AP_Float onoff_pitch_rate;
    AP_Float onoff_yaw_mintime;
    AP_Float onoff_pitch_mintime;
    AP_Float yaw_trim;
    AP_Float pitch_trim;
    AP_Int16 yaw_range;             // yaw axis total range of motion in degrees
    AP_Int16 distance_min;          // target's must be at least this distance from tracker to be tracked
    AP_Int16 pitch_min;
    AP_Int16 pitch_max;
    AP_Int16 gcs_pid_mask;
    AP_Int8  initial_mode;
    AP_Int8 disarm_pwm;
    AP_Int8 auto_opts;

    // Waypoints
    //
    AP_Int8 command_total; // 1 if HOME is set

    AP_Int32 log_bitmask;

    // AC_PID controllers
    AC_PID         pidPitch2Srv;
    AC_PID         pidYaw2Srv;

    Parameters() :
        pidPitch2Srv(0.2, 0.0f, 0.05f, 0.02f, 4000.0f, 0.0f, 0.0f, 0.0f, 0.1f),
        pidYaw2Srv  (0.2, 0.0f, 0.05f, 0.02f, 4000.0f, 0.0f, 0.0f, 0.0f, 0.1f)
        {}
};

extern const AP_Param::Info var_info[];
