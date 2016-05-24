// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>

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


    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t k_software_type = 4;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,

        k_param_gcs0 = 100,         // stream rates for uartA
        k_param_gcs1,               // stream rates for uartC
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial0_baud,       // deprecated
        k_param_serial1_baud,       // deprecated
        k_param_imu,
        k_param_compass_enabled,
        k_param_compass,
        k_param_ahrs,  // AHRS group
        k_param_barometer,
        k_param_scheduler,
        k_param_ins,
        k_param_sitl,
        k_param_pidPitch2Srv,
        k_param_pidYaw2Srv,
        k_param_gcs2,               // stream rates for uartD
        k_param_serial2_baud,       // deprecated

        k_param_yaw_slew_time,
        k_param_pitch_slew_time,
        k_param_min_reverse_time,

        k_param_start_latitude,
        k_param_start_longitude,
        k_param_startup_delay,
        k_param_BoardConfig,
        k_param_gps,
        k_param_scan_speed,
        k_param_proxy_mode_unused, // deprecated
        k_param_servo_pitch_type,
        k_param_onoff_yaw_rate,
        k_param_onoff_pitch_rate,
        k_param_onoff_yaw_mintime,
        k_param_onoff_pitch_mintime,
        k_param_yaw_trim,
        k_param_pitch_trim,
        k_param_yaw_range,
        k_param_pitch_range,
        k_param_distance_min,
        k_param_sysid_target,       // 138
        k_param_gcs3,               // stream rates for fourth MAVLink port
        k_param_log_bitmask,        // 140
        k_param_notify,

        //
        // 150: Telemetry control
        //
        k_param_serial_manager,     // serial manager library
        k_param_servo_yaw_type,
        k_param_alt_source,

        //
        // 200 : Radio settings
        //
        k_param_channel_yaw = 200,
        k_param_channel_pitch,

        //
        // 220: Waypoint data
        //
        k_param_command_total = 220,

        // 254,255: reserved
    };

    AP_Int16 format_version;
    AP_Int8 software_type;

    // Telemetry control
    //
    AP_Int16 sysid_this_mav;
    AP_Int16 sysid_my_gcs;
    AP_Int16 sysid_target;

    AP_Int8 compass_enabled;

    AP_Float yaw_slew_time;
    AP_Float pitch_slew_time;
    AP_Float min_reverse_time;
    AP_Float scan_speed;

    AP_Float start_latitude;
    AP_Float start_longitude;

    AP_Float startup_delay;
    AP_Int8  servo_pitch_type;
    AP_Int8  servo_yaw_type;
    AP_Int8  alt_source;
    AP_Float onoff_yaw_rate;
    AP_Float onoff_pitch_rate;
    AP_Float onoff_yaw_mintime;
    AP_Float onoff_pitch_mintime;
    AP_Float yaw_trim;
    AP_Float pitch_trim;
    AP_Int16 yaw_range;             // yaw axis total range of motion in degrees
    AP_Int16 pitch_range;           // pitch axis total range of motion in degrees
    AP_Int16 distance_min;          // target's must be at least this distance from tracker to be tracked

    // Waypoints
    //
    AP_Int8 command_total; // 1 if HOME is set

    AP_Int32 log_bitmask;

    // PID controllers
    PID         pidPitch2Srv;
    PID         pidYaw2Srv;

    Parameters() :
        pidPitch2Srv(0.2, 0, 0.05f, 4000.0f),
        pidYaw2Srv  (0.2, 0, 0.05f, 4000.0f)
        {}
};

extern const AP_Param::Info var_info[];
