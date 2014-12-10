// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

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
        k_param_serial0_baud,
        k_param_serial1_baud,
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
        k_param_serial2_baud,

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
        k_param_servo_type,
        k_param_onoff_yaw_rate,
        k_param_onoff_pitch_rate,
        k_param_onoff_yaw_mintime,
        k_param_onoff_pitch_mintime,
        k_param_yaw_trim,
        k_param_pitch_trim,
        k_param_yaw_range,
        k_param_pitch_range,            // 136

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
    AP_Int8 serial0_baud;
    AP_Int8 serial1_baud;
#if MAVLINK_COMM_NUM_BUFFERS > 2
    AP_Int8 serial2_baud;
#endif

    AP_Int8 compass_enabled;

    AP_Float yaw_slew_time;
    AP_Float pitch_slew_time;
    AP_Float min_reverse_time;
    AP_Float scan_speed;

    AP_Float start_latitude;
    AP_Float start_longitude;

    AP_Float startup_delay;
    AP_Int8  servo_type;
    AP_Float onoff_yaw_rate;
    AP_Float onoff_pitch_rate;
    AP_Float onoff_yaw_mintime;
    AP_Float onoff_pitch_mintime;
    AP_Float yaw_trim;
    AP_Float pitch_trim;
    AP_Int16 yaw_range;             // yaw axis total range of motion in degrees
    AP_Int16 pitch_range;           // pitch axis total range of motion in degrees

    // Waypoints
    //
    AP_Int8 command_total; // 1 if HOME is set

    // PID controllers
    PID         pidPitch2Srv;
    PID         pidYaw2Srv;

    Parameters() :
        pidPitch2Srv(0.2, 0, 0.05f, 4000.0f),
        pidYaw2Srv  (0.2, 0, 0.05f, 4000.0f)
        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H
