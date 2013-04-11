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
    static const uint16_t k_format_version = 13;
    //////////////////////////////////////////////////////////////////


    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t k_software_type = 1;          // 0 for APM trunk

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,
        k_param_num_resets,

        // Misc
        //
        k_param_auto_trim      = 10,
        k_param_manual_level,
        k_param_alt_offset,
        k_param_ins,                // libraries/AP_InertialSensor variables

        // 110: Telemetry control
        //
        k_param_mavlink0 = 110,         // stream rates for port0
        k_param_mavlink1,               // stream rates for port3
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_sysid_my_target,
        k_param_serial3_baud,
        k_param_telem_delay,
        k_param_serial0_baud,

        // 130: Sensor parameters
        //
        k_param_imu = 130,  // unused
        k_param_altitude_mix,

        k_param_compass_enabled,
        k_param_compass,
        k_param_battery_monitoring,
        k_param_volt_div_ratio,
        k_param_curr_amp_per_volt,
        k_param_input_voltage, // deprecated, can be deleted
        k_param_pack_capacity,
        k_param_sonar_enabled,
        k_param_ahrs,  // AHRS group
        k_param_barometer,   // barometer ground calibration
        k_param_airspeed,  // AP_Airspeed parameters
        k_param_curr_amp_offset,

        //
        // Camera and mount parameters
        //
        k_param_camera = 160,
        k_param_camera_mount,
        k_param_camera_mount2,

        //
        // Battery monitoring parameters
        //
        k_param_rssi_pin = 167,
        k_param_battery_volt_pin,
        k_param_battery_curr_pin,   // 169

        //
        // 170: Radio settings
        //
        k_param_channel_azimuth = 170,
        k_param_channel_elevation,
        k_param_rc_3,
        k_param_rc_4,

        //
        // 210: flight modes
        //
        k_param_flight_mode_channel = 210,
        k_param_flight_mode1,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,

        //
        // 220: Waypoint data
        //
        k_param_waypoint_mode = 220,
        k_param_waypoint_radius,
        k_param_loiter_radius,
        k_param_fence_action,
        k_param_fence_total,
        k_param_fence_channel,
        k_param_fence_minalt,
        k_param_fence_maxalt,

        // other objects
        k_param_sitl = 230,
        k_param_obc,
        k_param_rollController,
        k_param_pitchController,
        k_param_yawController,

        // 254,255: reserved
    };

    AP_Int16 format_version;
    AP_Int8 software_type;

    // Telemetry control
    //
    AP_Int16 sysid_this_mav;
    AP_Int16 sysid_my_gcs;
    AP_Int16 sysid_my_target;
    AP_Int8 serial0_baud;
    AP_Int8 serial3_baud;
    AP_Int8 telem_delay;

    // Estimation
    //
    AP_Float altitude_mix;

    // Flight modes
    //
    AP_Int8 flight_mode_channel;
    AP_Int8 flight_mode1;
    AP_Int8 flight_mode2;
    AP_Int8 flight_mode3;
    AP_Int8 flight_mode4;
    AP_Int8 flight_mode5;
    AP_Int8 flight_mode6;

    AP_Int16 alt_offset;

    // Misc
    //
    AP_Int16 num_resets;
    AP_Int8 manual_level;

    AP_Int8 compass_enabled;
    AP_Int8 battery_monitoring;                 // 0=disabled, 3=voltage only, 4=voltage and current
    AP_Float volt_div_ratio;
    AP_Float curr_amp_per_volt;
    AP_Float curr_amp_offset;
    AP_Int32 pack_capacity;                     // Battery pack capacity less reserve
    AP_Int8 battery_volt_pin;
    AP_Int8 battery_curr_pin;

    // RC channels
    RC_Channel channel_azimuth;
    RC_Channel channel_elevation;
    RC_Channel_aux rc_3;
    RC_Channel_aux rc_4;

    Parameters() :
        // variable				default
        //----------------------------------------
        channel_azimuth                 (CH_1),
        channel_elevation               (CH_2),
        rc_3                            (CH_3),
        rc_4                            (CH_4)
        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H
