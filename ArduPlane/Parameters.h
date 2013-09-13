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
    static const uint16_t k_software_type = 0;          // 0 for APM trunk

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,
        k_param_num_resets,

        // Misc
        //
        k_param_auto_trim      = 10,
        k_param_log_bitmask,
        k_param_pitch_trim_cd,
        k_param_mix_mode,
        k_param_reverse_elevons,
        k_param_reverse_ch1_elevon,
        k_param_reverse_ch2_elevon,
        k_param_flap_1_percent,
        k_param_flap_1_speed,
        k_param_flap_2_percent,
        k_param_flap_2_speed,
        k_param_reset_switch_chan,
        k_param_manual_level, // unused
        k_param_land_pitch_cd,
        k_param_ins_old,            // *** Deprecated, remove with next eeprom number change
        k_param_stick_mixing,
        k_param_reset_mission_chan,
        k_param_land_flare_alt,
        k_param_land_flare_sec,
        k_param_crosstrack_min_distance, // unused
        k_param_rudder_steer, // unused
        k_param_throttle_nudge,
        k_param_alt_offset,
        k_param_ins,                // libraries/AP_InertialSensor variables
        k_param_takeoff_throttle_min_speed,
        k_param_takeoff_throttle_min_accel,
        k_param_takeoff_heading_hold, // unused
        k_param_level_roll_limit,
        k_param_hil_servos,
        k_param_vtail_output,
        k_param_nav_controller,
        k_param_elevon_output,
        k_param_att_controller,
        k_param_mixing_gain,
        k_param_scheduler,
        k_param_relay,
        k_param_takeoff_throttle_delay,
        k_param_skip_gyro_cal,
        k_param_auto_fbw_steer,
        k_param_waypoint_max_radius,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,         // stream rates for port0
        k_param_gcs3,               // stream rates for port3
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial3_baud,
        k_param_telem_delay,
        k_param_serial0_baud,

        // 120: Fly-by-wire control
        //
        k_param_airspeed_min = 120,
        k_param_airspeed_max,
        k_param_FBWB_min_altitude_cm,  // 0=disabled, minimum value for altitude in cm (for first time try 30 meters = 3000 cm)
        k_param_flybywire_elev_reverse,
        k_param_alt_control_algorithm,
        k_param_flybywire_climb_rate,
        k_param_acro_roll_rate,
        k_param_acro_pitch_rate,

        //
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
        // 150: Navigation parameters
        //
        k_param_crosstrack_gain = 150, // unused
        k_param_crosstrack_entry_angle, // unused
        k_param_roll_limit_cd,
        k_param_pitch_limit_max_cd,
        k_param_pitch_limit_min_cd,
        k_param_airspeed_cruise_cm,
        k_param_RTL_altitude_cm,
        k_param_inverted_flight_ch,
        k_param_min_gndspeed_cm,
        k_param_crosstrack_use_wind, // unused


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
        k_param_rc_1 = 170,
        k_param_rc_2,
        k_param_rc_3,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,
        k_param_rc_9,
        k_param_rc_10,
        k_param_rc_11,

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_short_fs_action,
        k_param_long_fs_action,
        k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,
        k_param_throttle_suppress_manual,
        k_param_throttle_passthru_stabilize,
        k_param_rc_12,
        k_param_fs_batt_voltage,
        k_param_fs_batt_mah,
        k_param_short_fs_timeout,
        k_param_long_fs_timeout,

        //
        // 200: Feed-forward gains
        //
        k_param_kff_pitch_compensation = 200, // unused
        k_param_kff_rudder_mix,
        k_param_kff_pitch_to_throttle, // unused
        k_param_kff_throttle_to_pitch,
        k_param_scaling_speed,

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
        k_param_command_total,
        k_param_command_index,
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
        k_param_L1_controller,
        k_param_rcmap,
        k_param_TECS_controller,

        //
        // 240: PID Controllers
        k_param_pidNavRoll = 240, // unused
        k_param_pidServoRoll, // unused
        k_param_pidServoPitch, // unused
        k_param_pidNavPitchAirspeed, // unused
        k_param_pidServoRudder, // unused
        k_param_pidTeThrottle, // unused
        k_param_pidNavPitchAltitude, // unused
        k_param_pidWheelSteer,

        // 254,255: reserved
    };

    AP_Int16 format_version;
    AP_Int8 software_type;

    // Telemetry control
    //
    AP_Int16 sysid_this_mav;
    AP_Int16 sysid_my_gcs;
    AP_Int8 serial0_baud;
    AP_Int8 serial3_baud;
    AP_Int8 telem_delay;

    // Feed-forward gains
    //
    AP_Float kff_rudder_mix;
    AP_Float kff_pitch_to_throttle;
    AP_Float kff_throttle_to_pitch;

    // speed used for speed scaling
    AP_Float scaling_speed;

    // navigation controller type. See AP_Navigation::ControllerType
    AP_Int8  nav_controller;

    // attitude controller type.
    AP_Int8  att_controller;

    // skip gyro calibration
    AP_Int8  skip_gyro_cal;
    AP_Int8  auto_fbw_steer;

    // Estimation
    //
    AP_Float altitude_mix;
    AP_Int8  alt_control_algorithm;

    // Waypoints
    //
    AP_Int8 waypoint_mode;
    AP_Int8 command_total;
    AP_Int8 command_index;
    AP_Int16 waypoint_radius;
    AP_Int16 waypoint_max_radius;
    AP_Int16 loiter_radius;

#if GEOFENCE_ENABLED == ENABLED
    AP_Int8 fence_action;
    AP_Int8 fence_total;
    AP_Int8 fence_channel;
    AP_Int16 fence_minalt;    // meters
    AP_Int16 fence_maxalt;    // meters
#endif

    // Fly-by-wire
    //
    AP_Int8 flybywire_elev_reverse;
    AP_Int8 flybywire_climb_rate;

    // Throttle
    //
    AP_Int8 throttle_suppress_manual;
    AP_Int8 throttle_passthru_stabilize;
    AP_Int8 throttle_fs_enabled;
    AP_Int16 throttle_fs_value;
    AP_Int8 throttle_nudge;

    // Failsafe
    AP_Int8 short_fs_action;
    AP_Int8 long_fs_action;
    AP_Float short_fs_timeout;
    AP_Float long_fs_timeout;
    AP_Int8 gcs_heartbeat_fs_enabled;
    AP_Float fs_batt_voltage;
    AP_Float fs_batt_mah;

    // Flight modes
    //
    AP_Int8 flight_mode_channel;
    AP_Int8 flight_mode1;
    AP_Int8 flight_mode2;
    AP_Int8 flight_mode3;
    AP_Int8 flight_mode4;
    AP_Int8 flight_mode5;
    AP_Int8 flight_mode6;

    // Navigational maneuvering limits
    //
    AP_Int16 roll_limit_cd;
    AP_Int16 alt_offset;
    AP_Int16 acro_roll_rate;
    AP_Int16 acro_pitch_rate;

    // Misc
    //
    AP_Int8 auto_trim;
    AP_Int8 mix_mode;
    AP_Int8 vtail_output;
    AP_Int8 elevon_output;
    AP_Float mixing_gain;
    AP_Int8 reverse_elevons;
    AP_Int8 reverse_ch1_elevon;
    AP_Int8 reverse_ch2_elevon;
    AP_Int16 num_resets;
    AP_Int16 log_bitmask;
    AP_Int8 reset_switch_chan;
    AP_Int8 reset_mission_chan;
    AP_Int32 airspeed_cruise_cm;
    AP_Int32 RTL_altitude_cm;
    AP_Int16 land_pitch_cd;
    AP_Float land_flare_alt;
    AP_Float land_flare_sec;
    AP_Int32 min_gndspeed_cm;
    AP_Int16 pitch_trim_cd;
    AP_Int16 FBWB_min_altitude_cm;
#if HIL_MODE != HIL_MODE_DISABLED
    AP_Int8  hil_servos;
#endif

    AP_Int8 compass_enabled;
    AP_Int8 battery_monitoring;                 // 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8 flap_1_percent;
    AP_Int8 flap_1_speed;
    AP_Int8 flap_2_percent;
    AP_Int8 flap_2_speed;
    AP_Float volt_div_ratio;
    AP_Float curr_amp_per_volt;
    AP_Float curr_amp_offset;
    AP_Int32 pack_capacity;                     // Battery pack capacity less reserve
    AP_Int8 rssi_pin;
    AP_Int8 battery_volt_pin;
    AP_Int8 battery_curr_pin;
    AP_Int8 inverted_flight_ch;             // 0=disabled, 1-8 is channel for inverted flight trigger
    AP_Int8 stick_mixing;
    AP_Float takeoff_throttle_min_speed;
    AP_Float takeoff_throttle_min_accel;
    AP_Int8 takeoff_throttle_delay;
    AP_Int8 level_roll_limit;

    // RC channels
    RC_Channel rc_1;
    RC_Channel rc_2;
    RC_Channel rc_3;
    RC_Channel rc_4;
    RC_Channel_aux rc_5;
    RC_Channel_aux rc_6;
    RC_Channel_aux rc_7;
    RC_Channel_aux rc_8;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux rc_9;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux rc_10;
    RC_Channel_aux rc_11;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux rc_12;
#endif

    // PID controllers
    PID         pidWheelSteer;

    Parameters() :
        // variable				default
        //----------------------------------------
        rc_1                                    (CH_1),
        rc_2                                    (CH_2),
        rc_3                                    (CH_3),
        rc_4                                    (CH_4),
        rc_5                                    (CH_5),
        rc_6                                    (CH_6),
        rc_7                                    (CH_7),
        rc_8                                    (CH_8),
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_9                                    (CH_9),
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_10                                   (CH_10),
        rc_11                                   (CH_11),
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_12                                   (CH_12),
#endif

        // PID controller    initial P        initial I        initial D        initial imax
        //-----------------------------------------------------------------------------------
        pidWheelSteer         (0, 0, 0, 0)

        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H
