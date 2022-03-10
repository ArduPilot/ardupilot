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
    static const uint16_t k_format_version = 13;
    //////////////////////////////////////////////////////////////////


    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type, // unused;
        k_param_num_resets,
        k_param_NavEKF2,
        k_param_g2,
        k_param_avoidance_adsb,
        k_param_landing,
        k_param_NavEKF3,
        k_param_can_mgr,
        k_param_osd,

        // Misc
        //
        k_param_auto_trim      = 10, // unused
        k_param_log_bitmask_old,  // unused
        k_param_pitch_trim_cd,
        k_param_mix_mode,
        k_param_reverse_elevons, // unused
        k_param_reverse_ch1_elevon, // unused
        k_param_reverse_ch2_elevon, // unused
        k_param_flap_1_percent,
        k_param_flap_1_speed,
        k_param_flap_2_percent,
        k_param_flap_2_speed,
        k_param_reset_switch_chan, // unused - moved to RC option
        k_param_manual_level, // unused
        k_param_land_pitch_cd,  // unused - moved to AP_Landing
        k_param_ins_old,            // *** Deprecated, remove with next eeprom number change
        k_param_stick_mixing,
        k_param_reset_mission_chan, // unused - moved to RC option
        k_param_land_flare_alt, // unused - moved to AP_Landing
        k_param_land_flare_sec, // unused - moved to AP_Landing
        k_param_crosstrack_min_distance, // unused
        k_param_rudder_steer, // unused
        k_param_throttle_nudge,
        k_param_alt_offset,
        k_param_ins,                // libraries/AP_InertialSensor variables
        k_param_takeoff_throttle_min_speed,
        k_param_takeoff_throttle_min_accel,
        k_param_takeoff_heading_hold, // unused
        k_param_level_roll_limit,
        k_param_hil_servos_unused,  // unused
        k_param_vtail_output, // unused
        k_param_nav_controller, // unused
        k_param_elevon_output, // unused
        k_param_att_controller, // unused
        k_param_mixing_gain,
        k_param_scheduler,
        k_param_relay,
        k_param_takeoff_throttle_delay,
        k_param_mode_takeoff, // was skip_gyro_cal
        k_param_auto_fbw_steer, // unused
        k_param_waypoint_max_radius,
        k_param_ground_steer_alt,        
        k_param_ground_steer_dps,
        k_param_rally_limit_km_old, //unused anymore -- just holding this index
        k_param_hil_err_limit_unused,  // unused
        k_param_sonar_old, // unused
        k_param_log_bitmask,
        k_param_BoardConfig,
        k_param_rssi_range,     // unused, replaced by rssi_ library parameters
        k_param_flapin_channel_old,  // unused, moved to RC_OPTION
        k_param_flaperon_output, // unused
        k_param_gps,
        k_param_autotune_level,
        k_param_rally,
        k_param_serial0_baud,           // deprecated
        k_param_serial1_baud,           // deprecated
        k_param_serial2_baud,           // deprecated
        k_param_takeoff_tdrag_elevator,
        k_param_takeoff_tdrag_speed1,
        k_param_takeoff_rotate_speed,
        k_param_takeoff_throttle_slewrate,
        k_param_takeoff_throttle_max,
        k_param_rangefinder,
        k_param_terrain,
        k_param_terrain_follow,
        k_param_stab_pitch_down_cd_old, // deprecated
        k_param_glide_slope_min,
        k_param_stab_pitch_down,
        k_param_terrain_lookahead,
        k_param_fbwa_tdrag_chan, // unused - moved to RC option
        k_param_rangefinder_landing,
        k_param_land_flap_percent,  // unused - moved to AP_Landing
        k_param_takeoff_flap_percent,
        k_param_flap_slewrate,
        k_param_rtl_autoland,
        k_param_override_channel,
        k_param_stall_prevention,
        k_param_optflow,
        k_param_cli_enabled_old, // unused - CLI removed
        k_param_trim_rc_at_start, // unused
        k_param_hil_mode_unused,  // unused
        k_param_land_disarm_delay,  // unused - moved to AP_Landing
        k_param_glide_slope_threshold,
        k_param_rudder_only,
        k_param_gcs3,            // 93
        k_param_gcs_pid_mask,
        k_param_crash_detection_enable,
        k_param_land_abort_throttle_enable, // unused - moved to AP_Landing
        k_param_rssi = 97,
        k_param_rpm_sensor,
        k_param_parachute,
        k_param_arming = 100,
        k_param_parachute_channel, // unused - moved to RC option
        k_param_crash_accel_threshold,
        k_param_override_safety, // unused
        k_param_land_throttle_slewrate, // 104 unused - moved to AP_Landing

        // 105: Extra parameters
        k_param_fence_retalt = 105,
        k_param_fence_autoenable,
        k_param_fence_ret_rally,
        k_param_q_attitude_control,
        k_param_takeoff_pitch_limit_reduction_sec,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,         // stream rates for uartA
        k_param_gcs1,               // stream rates for uartC
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial1_baud_old,   // deprecated
        k_param_telem_delay,
        k_param_serial0_baud_old,   // deprecated
        k_param_gcs2,               // stream rates for uartD
        k_param_serial2_baud_old,   // deprecated
        k_param_serial2_protocol,   // deprecated

        // 120: Fly-by-wire control
        //
        k_param_airspeed_min = 120,
        k_param_airspeed_max,
        k_param_FBWB_min_altitude_cm,  // 0=disabled, minimum value for altitude in cm (for first time try 30 meters = 3000 cm)
        k_param_flybywire_elev_reverse,
        k_param_alt_control_algorithm, // unused
        k_param_flybywire_climb_rate,
        k_param_acro_roll_rate,
        k_param_acro_pitch_rate,
        k_param_acro_locking,
        k_param_use_reverse_thrust = 129,

        //
        // 130: Sensor parameters
        //
        k_param_imu = 130,  // unused
        k_param_altitude_mix, // deprecated

        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_battery_monitoring, // unused
        k_param_volt_div_ratio,     // unused
        k_param_curr_amp_per_volt,  // unused
        k_param_input_voltage, // deprecated, can be deleted
        k_param_pack_capacity,      // unused
        k_param_sonar_enabled_old,  // unused
        k_param_ahrs,  // AHRS group
        k_param_barometer,   // barometer ground calibration
        k_param_airspeed,           // only used for parameter conversion; AP_Airspeed parameters moved to AP_Vehicle
        k_param_curr_amp_offset,
        k_param_NavEKF,  // deprecated - remove
        k_param_mission, // mission library
        k_param_serial_manager, // serial manager library
        k_param_NavEKF2_old,  // deprecated - remove
        k_param_land_pre_flare_alt, // unused - moved to AP_Landing
        k_param_land_pre_flare_airspeed = 149,  // unused - moved to AP_Landing

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
        k_param_inverted_flight_ch_unused, // unused
        k_param_min_gndspeed_cm,
        k_param_crosstrack_use_wind, // unused


        //
        // Camera and mount parameters
        //
        k_param_camera = 160,
        k_param_camera_mount,
        k_param_camera_mount2,      // unused
        k_param_adsb,
        k_param_notify,
        k_param_land_pre_flare_sec = 165,   // unused - moved to AP_Landing

        //
        // Battery monitoring parameters
        //
        k_param_battery = 166,
        k_param_rssi_pin,               // unused, replaced by rssi_ library parameters - 167
        k_param_battery_volt_pin,       // unused - 168
        k_param_battery_curr_pin,       // unused - 169

        //
        // 170: Radio settings - all unused now
        //
        k_param_rc_1_old = 170,
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_9_old,
        k_param_rc_10_old,
        k_param_rc_11_old,

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_fs_action_short,
        k_param_fs_action_long,
        k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,
        k_param_throttle_suppress_manual,
        k_param_throttle_passthru_stabilize,
        k_param_rc_12_old,
        k_param_fs_batt_voltage, // unused - moved to AP_BattMonitor
        k_param_fs_batt_mah,     // unused - moved to AP_BattMonitor
        k_param_fs_timeout_short,
        k_param_fs_timeout_long,
        k_param_rc_13_old,
        k_param_rc_14_old,
        k_param_tuning,

        //
        // 200: Feed-forward gains
        //
        k_param_kff_pitch_compensation = 200, // unused
        k_param_kff_rudder_mix,
        k_param_kff_pitch_to_throttle, // unused
        k_param_kff_throttle_to_pitch,
        k_param_scaling_speed,
        k_param_quadplane,
        k_param_rtl_radius,
        k_param_land_then_servos_neutral,   // unused - moved to AP_Landing
        k_param_rc_15_old,
        k_param_rc_16_old,

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
        k_param_initial_mode,
        k_param_land_slope_recalc_shallow_threshold,    // unused - moved to AP_Landing
        k_param_land_slope_recalc_steep_threshold_to_abort, // unused - moved to AP_Landing

        //
        // 220: Waypoint data
        //
        k_param_waypoint_mode = 220, // unused
        k_param_command_total,  // unused
        k_param_command_index,  // unused
        k_param_waypoint_radius,
        k_param_loiter_radius,
        k_param_fence_action,
        k_param_fence_total,
        k_param_fence_channel, // unused - moved to RC option
        k_param_fence_minalt,
        k_param_fence_maxalt,

        // other objects
        k_param_sitl = 230,
        k_param_afs,
        k_param_rollController,
        k_param_pitchController,
        k_param_yawController,
        k_param_L1_controller,
        k_param_rcmap,
        k_param_TECS_controller,
        k_param_rally_total_old,  //unused
        k_param_steerController,

        //
        // 240: PID Controllers
        k_param_pidNavRoll = 240, // unused
        k_param_pidServoRoll, // unused
        k_param_pidServoPitch, // unused
        k_param_pidNavPitchAirspeed, // unused
        k_param_pidServoRudder, // unused
        k_param_pidTeThrottle, // unused
        k_param_pidNavPitchAltitude, // unused
        k_param_pidWheelSteer, // unused

        k_param_mixing_offset,
        k_param_dspoiler_rud_rate,

        k_param_logger = 253, // Logging Group

        // 254,255: reserved

        k_param_vehicle = 257, // vehicle common block of parameters
        k_param_gcs4,          // stream rates
        k_param_gcs5,          // stream rates
        k_param_gcs6,          // stream rates
        k_param_fence,         // vehicle fence
        k_param_acro_yaw_rate,
    };

    AP_Int16 format_version;

    // Telemetry control
    //
    AP_Int16 sysid_this_mav;
    AP_Int16 sysid_my_gcs;
    AP_Int8 telem_delay;

    AP_Int8  rtl_autoland;

    AP_Int8  crash_accel_threshold;

    // Feed-forward gains
    //
    AP_Float kff_rudder_mix;
    AP_Float kff_pitch_to_throttle;
    AP_Float kff_throttle_to_pitch;
    AP_Float ground_steer_alt;
    AP_Int16 ground_steer_dps;
    AP_Float stab_pitch_down;

    // speed used for speed scaling
    AP_Float scaling_speed;

    // Waypoints
    //
    AP_Int16 waypoint_radius;
    AP_Int16 waypoint_max_radius;
    AP_Int16 rtl_radius;

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
    AP_Int32 use_reverse_thrust;

    // Failsafe
    AP_Int8 fs_action_short;
    AP_Int8 fs_action_long;
    AP_Float fs_timeout_short;
    AP_Float fs_timeout_long;
    AP_Int8 gcs_heartbeat_fs_enabled;

    // Flight modes
    //
    AP_Int8 flight_mode_channel;
    AP_Int8 flight_mode1;
    AP_Int8 flight_mode2;
    AP_Int8 flight_mode3;
    AP_Int8 flight_mode4;
    AP_Int8 flight_mode5;
    AP_Int8 flight_mode6;
    AP_Int8 initial_mode;

    // Navigational maneuvering limits
    //
    AP_Int16 alt_offset;
    AP_Int16 acro_roll_rate;
    AP_Int16 acro_pitch_rate;
    AP_Int16 acro_yaw_rate;
    AP_Int8  acro_locking;

    // Misc
    //
    AP_Int8 rudder_only;
    AP_Float mixing_gain;
    AP_Int16 mixing_offset;
    AP_Int16 dspoiler_rud_rate;
    AP_Int16 num_resets;
    AP_Int32 log_bitmask;
    AP_Int32 RTL_altitude_cm;
    AP_Int16 pitch_trim_cd;
    AP_Int16 FBWB_min_altitude_cm;

    AP_Int8 flap_1_percent;
    AP_Int8 flap_1_speed;
    AP_Int8 flap_2_percent;
    AP_Int8 flap_2_speed;
    AP_Int8 takeoff_flap_percent;  
    AP_Enum<StickMixing> stick_mixing;
    AP_Float takeoff_throttle_min_speed;
    AP_Float takeoff_throttle_min_accel;
    AP_Int8 takeoff_throttle_delay;
    AP_Int8 takeoff_tdrag_elevator;
    AP_Float takeoff_tdrag_speed1;
    AP_Float takeoff_rotate_speed;
    AP_Int8 takeoff_throttle_slewrate;
    AP_Float takeoff_pitch_limit_reduction_sec;
    AP_Int8 level_roll_limit;
#if AP_TERRAIN_AVAILABLE
    AP_Int32 terrain_follow;
    AP_Int16 terrain_lookahead;
#endif
    AP_Int16 glide_slope_min;
    AP_Float glide_slope_threshold;
    AP_Int8 rangefinder_landing;
    AP_Int8 flap_slewrate;
#if HAL_WITH_IO_MCU
    AP_Int8 override_channel;
#endif
    AP_Int16 gcs_pid_mask;
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
 */
class ParametersG2 {
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // button reporting library
#if HAL_BUTTON_ENABLED
    AP_Button *button_ptr;
#endif

#if STATS_ENABLED == ENABLED
    // vehicle statistics
    AP_Stats stats;
#endif

    // internal combustion engine control
    AP_ICEngine ice_control;

    // RC input channels
    RC_Channels_Plane rc_channels;
    
    // control over servo output ranges
    SRV_Channels servo_channels;

    // whether to enforce acceptance of packets only from sysid_my_gcs
    AP_Int8 sysid_enforce;

#if HAL_SOARING_ENABLED
    // ArduSoar parameters
    SoaringController soaring_controller;
#endif

    // dual motor tailsitter rudder to differential thrust scaling: 0-100%
    AP_Int8 rudd_dt_gain;

    // mask of channels to do manual pass-thru for
    AP_Int32 manual_rc_mask;

    // home reset altitude threshold
    AP_Int8 home_reset_threshold;

#if GRIPPER_ENABLED == ENABLED
    // Payload Gripper
    AP_Gripper gripper;
#endif

    AP_Int32 flight_options;

#if AP_SCRIPTING_ENABLED
    AP_Scripting scripting;
#endif // AP_SCRIPTING_ENABLED

    AP_Int8 takeoff_throttle_accel_count;
    AP_Int8 takeoff_timeout;

#if LANDING_GEAR_ENABLED == ENABLED
    AP_LandingGear landing_gear;
#endif

    // crow flaps weighting
    AP_Int8 crow_flap_weight_outer;
    AP_Int8 crow_flap_weight_inner;
    AP_Int8 crow_flap_options;
    AP_Int8 crow_flap_aileron_matching;

    // Forward throttle battery voltage compenstaion
    AP_Float fwd_thr_batt_voltage_max;
    AP_Float fwd_thr_batt_voltage_min;
    AP_Int8  fwd_thr_batt_idx;

#if OFFBOARD_GUIDED == ENABLED
    // guided yaw heading PID
    AC_PID guidedHeading{5000.0,  0.0,   0.0, 0 ,  10.0,   5.0,  5.0 ,  5.0  , 0.2};
#endif

#if AP_SCRIPTING_ENABLED
    AP_Follow follow;
#endif

    AP_Float        fs_ekf_thresh;

    // min initial climb in RTL
    AP_Int16        rtl_climb_min;

    AP_Int8         man_expo_roll;
    AP_Int8         man_expo_pitch;
    AP_Int8         man_expo_rudder;

    AP_Int32        oneshot_mask;
};

extern const AP_Param::Info var_info[];
