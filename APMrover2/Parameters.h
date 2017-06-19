#pragma once

#include <AP_Common/AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
    //
    static const uint16_t k_format_version = 16;
    static const uint16_t k_software_type = 20;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,

        // Misc
        //
        k_param_log_bitmask_old = 10,  // unused
        k_param_num_resets,
        k_param_reset_switch_chan,
        k_param_initial_mode,
        k_param_scheduler,
        k_param_relay,
        k_param_BoardConfig,
        k_param_pivot_turn_angle,
        k_param_rc_13_old,
        k_param_rc_14_old,

        // IO pins
        k_param_rssi_pin = 20,  // unused, replaced by rssi_ library parameters
        k_param_battery_volt_pin,
        k_param_battery_curr_pin,

        // braking
        k_param_braking_percent = 30,
        k_param_braking_speederr,

        // misc2
        k_param_log_bitmask = 40,
        k_param_gps,
        k_param_serial0_baud,   // deprecated, can be deleted
        k_param_serial1_baud,   // deprecated, can be deleted
        k_param_serial2_baud,   // deprecated, can be deleted

        // 97: RSSI
        k_param_rssi = 97,

        // 100: Arming parameters
        k_param_arming = 100,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,         // stream rates for uartA
        k_param_gcs1,               // stream rates for uartC
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial0_baud_old,
        k_param_serial1_baud_old,
        k_param_telem_delay,
        k_param_skip_gyro_cal,      // unused
        k_param_gcs2,               // stream rates for uartD
        k_param_serial2_baud_old,
        k_param_serial2_protocol,   // deprecated, can be deleted
        k_param_serial_manager,     // serial manager library
        k_param_cli_enabled,
        k_param_gcs3,
        k_param_gcs_pid_mask,

        //
        // 130: Sensor parameters
        //
        k_param_compass_enabled = 130,
        k_param_steering_learn,     // unused
        k_param_NavEKF,             // deprecated - remove
        k_param_mission,            // mission library
        k_param_NavEKF2_old,        // deprecated - remove
        k_param_NavEKF2,
        k_param_g2,                 // 2nd block of parameters
        k_param_NavEKF3,

        // 140: battery controls
        k_param_battery_monitoring = 140,   // deprecated, can be deleted
        k_param_volt_div_ratio,             // deprecated, can be deleted
        k_param_curr_amp_per_volt,          // deprecated, can be deleted
        k_param_input_voltage,              // deprecated, can be deleted
        k_param_pack_capacity,              // deprecated, can be deleted
        k_param_battery,

        //
        // 150: Navigation parameters
        //
        k_param_crosstrack_gain = 150,
        k_param_crosstrack_entry_angle,
        k_param_speed_cruise,
        k_param_speed_turn_gain,
        k_param_speed_turn_dist,
        k_param_ch7_option,
        k_param_auto_trigger_pin,
        k_param_auto_kickstart,
        k_param_turn_circle,  // unused
        k_param_turn_max_g,

        //
        // 160: Radio settings
        //
        k_param_rc_1_old = 160,
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,

        // throttle control
        k_param_throttle_min = 170,
        k_param_throttle_max,
        k_param_throttle_cruise,
        k_param_throttle_slewrate,
        k_param_throttle_reduction,
        k_param_skid_steer_in,
        k_param_skid_steer_out,

        // failsafe control
        k_param_fs_action = 180,
        k_param_fs_timeout,
        k_param_fs_throttle_enabled,
        k_param_fs_throttle_value,
        k_param_fs_gcs_enabled,
        k_param_fs_crash_check,

        // obstacle control
        k_param_sonar_enabled = 190,  // deprecated, can be removed
        k_param_sonar_old,            // unused
        k_param_sonar_trigger_cm,
        k_param_sonar_turn_angle,
        k_param_sonar_turn_time,
        k_param_sonar2_old,           // unused
        k_param_sonar_debounce,
        k_param_sonar,                // sonar object

        //
        // 210: driving modes
        //
        k_param_mode_channel = 210,
        k_param_mode1,
        k_param_mode2,
        k_param_mode3,
        k_param_mode4,
        k_param_mode5,
        k_param_mode6,
        k_param_learn_channel,

        //
        // 220: Waypoint data
        //
        k_param_command_total = 220,    // unused
        k_param_command_index,          // unused
        k_param_waypoint_radius,

        //
        // 230: camera control
        //
        k_param_camera,
        k_param_camera_mount,
        k_param_camera_mount2,          // unused

        //
        // 240: PID Controllers
        k_param_pidNavSteer = 230,
        k_param_pidServoSteer,  // unused
        k_param_pidSpeedThrottle,

        // high RC channels
        k_param_rc_9_old = 235,
        k_param_rc_10_old,
        k_param_rc_11_old,
        k_param_rc_12_old,

        // other objects
        k_param_sitl = 240,
        k_param_ahrs,
        k_param_ins,
        k_param_compass,
        k_param_rcmap,
        k_param_L1_controller,
        k_param_steerController,
        k_param_barometer,
        k_param_notify,
        k_param_button,

        k_param_DataFlash = 253,  // Logging Group

        // 254,255: reserved
        };

    AP_Int16    format_version;
    AP_Int8     software_type;

    // Misc
    //
    AP_Int32    log_bitmask;
    AP_Int16    num_resets;
    AP_Int8     reset_switch_chan;
    AP_Int8     initial_mode;

    // braking
    AP_Int8     braking_percent;
    AP_Float    braking_speederr;

    // Telemetry control
    //
    AP_Int16    sysid_this_mav;
    AP_Int16    sysid_my_gcs;
    AP_Int8     telem_delay;
#if CLI_ENABLED == ENABLED
    AP_Int8     cli_enabled;
#endif

    // sensor parameters
    AP_Int8     compass_enabled;

    // navigation parameters
    //
    AP_Float    speed_cruise;
    AP_Int8     speed_turn_gain;
    AP_Float    speed_turn_dist;
    AP_Int8     ch7_option;
    AP_Int8     auto_trigger_pin;
    AP_Float    auto_kickstart;
    AP_Float    turn_max_g;
    AP_Int16    pivot_turn_angle;
    AP_Int16    gcs_pid_mask;

    // Throttle
    //
    AP_Int8     throttle_min;
    AP_Int8     throttle_max;
    AP_Int8     throttle_cruise;
    AP_Int8     throttle_slewrate;
    AP_Int8     skid_steer_in;
    AP_Int8     skid_steer_out;

    // failsafe control
    AP_Int8     fs_action;
    AP_Float    fs_timeout;
    AP_Int8     fs_throttle_enabled;
    AP_Int16    fs_throttle_value;
    AP_Int8     fs_gcs_enabled;
    AP_Int8     fs_crash_check;

    // obstacle control
    AP_Int16    sonar_trigger_cm;
    AP_Float    sonar_turn_angle;
    AP_Float    sonar_turn_time;
    AP_Int8     sonar_debounce;


    // driving modes
    //
    AP_Int8     mode_channel;
    AP_Int8     mode1;
    AP_Int8     mode2;
    AP_Int8     mode3;
    AP_Int8     mode4;
    AP_Int8     mode5;
    AP_Int8     mode6;
    AP_Int8     learn_channel;

    // Waypoints
    //
    AP_Float    waypoint_radius;

    // PID controllers
    //
    PID         pidSpeedThrottle;

    Parameters() :
        // PID controller    initial P        initial I        initial D        initial imax
        //-----------------------------------------------------------------------------------
        pidSpeedThrottle    (0.7,             0.2,             0.2,             4000)
        {}
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
 */
class ParametersG2 {
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // vehicle statistics
    AP_Stats stats;

    // whether to enforce acceptance of packets only from sysid_my_gcs
    AP_Int8 sysid_enforce;

    // RC input channels
    RC_Channels rc_channels;
    
    // control over servo output ranges
    SRV_Channels servo_channels;

#if ADVANCED_FAILSAFE == ENABLED
    // advanced failsafe library
    AP_AdvancedFailsafe_Rover afs;
#endif

};

extern const AP_Param::Info var_info[];
