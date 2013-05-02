// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

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
        k_param_log_bitmask = 10,
        k_param_num_resets,
        k_param_reset_switch_chan,
        k_param_initial_mode,

        // IO pins
        k_param_rssi_pin = 20,
        k_param_battery_volt_pin,
        k_param_battery_curr_pin,


        // 110: Telemetry control
        //
        k_param_gcs0 = 110, // stream rates for port0
        k_param_gcs3,       // stream rates for port3
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial0_baud,
        k_param_serial3_baud,
        k_param_telem_delay,

        //
        // 130: Sensor parameters
        //
        k_param_compass_enabled = 130,

        // 140: battery controls
        k_param_battery_monitoring = 140,
        k_param_volt_div_ratio,
        k_param_curr_amp_per_volt,
        k_param_input_voltage, // deprecated, can be deleted
        k_param_pack_capacity,

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

        //
        // 160: Radio settings
        //
        k_param_channel_steer = 160,
        k_param_rc_2,
        k_param_channel_throttle,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,

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

        // obstacle control
        k_param_sonar_enabled = 190, // deprecated, can be removed
        k_param_sonar, // sonar object
        k_param_sonar_trigger_cm,
        k_param_sonar_turn_angle,
        k_param_sonar_turn_time,
        k_param_sonar2, // sonar2 object
        k_param_sonar_debounce,
        
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

        //
        // 220: Waypoint data
        //
        k_param_command_total = 220,
        k_param_command_index,
        k_param_waypoint_radius,

        //
        // 240: PID Controllers
        k_param_pidNavSteer = 230,
        k_param_pidServoSteer,
        k_param_pidSpeedThrottle,

        // other objects
        k_param_sitl = 240,
        k_param_ahrs,
        k_param_ins,
        k_param_compass,

        // 254,255: reserved
        };

    AP_Int16    format_version;
	AP_Int8	    software_type;

    // Misc
    //
    AP_Int16    log_bitmask;
    AP_Int16    num_resets;
    AP_Int8	    reset_switch_chan;
    AP_Int8     initial_mode;

    // IO pins
    AP_Int8     rssi_pin;
    AP_Int8     battery_volt_pin;
    AP_Int8     battery_curr_pin;

	// Telemetry control
	//
	AP_Int16    sysid_this_mav;
	AP_Int16    sysid_my_gcs;
    AP_Int8	    serial0_baud;
    AP_Int8	    serial3_baud;
    AP_Int8     telem_delay;

    // sensor parameters
    AP_Int8	    compass_enabled;

    // battery controls
    AP_Int8	    battery_monitoring;	// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Float    volt_div_ratio;
    AP_Float    curr_amp_per_volt;
    AP_Int16    pack_capacity;		// Battery pack capacity less reserve    

    // navigation parameters
    //
    AP_Float    crosstrack_gain;
    AP_Int16    crosstrack_entry_angle;
    AP_Float    speed_cruise;
    AP_Int8     speed_turn_gain;
    AP_Float    speed_turn_dist;    
    AP_Int8	    ch7_option;
    AP_Int8     auto_trigger_pin;
    AP_Float    auto_kickstart;

    // RC channels
    RC_Channel      channel_steer;
    RC_Channel_aux	rc_2;
    RC_Channel      channel_throttle;
    RC_Channel_aux  rc_4;
    RC_Channel_aux	rc_5;
    RC_Channel_aux	rc_6;
    RC_Channel_aux	rc_7;
    RC_Channel_aux	rc_8;

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
	AP_Int8	    fs_gcs_enabled;

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
    
    // Waypoints
    //
    AP_Int8     command_total;
    AP_Int8     command_index;
    AP_Float    waypoint_radius;

    // PID controllers
    //
    PID         pidNavSteer;
    PID         pidServoSteer;
    PID         pidSpeedThrottle;

    Parameters() :
        // RC channels
        channel_steer(CH_1),
        rc_2(CH_2),
        channel_throttle(CH_3),
        rc_4(CH_4),
        rc_5(CH_5),
        rc_6(CH_6),
        rc_7(CH_7),
        rc_8(CH_8),

        // PID controller    initial P        initial I        initial D        initial imax
        //-----------------------------------------------------------------------------------
        pidNavSteer         (0.7,             0.1,             0.2,             2000),
        pidServoSteer       (0.5,             0.1,             0.2,             2000),
        pidSpeedThrottle    (0.7,             0.2,             0.2,             4000)
        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H

