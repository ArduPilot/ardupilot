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
    static const uint16_t k_format_version = 14;

	// The parameter software_type is set up solely for ground station use
	// and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
	// GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
	// values within that range to identify different branches.
	//
    static const uint16_t k_software_type = 20;		// 0 for APM trunk

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
		k_param_software_type,

        // Misc
        //
        k_param_log_bitmask,
        k_param_num_resets,
        k_param_log_last_filenumber,		// *** Deprecated - remove with next eeprom number change
        k_param_reset_switch_chan,
        k_param_manual_level,
        k_param_ins,                        // libraries/AP_InertialSensor variables        
        k_param_rssi_pin,
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
        k_param_imu = 130,  
        k_param_compass_enabled,
        k_param_compass,
        k_param_battery_monitoring,
        k_param_volt_div_ratio,
        k_param_curr_amp_per_volt,
        k_param_input_voltage,
        k_param_pack_capacity,

#if CONFIG_SONAR == ENABLED       
        k_param_sonar_enabled,
        k_param_sonar_type,
#endif

        k_param_ahrs,  // AHRS group
        
        //
        // 150: Navigation parameters
        //
        k_param_crosstrack_gain = 150,
        k_param_crosstrack_entry_angle,
        k_param_speed_cruise,
        k_param_ch7_option,

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

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled, // 170
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_long_fs_action,
        k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,

        // 180: APMrover parameters - JLN update
        k_param_sonar_trigger,
        k_param_booster,        
        
        //
        // 210: flight modes
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
        k_param_waypoint_mode = 220,
        k_param_command_total,
        k_param_command_index,
        k_param_waypoint_radius,

        // other objects
        k_param_sitl = 230,

        //
        // 240: PID Controllers
        k_param_pidNavSteer = 240,

        // steering-to-servo PID:
        k_param_pidServoSteer,

        // steering-to-servo PID:
        k_param_pidSpeedThrottle,

        // 254,255: reserved
        };

    AP_Int16    format_version;
	AP_Int8	    software_type;

	// Telemetry control
	//
	AP_Int16    sysid_this_mav;
	AP_Int16    sysid_my_gcs;
    AP_Int8	    serial0_baud;
    AP_Int8	    serial3_baud;
    AP_Int8     telem_delay;

    // Crosstrack navigation
    //
    AP_Float    crosstrack_gain;
    AP_Int16    crosstrack_entry_angle;
    
    // Waypoints
    //
    AP_Int8     waypoint_mode;
    AP_Int8     command_total;
    AP_Int8     command_index;
    AP_Float    waypoint_radius;
    
    // Throttle
    //
    AP_Int8     throttle_min;
    AP_Int8     throttle_max;
    AP_Int8     throttle_slewrate;
    AP_Int8     throttle_fs_enabled;
    AP_Int16    throttle_fs_value;
    AP_Int8     throttle_cruise;
    
    // Failsafe
    AP_Int8     long_fs_action;
	AP_Int8	    gcs_heartbeat_fs_enabled;

    // Flight modes
    //
    AP_Int8     mode_channel;
    AP_Int8     mode1;
    AP_Int8     mode2;
    AP_Int8     mode3;
    AP_Int8     mode4;
    AP_Int8     mode5;
    AP_Int8     mode6;
    
    // Misc
    //
    AP_Int16    num_resets;
    AP_Int16    log_bitmask;
    AP_Int16    log_last_filenumber;		// *** Deprecated - remove with next eeprom number change
    AP_Int8	    reset_switch_chan;
    AP_Int8	    manual_level;	
    AP_Float    speed_cruise;
    AP_Int8	    ch7_option;
        
    AP_Int8	    compass_enabled;
    AP_Int8	    battery_monitoring;	// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Float    volt_div_ratio;
    AP_Float    curr_amp_per_volt;
    AP_Float    input_voltage;
    AP_Int16    pack_capacity;		// Battery pack capacity less reserve
    AP_Int8     rssi_pin;
    AP_Int8     battery_volt_pin;
    AP_Int8     battery_curr_pin;

#if HIL_MODE != HIL_MODE_ATTITUDE
#if CONFIG_SONAR == ENABLED     
    AP_Int8	    sonar_enabled;
	AP_Int8	    sonar_type;   // 0 = XL, 1 = LV,
    // 2 = XLL (XL with 10m range)   
    // 3 = HRLV 
#endif
#endif

// ************ ThermoPilot parameters  ************************ 
//  - JLN update

    AP_Float    sonar_trigger;
    AP_Int8     booster;
        
// ************************************************************   

    // RC channels
    RC_Channel      channel_steer;
    RC_Channel_aux	rc_2;
    RC_Channel      channel_throttle;
    RC_Channel_aux  rc_4;
    RC_Channel_aux	rc_5;
    RC_Channel_aux	rc_6;
    RC_Channel_aux	rc_7;
    RC_Channel_aux	rc_8;

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
        pidNavSteer         (0.7,             0.1,             0.2,             200),
        pidServoSteer       (0.5,             0.1,             0.2,             200),
        pidSpeedThrottle    (0.7,             0.2,             0.2,             200)
        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H

