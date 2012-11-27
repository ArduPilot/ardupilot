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
        k_param_auto_trim,
        k_param_switch_enable,
        k_param_log_bitmask,
        k_param_mix_mode, // unused
        k_param_reverse_elevons, // unused
        k_param_reverse_ch1_elevon, // unused
        k_param_reverse_ch2_elevon, // unused
        k_param_flap_1_percent, // unused
        k_param_flap_1_speed, // unused
        k_param_flap_2_percent, // unused
        k_param_flap_2_speed, // unused
        k_param_num_resets,
        k_param_log_last_filenumber,		// *** Deprecated - remove with next eeprom number change
        k_param_reset_switch_chan,
        k_param_manual_level,
        k_param_ins,                        // libraries/AP_InertialSensor variables        


	// 110: Telemetry control
	//
	k_param_gcs0 = 110, // stream rates for port0
	k_param_gcs3,       // stream rates for port3
	k_param_sysid_this_mav,
	k_param_sysid_my_gcs,
        k_param_serial3_baud,
        k_param_telem_delay,

        // 120: Fly-by-wire control
        //
        k_param_flybywire_airspeed_min = 120,
        k_param_flybywire_airspeed_max,
        k_param_FBWB_min_altitude,  // 0=disabled, minimum value for altitude in cm (for first time try 30 meters = 3000 cm)
  
        //
        // 130: Sensor parameters
        //
        k_param_imu = 130,  
        k_param_altitude_mix,   // sensor calibration
        k_param_airspeed_ratio,
        k_param_ground_temperature,
        k_param_ground_pressure,
	k_param_compass_enabled,
	k_param_compass,
	k_param_battery_monitoring,
	k_param_volt_div_ratio,
	k_param_curr_amp_per_volt,
	k_param_input_voltage,
	k_param_pack_capacity,
        k_param_airspeed_offset,
#if HIL_MODE != HIL_MODE_ATTITUDE
#if CONFIG_SONAR == ENABLED       
 	k_param_sonar_enabled,
 	k_param_sonar_type,
#endif
#endif
        k_param_ahrs,  // AHRS group
        
        //
        // 150: Navigation parameters
        //
        k_param_crosstrack_gain = 150,
        k_param_crosstrack_entry_angle,
        k_param_roll_limit,
        k_param_pitch_limit_max,
        k_param_pitch_limit_min,
        k_param_airspeed_cruise,
        k_param_min_gndspeed,
        k_param_ch7_option,
        //
        // 160: Radio settings
        //
        k_param_channel_roll = 160,
        k_param_channel_pitch,
        k_param_channel_throttle,
        k_param_channel_rudder,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled, // 170
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_short_fs_action,
        k_param_long_fs_action,
	k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,

 // ************************************************************
        // 180: APMrover parameters - JLN update
        
        k_param_closed_loop_nav,
        k_param_auto_wp_radius,
        k_param_sonar_trigger,
        k_param_turn_gain,
        k_param_booster,        
        
// ************************************************************
//
        // 200: Feed-forward gains
        //
        k_param_kff_pitch_compensation = 200,
        k_param_kff_rudder_mix,
        k_param_kff_pitch_to_throttle,
        k_param_kff_throttle_to_pitch,

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

        //
        // 240: PID Controllers
        //
        // Heading-to-roll PID:
        // heading error from command to roll command deviation from trim
        // (bank to turn strategy)
        //
        k_param_pidNavRoll = 240,

        // Roll-to-servo PID:
        // roll error from command to roll servo deviation from trim
        // (tracks commanded bank angle)
        //
        k_param_pidServoRoll,

        //
        // Pitch control
        //
        // Pitch-to-servo PID:
        // pitch error from command to pitch servo deviation from trim
        // (front-side strategy)
        //
        k_param_pidServoPitch,

        // Airspeed-to-pitch PID:
        // airspeed error from command to pitch servo deviation from trim
        // (back-side strategy)
        //
        k_param_pidNavPitchAirspeed,

        //
        // Yaw control
        //
        // Yaw-to-servo PID:
        // yaw rate error from command to yaw servo deviation from trim
        // (stabilizes dutch roll)
        //
        k_param_pidServoRudder,

        //
        // Throttle control
        //
        // Energy-to-throttle PID:
        // total energy error from command to throttle servo deviation from trim
        // (throttle back-side strategy alternative)
        //
        k_param_pidTeThrottle,

        // Altitude-to-pitch PID:
        // altitude error from command to pitch servo deviation from trim
        // (throttle front-side strategy alternative)
        //
        k_param_pidNavPitchAltitude,

        // 254,255: reserved
        };

        AP_Int16    format_version;
	AP_Int8	    software_type;

	// Telemetry control
	//
	AP_Int16    sysid_this_mav;
	AP_Int16    sysid_my_gcs;
        AP_Int8	    serial3_baud;
        AP_Int8     telem_delay;

        // Feed-forward gains
        //
        AP_Float    kff_pitch_compensation;
        AP_Float    kff_rudder_mix;
        AP_Float    kff_pitch_to_throttle;
        AP_Float    kff_throttle_to_pitch;
    
        // Crosstrack navigation
        //
        AP_Float    crosstrack_gain;
        AP_Int16    crosstrack_entry_angle;
    
        // Estimation
        //
        AP_Float    altitude_mix;
        AP_Float    airspeed_ratio;
	AP_Int16    airspeed_offset;

        // Waypoints
        //
        AP_Int8     waypoint_mode;
        AP_Int8     command_total;
        AP_Int8     command_index;
        AP_Int8     waypoint_radius;
        AP_Int8     loiter_radius;
    
        // Fly-by-wire
        //
        AP_Int8     flybywire_airspeed_min;
        AP_Int8     flybywire_airspeed_max;
        AP_Int16    FBWB_min_altitude;
    
        // Throttle
        //
        AP_Int8     throttle_min;
        AP_Int8     throttle_max;
        AP_Int8     throttle_slewrate;
    	AP_Int8     throttle_fs_enabled;
        AP_Int16    throttle_fs_value;
        AP_Int8     throttle_cruise;
    
    	// Failsafe
        AP_Int8     short_fs_action;
        AP_Int8     long_fs_action;
	AP_Int8	    gcs_heartbeat_fs_enabled;

        // Flight modes
        //
        AP_Int8     flight_mode_channel;
        AP_Int8     flight_mode1;
        AP_Int8     flight_mode2;
        AP_Int8     flight_mode3;
        AP_Int8     flight_mode4;
        AP_Int8     flight_mode5;
        AP_Int8     flight_mode6;
    
        // Navigational maneuvering limits
        //
        AP_Int16    roll_limit;
        AP_Int16    pitch_limit_max;
        AP_Int16    pitch_limit_min;
    
        // Misc
        //
        AP_Int8     auto_trim;
        AP_Int16    num_resets;
        AP_Int16    log_bitmask;
        AP_Int16    log_last_filenumber;		// *** Deprecated - remove with next eeprom number change
        AP_Int8	    reset_switch_chan;
        AP_Int8	    manual_level;	
        AP_Int16    airspeed_cruise;
        AP_Int16    min_gndspeed;
        AP_Int8	    ch7_option;
        
        AP_Int16    ground_temperature;
        AP_Int32    ground_pressure;
        AP_Int8	    compass_enabled;
        AP_Int16    angle_of_attack;
        AP_Int8	    battery_monitoring;	// 0=disabled, 3=voltage only, 4=voltage and current
        AP_Float    volt_div_ratio;
        AP_Float    curr_amp_per_volt;
        AP_Float    input_voltage;
	AP_Int16    pack_capacity;		// Battery pack capacity less reserve
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

        AP_Int8     closed_loop_nav;
        AP_Int8     auto_wp_radius;
        AP_Int16    sonar_trigger;
        AP_Int16    turn_gain;
        AP_Int8     booster;
        
// ************************************************************   

        // RC channels
    RC_Channel  channel_roll;
    RC_Channel  channel_pitch;
    RC_Channel  channel_throttle;
    RC_Channel  channel_rudder;
    RC_Channel_aux	rc_5;
    RC_Channel_aux	rc_6;
    RC_Channel_aux	rc_7;
    RC_Channel_aux	rc_8;

        // PID controllers
        //
        PID         pidNavRoll;
        PID         pidServoRoll;
        PID         pidServoPitch;
        PID         pidNavPitchAirspeed;
        PID         pidServoRudder;
        PID         pidTeThrottle;
        PID         pidNavPitchAltitude;

    Parameters() :
        // RC channels
        channel_roll(CH_1),
        channel_pitch(CH_2),
        channel_throttle(CH_3),
        channel_rudder(CH_4),
        rc_5(CH_5),
        rc_6(CH_6),
        rc_7(CH_7),
        rc_8(CH_8),

        // PID controller    initial P        initial I        initial D        initial imax
        //-----------------------------------------------------------------------------------
        pidNavRoll          (NAV_ROLL_P,      NAV_ROLL_I,      NAV_ROLL_D,      NAV_ROLL_INT_MAX_CENTIDEGREE),
        pidServoRoll        (SERVO_ROLL_P,    SERVO_ROLL_I,    SERVO_ROLL_D,    SERVO_ROLL_INT_MAX_CENTIDEGREE),
        pidServoPitch       (SERVO_PITCH_P,   SERVO_PITCH_I,   SERVO_PITCH_D,   SERVO_PITCH_INT_MAX_CENTIDEGREE),
        pidNavPitchAirspeed (NAV_PITCH_ASP_P, NAV_PITCH_ASP_I, NAV_PITCH_ASP_D, NAV_PITCH_ASP_INT_MAX_CMSEC),
        pidServoRudder      (SERVO_YAW_P,     SERVO_YAW_I,     SERVO_YAW_D,     SERVO_YAW_INT_MAX),
        pidTeThrottle       (THROTTLE_TE_P,   THROTTLE_TE_I,   THROTTLE_TE_D,   THROTTLE_TE_INT_MAX),
        pidNavPitchAltitude (NAV_PITCH_ALT_P, NAV_PITCH_ALT_I, NAV_PITCH_ALT_D, NAV_PITCH_ALT_INT_MAX_CM)
        {}
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H

/* ************ ThermoPilot parameters  (old parameters setup ) ************************

        low_rate_turn           (LOW_RATE_TURN,             k_param_low_rate_turn,          PSTR("TP_LOWR_TURN")),
        medium_rate_turn        (MEDIUM_RATE_TURN,          k_param_medium_rate_turn,       PSTR("TP_MEDR_TURN")),
        high_rate_turn          (HIGH_RATE_TURN,            k_param_high_rate_turn,         PSTR("TP_HIGR_TURN")),
        search_mode_turn        (SEARCH_MODE_TURN,          k_param_search_mode_turn,       PSTR("TP_SRCM_TURN")),
        slope_thermal           (SLOPE_THERMAL,             k_param_slope_thermal,          PSTR("TP_SLOPE_THER")),
        auto_thermal            (AUTO_THERMAL,              k_param_auto_thermal,           PSTR("TP_AUTO_THER")),
        stab_thermal            (STAB_THERMAL,              k_param_auto_thermal,           PSTR("TP_STAB_THER")),
        closed_loop_nav         (CLOSED_LOOP_NAV,           k_param_closed_loop_nav,        PSTR("TP_CL_NAV")),
        auto_wp_radius          (AUTO_WP_RADIUS,            k_param_closed_loop_nav,        PSTR("TP_AWPR_NAV")), 
        min_alt                 (MIN_ALT,                   k_param_min_alt,                PSTR("TP_MIN_ALT")),        
        max_alt                 (MAX_ALT,                   k_param_max_alt,                PSTR("TP_MAX_ALT")), 
        max_dist                (MAX_DIST,                  k_param_max_dist,               PSTR("TP_MAX_DIST")), 
        sarsec_branch           (SARSEC_BRANCH,             k_param_sarsec_branch,          PSTR("TP_SARSEC")), 
       
 ************************************************************/

