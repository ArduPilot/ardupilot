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
    static const uint16_t k_format_version = 2;

    //
    // Parameter identities.
    //
    // The enumeration defined here is used to ensure that every parameter
    // or parameter group has a unique ID number.  This number is used by
    // AP_Var to store and locate parameters in EEPROM.
    //
    // Note that entries without a number are assigned the next number after
    // the entry preceding them.  When adding new entries, ensure that they
    // don't overlap.
    //
    // Try to group related variables together, and assign them a set
    // range in the enumeration.  Place these groups in numerical order
    // at the end of the enumeration.
    //
    // WARNING: Care should be taken when editing this enumeration as the
    //          AP_Var load/save code depends on the values here to identify
    //          variables saved in EEPROM.
    //
    //
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,


        // Misc
        //
        k_param_log_bitmask,
        k_param_frame_type,

        //
        // 140: Sensor parameters
        //
        k_param_IMU_calibration = 140,
        k_param_ground_temperature,
        k_param_ground_pressure,
		k_param_current,
		k_param_milliamp_hours,
		k_param_compass_enabled,
		k_param_compass,
		k_param_sonar,

        //
        // 160: Navigation parameters
        //
        k_param_crosstrack_gain = 160,
        k_param_crosstrack_entry_angle,
        k_param_pitch_max,
        k_param_RTL_altitude,

        //
        // 180: Radio settings
        //
        k_param_rc_1 = 180,
        k_param_rc_2,
        k_param_rc_3,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,
        k_param_rc_9,
        k_param_rc_10,
        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_action,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,
        k_param_flight_mode_channel,
        k_param_flight_modes,

        //
        // 220: Waypoint data
        //
        k_param_waypoint_mode = 220,
        k_param_waypoint_total,
        k_param_waypoint_index,
        k_param_command_must_index,
        k_param_waypoint_radius,
        k_param_loiter_radius,

        //
        // 240: PID Controllers
        //
        // Heading-to-roll PID:
        // heading error from commnd to roll command deviation from trim
        // (bank to turn strategy)
        //
		k_param_pid_acro_rate_roll = 240,
		k_param_pid_acro_rate_pitch,
		k_param_pid_acro_rate_yaw,
		k_param_pid_stabilize_roll,
		k_param_pid_stabilize_pitch,
		k_param_pid_yaw,
		k_param_pid_nav_lat,
		k_param_pid_nav_lon,
		k_param_pid_baro_throttle,
		k_param_pid_sonar_throttle,
		// special D term alternatives
		k_param_stabilize_dampener,
		k_param_hold_yaw_dampener,


        // 255: reserved
    };

    AP_Int16    format_version;


    // Crosstrack navigation
    //
    AP_Float    crosstrack_gain;
    AP_Int16    crosstrack_entry_angle;

    // Waypoints
    //
    AP_Int8     waypoint_mode;
    AP_Int8     waypoint_total;
    AP_Int8     waypoint_index;
    AP_Int8		command_must_index;
    AP_Int8     waypoint_radius;
    AP_Int8     loiter_radius;

    // Throttle
    //
    AP_Int16    throttle_min;
    AP_Int16    throttle_max;
    AP_Int8     throttle_fs_enabled;
    AP_Int8     throttle_fs_action;
    AP_Int16    throttle_fs_value;
    AP_Int16    throttle_cruise;

    // Flight modes
    //
    AP_Int8     flight_mode_channel;
    AP_VarA<uint8_t,6> flight_modes;

    // Radio settings
    //
    //AP_Var_group pwm_roll;
    //AP_Var_group pwm_pitch;
    //AP_Var_group pwm_throttle;
    //AP_Var_group pwm_yaw;

    AP_Int16     pitch_max;

    // Misc
    //
    AP_Int16    log_bitmask;
    AP_Int16    ground_temperature;
    AP_Int16    ground_pressure;
    AP_Int16    RTL_altitude;
    AP_Int8		frame_type;

    AP_Int8		sonar_enabled;
    AP_Int8		current_enabled;
    AP_Int16	milliamp_hours;
    AP_Int8		compass_enabled;


    // RC channels
	RC_Channel	rc_1;
	RC_Channel	rc_2;
	RC_Channel	rc_3;
	RC_Channel	rc_4;
	RC_Channel	rc_5;
	RC_Channel	rc_6;
	RC_Channel	rc_7;
	RC_Channel	rc_8;
	RC_Channel	rc_camera_pitch;
	RC_Channel	rc_camera_roll;

    // PID controllers
	PID			pid_acro_rate_roll;
	PID			pid_acro_rate_pitch;
	PID			pid_acro_rate_yaw;
	PID			pid_stabilize_roll;
	PID			pid_stabilize_pitch;
	PID			pid_yaw;
	PID			pid_nav_lat;
	PID			pid_nav_lon;
	PID			pid_baro_throttle;
	PID			pid_sonar_throttle;

	AP_Float 	stabilize_dampener;
	AP_Float 	hold_yaw_dampener;

    uint8_t     junk;

    Parameters() :
        // variable             default                     key										name
        //-------------------------------------------------------------------------------------------------------------------
        format_version          (k_format_version,          k_param_format_version,        			NULL),

        crosstrack_gain         (XTRACK_GAIN * 100,			k_param_crosstrack_gain,        		PSTR("XTRK_GAIN")),
        crosstrack_entry_angle  (XTRACK_ENTRY_ANGLE * 100,	k_param_crosstrack_entry_angle, 		PSTR("XTRACK_ANGLE")),

        frame_type  			(FRAME_CONFIG,				k_param_frame_type,						PSTR("FRAME_CONFIG")),

        sonar_enabled  			(DISABLED,					k_param_sonar,							PSTR("SONAR_ENABLE")),
        current_enabled  		(DISABLED,					k_param_current,						PSTR("CURRENT_ENABLE")),
        milliamp_hours  		(CURR_AMP_HOURS,			k_param_milliamp_hours,					PSTR("MAH")),
        compass_enabled			(MAGNETOMETER,				k_param_compass_enabled,				PSTR("MAG_ENABLE")),

        waypoint_mode           (0,                         k_param_waypoint_mode,          		PSTR("WP_MODE")),
        waypoint_total          (0,                         k_param_waypoint_total,         		PSTR("WP_TOTAL")),
        waypoint_index          (0,                         k_param_waypoint_index,         		PSTR("WP_INDEX")),
        command_must_index      (0,                         k_param_command_must_index,     		PSTR("WP_MUST_INDEX")),
        waypoint_radius         (WP_RADIUS_DEFAULT,         k_param_waypoint_radius,        		PSTR("WP_RADIUS")),
        loiter_radius           (LOITER_RADIUS_DEFAULT,     k_param_loiter_radius,          		PSTR("LOITER_RADIUS")),

        throttle_min            (THROTTLE_MIN,              k_param_throttle_min,					PSTR("THR_MIN")),
        throttle_max            (THROTTLE_MAX,              k_param_throttle_max,					PSTR("THR_MAX")),
        throttle_fs_enabled   	(THROTTLE_FAILSAFE,         k_param_throttle_fs_enabled,			PSTR("THR_FAILSAFE")),
        throttle_fs_action		(THROTTLE_FAILSAFE_ACTION,  k_param_throttle_fs_action, 			PSTR("THR_FS_ACTION")),
        throttle_fs_value 		(THROTTLE_FS_VALUE,         k_param_throttle_fs_value, 				PSTR("THR_FS_VALUE")),
        throttle_cruise         (THROTTLE_CRUISE,           k_param_throttle_cruise,    			PSTR("TRIM_THROTTLE")),

        flight_mode_channel     (FLIGHT_MODE_CHANNEL+1,       k_param_flight_mode_channel,   		PSTR("FLT_MODE_CH")),
        flight_modes            (k_param_flight_modes,                                     			PSTR("FLIGHT_MODES")),

        pitch_max         		(PITCH_MAX * 100,			k_param_pitch_max,		       			PSTR("PITCH_MAX")),

        log_bitmask             (MASK_LOG_SET_DEFAULTS,		k_param_log_bitmask,            		PSTR("LOG_BITMASK")),
        ground_temperature      (0,                         k_param_ground_temperature,    			PSTR("GND_TEMP")),
        ground_pressure         (0,                         k_param_ground_pressure,       			PSTR("GND_ABS_PRESS")),
        RTL_altitude            (ALT_HOLD_HOME * 100,		k_param_RTL_altitude,          			PSTR("ALT_HOLD_RTL")),

        // RC channel           group key                   name
        //----------------------------------------------------------------------
        rc_1					(k_param_rc_1,		PSTR("RC1_")),
        rc_2					(k_param_rc_2,		PSTR("RC2_")),
        rc_3					(k_param_rc_3,		PSTR("RC3_")),
        rc_4					(k_param_rc_4,		PSTR("RC4_")),
        rc_5					(k_param_rc_5,		PSTR("RC5_")),
        rc_6					(k_param_rc_6,		PSTR("RC6_")),
        rc_7					(k_param_rc_7,		PSTR("RC7_")),
        rc_8					(k_param_rc_8,		PSTR("RC8_")),
        rc_camera_pitch			(k_param_rc_9,		PSTR("RC9_")),
        rc_camera_roll			(k_param_rc_10,		PSTR("RC10_")),

        // PID controller   group key						name				initial P			initial I			initial D			initial imax
        //--------------------------------------------------------------------------------------------------------------------------------------------------------------------
		pid_acro_rate_roll	(k_param_pid_acro_rate_roll,	PSTR("ACR_RLL_"),	ACRO_RATE_ROLL_P,   ACRO_RATE_ROLL_I,	ACRO_RATE_ROLL_D,	ACRO_RATE_ROLL_IMAX * 100),
		pid_acro_rate_pitch	(k_param_pid_acro_rate_pitch,	PSTR("ACR_PIT_"),	ACRO_RATE_PITCH_P,  ACRO_RATE_PITCH_I,	ACRO_RATE_PITCH_D,	ACRO_RATE_PITCH_IMAX * 100),
		pid_acro_rate_yaw	(k_param_pid_acro_rate_yaw,		PSTR("ACR_YAW_"),	ACRO_RATE_YAW_P,    ACRO_RATE_YAW_I,	ACRO_RATE_YAW_D,	ACRO_RATE_YAW_IMAX * 100),

		pid_stabilize_roll	(k_param_pid_stabilize_roll,	PSTR("STB_RLL_"),	STABILIZE_ROLL_P,   STABILIZE_ROLL_I,	STABILIZE_ROLL_D,   STABILIZE_ROLL_IMAX * 100),
		pid_stabilize_pitch	(k_param_pid_stabilize_pitch,	PSTR("STB_PIT_"),	STABILIZE_PITCH_P,  STABILIZE_PITCH_I,	STABILIZE_PITCH_D,  STABILIZE_PITCH_IMAX * 100),
		pid_yaw				(k_param_pid_yaw,				PSTR("STB_YAW_"),	YAW_P,      		YAW_I,				YAW_D,				YAW_IMAX * 100),

		pid_nav_lat			(k_param_pid_nav_lat,			PSTR("NAV_LAT_"),	NAV_P,      		NAV_I,				NAV_D,				NAV_IMAX * 100),
		pid_nav_lon			(k_param_pid_nav_lon,			PSTR("NAV_LON_"),	NAV_P,      		NAV_I,				NAV_D,				NAV_IMAX * 100),

		pid_baro_throttle	(k_param_pid_baro_throttle,		PSTR("THR_BAR_"),	THROTTLE_BARO_P,    THROTTLE_BARO_I,	THROTTLE_BARO_D,	THROTTLE_BARO_IMAX * 100),
		pid_sonar_throttle	(k_param_pid_sonar_throttle,	PSTR("THR_SON_"),	THROTTLE_SONAR_P,   THROTTLE_SONAR_I,	THROTTLE_SONAR_D,	THROTTLE_SONAR_IMAX * 100),

		stabilize_dampener	(STABILIZE_DAMPENER,	k_param_stabilize_dampener, 	PSTR("STB_DAMP")),
		hold_yaw_dampener	(HOLD_YAW_DAMPENER, 	k_param_hold_yaw_dampener, 		PSTR("YAW_DAMP")),

        junk(0)     // XXX just so that we can add things without worrying about the trailing comma
    {
    }
};

#endif // PARAMETERS_H
