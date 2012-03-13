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
	static const uint16_t k_format_version = 117;

	// The parameter software_type is set up solely for ground station use
	// and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
	// GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
	// values within that range to identify different branches.
	//
	static const uint16_t k_software_type = 10;		// 0 for APM trunk

	// Parameter identities.
	//
	// The enumeration defined here is used to ensure that every parameter
	// or parameter group has a unique ID number.	This number is used by
	// AP_Var to store and locate parameters in EEPROM.
	//
	// Note that entries without a number are assigned the next number after
	// the entry preceding them.	When adding new entries, ensure that they
	// don't overlap.
	//
	// Try to group related variables together, and assign them a set
	// range in the enumeration.	Place these groups in numerical order
	// at the end of the enumeration.
	//
	// WARNING: Care should be taken when editing this enumeration as the
	//			AP_Var load/save code depends on the values here to identify
	//			variables saved in EEPROM.
	//
	//
	enum {
	// Layout version number, always key zero.
	//
	k_param_format_version = 0,
	k_param_software_type,


	// Misc
	//
	k_param_log_bitmask = 20,
    k_param_log_last_filenumber,		// *** Deprecated - remove with next eeprom number change

	#if FRAME_CONFIG ==	HELI_FRAME
	//
	// 80: Heli
	//
	k_param_heli_servo_1 = 80,
	k_param_heli_servo_2,
	k_param_heli_servo_3,
	k_param_heli_servo_4,
	k_param_heli_servo1_pos ,
	k_param_heli_servo2_pos,
	k_param_heli_servo3_pos,
	k_param_heli_roll_max,
	k_param_heli_pitch_max,
	k_param_heli_collective_min,
	k_param_heli_collective_max,
	k_param_heli_collective_mid,
	k_param_heli_ext_gyro_enabled,
	k_param_heli_ext_gyro_gain,
	k_param_heli_servo_averaging,
	k_param_heli_servo_manual,
	k_param_heli_phase_angle,
	k_param_heli_collective_yaw_effect,
	k_param_heli_h1_swash_enabled, //98
	#endif

	// 110: Telemetry control
	//
	k_param_gcs0 = 110,
	k_param_gcs3,
	k_param_sysid_this_mav,
	k_param_sysid_my_gcs,
    k_param_serial3_baud,

	//
	// 140: Sensor parameters
	//
	k_param_imu = 140, // sensor calibration
    k_param_battery_monitoring,
    k_param_volt_div_ratio,
    k_param_curr_amp_per_volt,
    k_param_input_voltage,
	k_param_pack_capacity,
	k_param_compass_enabled,
	k_param_compass,
	k_param_sonar_enabled,
	k_param_frame_orientation,
	k_param_top_bottom_ratio,
	k_param_optflow_enabled,
	k_param_low_voltage,
	k_param_ch7_option,
	k_param_auto_slew_rate,
	k_param_sonar_type,
	k_param_super_simple, //155
	k_param_rtl_land_enabled,
	k_param_axis_enabled,

	//
	// 160: Navigation parameters
	//
	k_param_RTL_altitude = 160,
	k_param_crosstrack_gain,
	k_param_auto_land_timeout,


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
	k_param_rc_camera_pitch,// rc_9
	k_param_rc_camera_roll, // rc_10
	k_param_throttle_min,
	k_param_throttle_max,
	k_param_throttle_fs_enabled,
	k_param_throttle_fs_action,
	k_param_throttle_fs_value,
	k_param_throttle_cruise,
	k_param_esc_calibrate,
	k_param_radio_tuning,
	k_param_radio_tuning_high,
	k_param_radio_tuning_low,
	k_param_camera_pitch_gain,
	k_param_camera_roll_gain,
    k_param_rc_speed,

    //
    // 200: flight modes
    //
    k_param_flight_mode1 = 200,
    k_param_flight_mode2,
    k_param_flight_mode3,
    k_param_flight_mode4,
    k_param_flight_mode5,
    k_param_flight_mode6,
    k_param_simple_modes,

	//
	// 210: Waypoint data
	//
	k_param_waypoint_mode = 210,
	k_param_command_total,
	k_param_command_index,
	k_param_command_nav_index,
	k_param_waypoint_radius,
	k_param_loiter_radius,
	k_param_waypoint_speed_max,

	//
	// 220: PI/D Controllers
	//
	k_param_stabilize_d_schedule = 219,
	k_param_stabilize_d = 220,
	k_param_acro_p,
	k_param_axis_lock_p,
	k_param_pid_rate_roll,
	k_param_pid_rate_pitch,
	k_param_pid_rate_yaw,
	k_param_pi_stabilize_roll,
	k_param_pi_stabilize_pitch,
	k_param_pi_stabilize_yaw,
	k_param_pi_loiter_lat,
	k_param_pi_loiter_lon,
	k_param_pid_loiter_rate_lat,
	k_param_pid_loiter_rate_lon,
	k_param_pid_nav_lat,
	k_param_pid_nav_lon,
	k_param_pi_alt_hold,
	k_param_pid_throttle,
	k_param_pid_optflow_roll,
	k_param_pid_optflow_pitch,

    // 254,255: reserved
	};

	AP_Int16	format_version;
	AP_Int8		software_type;

	// Telemetry control
	//
	AP_Int16	sysid_this_mav;
	AP_Int16	sysid_my_gcs;
    AP_Int8		serial3_baud;


	AP_Int16	RTL_altitude;
	AP_Int8		sonar_enabled;
	AP_Int8		sonar_type;   // 0 = XL, 1 = LV, 2 = XLL (XL with 10m range)
    AP_Int8		battery_monitoring;	// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Float	volt_div_ratio;
    AP_Float	curr_amp_per_volt;
    AP_Float	input_voltage;
	AP_Int16	pack_capacity;		// Battery pack capacity less reserve
	AP_Int8		compass_enabled;
    AP_Int8		optflow_enabled;
	AP_Float	low_voltage;
	AP_Int8		super_simple;
	AP_Int8		rtl_land_enabled;
	AP_Int8		axis_enabled;



	// Waypoints
	//
	AP_Int8		waypoint_mode;
	AP_Int8		command_total;
	AP_Int8		command_index;
	AP_Int8		command_nav_index;
	AP_Int8		waypoint_radius;
	AP_Int16	loiter_radius;
	AP_Int16	waypoint_speed_max;
	AP_Float	crosstrack_gain;
	AP_Int32	auto_land_timeout;


	// Throttle
	//
	AP_Int16	throttle_min;
	AP_Int16	throttle_max;
	AP_Int8		throttle_fs_enabled;
	AP_Int8		throttle_fs_action;
	AP_Int16	throttle_fs_value;
	AP_Int16	throttle_cruise;

	// Flight modes
	//
    AP_Int8     flight_mode1;
    AP_Int8     flight_mode2;
    AP_Int8     flight_mode3;
    AP_Int8     flight_mode4;
    AP_Int8     flight_mode5;
    AP_Int8     flight_mode6;
    AP_Int8     simple_modes;

	// Misc
	//
	AP_Int16	log_bitmask;
    AP_Int16	log_last_filenumber;		// *** Deprecated - remove with next eeprom number change

	AP_Int8		esc_calibrate;
	AP_Int8		radio_tuning;
	AP_Int16	radio_tuning_high;
	AP_Int16	radio_tuning_low;
	AP_Int8		frame_orientation;
	AP_Float	top_bottom_ratio;
	AP_Int8		ch7_option;
	AP_Int16	auto_slew_rate;

	#if FRAME_CONFIG ==	HELI_FRAME
	// Heli
	RC_Channel	heli_servo_1, heli_servo_2, heli_servo_3, heli_servo_4;	// servos for swash plate and tail
	AP_Int16	heli_servo1_pos, heli_servo2_pos, heli_servo3_pos;		// servo positions (3 because we don't need pos for tail servo)
	AP_Int16	heli_roll_max, heli_pitch_max;	// maximum allowed roll and pitch of swashplate
	AP_Int16	heli_collective_min, heli_collective_max, heli_collective_mid;		// min and max collective.	mid = main blades at zero pitch
	AP_Int8		heli_ext_gyro_enabled;	// 0 = no external tail gyro, 1 = external tail gyro
	AP_Int16	heli_ext_gyro_gain;		// radio output 1000~2000 (value output on CH_7)
	AP_Int8		heli_servo_averaging;	// 0 or 1 = no averaging (250hz) for **digital servos**, 2=average of two samples (125hz), 3=three samples (83.3hz) **analog servos**, 4=four samples (62.5hz), 5=5 samples(50hz)
	AP_Int8		heli_servo_manual;	    // 0 = normal mode, 1 = radio inputs directly control swash.  required for swash set-up
	AP_Int16	heli_phase_angle;		// 0 to 360 degrees.  specifies mixing between roll and pitch for helis
	AP_Float	heli_collective_yaw_effect;	// -5.0 ~ 5.0.  Feed forward control from collective to yaw.  1.0 = move rudder right 1% for every 1% of collective above the mid point
	AP_Int8		heli_h1_swash_enabled; 	// 0 = CCPM swashplate, 1 = H1 swashplate (no servo mixing)
	#endif

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
    AP_Int16    rc_speed; // speed of fast RC Channels in Hz

	AP_Float	camera_pitch_gain;
	AP_Float	camera_roll_gain;
	AP_Float	stabilize_d;
	AP_Float	stabilize_d_schedule;

	// PI/D controllers
    AP_Float	acro_p;
	AP_Float	axis_lock_p;

	AC_PID		pid_rate_roll;
	AC_PID		pid_rate_pitch;
	AC_PID		pid_rate_yaw;
	AC_PID		pid_loiter_rate_lat;
	AC_PID		pid_loiter_rate_lon;
	AC_PID		pid_nav_lat;
	AC_PID		pid_nav_lon;

	AC_PID		pid_throttle;
	AC_PID		pid_optflow_roll;
	AC_PID		pid_optflow_pitch;

	APM_PI		pi_loiter_lat;
	APM_PI		pi_loiter_lon;
	APM_PI		pi_stabilize_roll;
	APM_PI		pi_stabilize_pitch;
	APM_PI		pi_stabilize_yaw;
	APM_PI		pi_alt_hold;

	// Note: keep initializers here in the same order as they are declared above.
	Parameters() :
	// variable				default
	//----------------------------------------
    format_version			(k_format_version),
	software_type			(k_software_type),

	sysid_this_mav			(MAV_SYSTEM_ID),
	sysid_my_gcs			(255),
    serial3_baud			(SERIAL3_BAUD/1000),

	RTL_altitude			(ALT_HOLD_HOME * 100),
	sonar_enabled			(DISABLED),
	sonar_type				(AP_RANGEFINDER_MAXSONARXL),
    battery_monitoring 		(DISABLED),
    volt_div_ratio			(VOLT_DIV_RATIO),
    curr_amp_per_volt		(CURR_AMP_PER_VOLT),
    input_voltage			(INPUT_VOLTAGE),
	pack_capacity			(HIGH_DISCHARGE),
	compass_enabled			(MAGNETOMETER),
	optflow_enabled			(OPTFLOW),
	low_voltage				(LOW_VOLTAGE),
	super_simple			(SUPER_SIMPLE),
	rtl_land_enabled		(RTL_AUTO_LAND),
	axis_enabled			(AXIS_LOCK_ENABLED),

	waypoint_mode			(0),
	command_total			(0),
	command_index			(0),
	command_nav_index		(0),
	waypoint_radius			(WP_RADIUS_DEFAULT * 100),
	loiter_radius			(LOITER_RADIUS),
	waypoint_speed_max		(WAYPOINT_SPEED_MAX),
	crosstrack_gain			(CROSSTRACK_GAIN),
	auto_land_timeout		(AUTO_LAND_TIME * 1000),

	throttle_min			(0),
	throttle_max			(1000),
	throttle_fs_enabled		(THROTTLE_FAILSAFE),
	throttle_fs_action		(THROTTLE_FAILSAFE_ACTION),
	throttle_fs_value 		(THROTTLE_FS_VALUE),
	throttle_cruise			(THROTTLE_CRUISE),

    flight_mode1            (FLIGHT_MODE_1),
    flight_mode2            (FLIGHT_MODE_2),
    flight_mode3            (FLIGHT_MODE_3),
    flight_mode4            (FLIGHT_MODE_4),
    flight_mode5            (FLIGHT_MODE_5),
    flight_mode6            (FLIGHT_MODE_6),
    simple_modes            (0),

	log_bitmask				(DEFAULT_LOG_BITMASK),
    log_last_filenumber     (0),
	esc_calibrate 			(0),
	radio_tuning 			(0),
	radio_tuning_high 		(1000),
	radio_tuning_low 		(0),
	frame_orientation 		(FRAME_ORIENTATION),
	top_bottom_ratio 		(TOP_BOTTOM_RATIO),
	ch7_option 				(CH7_OPTION),
	auto_slew_rate			(AUTO_SLEW_RATE),

	#if FRAME_CONFIG ==	HELI_FRAME
	heli_servo1_pos				(-60),
	heli_servo2_pos				(60),
	heli_servo3_pos				(180),
	heli_roll_max				(4500),
	heli_pitch_max				(4500),
	heli_collective_min			(1250),
	heli_collective_max			(1750),
	heli_collective_mid			(1500),
	heli_ext_gyro_enabled		(0),
	heli_ext_gyro_gain			(1350),
	heli_servo_averaging		(0),
	heli_servo_manual			(0),
	heli_phase_angle			(0),
	heli_collective_yaw_effect	(0),
	heli_h1_swash_enabled		(0),
	#endif

    rc_speed(RC_FAST_SPEED),

	camera_pitch_gain 		(CAM_PITCH_GAIN),
	camera_roll_gain 		(CAM_ROLL_GAIN),
	stabilize_d 			(STABILIZE_D),
	stabilize_d_schedule	(STABILIZE_D_SCHEDULE),
	acro_p					(ACRO_P),
	axis_lock_p				(AXIS_LOCK_P),

	// PID controller	initial P	   	    initial I		 	initial D       	initial imax
	//-----------------------------------------------------------------------------------------------------
	pid_rate_roll       (RATE_ROLL_P,       RATE_ROLL_I,        RATE_ROLL_D,    	RATE_ROLL_IMAX * 100),
	pid_rate_pitch      (RATE_PITCH_P,      RATE_PITCH_I,       RATE_PITCH_D,   	RATE_PITCH_IMAX * 100),
	pid_rate_yaw        (RATE_YAW_P,        RATE_YAW_I,         RATE_YAW_D,     	RATE_YAW_IMAX * 100),

	pid_loiter_rate_lat	(LOITER_RATE_P,		LOITER_RATE_I,		LOITER_RATE_D,		LOITER_RATE_IMAX * 100),
	pid_loiter_rate_lon	(LOITER_RATE_P,		LOITER_RATE_I,		LOITER_RATE_D,		LOITER_RATE_IMAX * 100),

	pid_nav_lat			(NAV_P,				NAV_I,				NAV_D,				NAV_IMAX * 100),
	pid_nav_lon			(NAV_P,				NAV_I,				NAV_D,				NAV_IMAX * 100),

	pid_throttle		(THROTTLE_P,		THROTTLE_I,			THROTTLE_D,			THROTTLE_IMAX),
	pid_optflow_roll	(OPTFLOW_ROLL_P,	OPTFLOW_ROLL_I,		OPTFLOW_ROLL_D,		OPTFLOW_IMAX * 100),
	pid_optflow_pitch	(OPTFLOW_PITCH_P,	OPTFLOW_PITCH_I,	OPTFLOW_PITCH_D,	OPTFLOW_IMAX * 100),

	// PI controller	initial P			initial I			initial imax
	//----------------------------------------------------------------------
	pi_stabilize_roll	(STABILIZE_ROLL_P,	STABILIZE_ROLL_I,	STABILIZE_ROLL_IMAX * 100),
	pi_stabilize_pitch	(STABILIZE_PITCH_P,	STABILIZE_PITCH_I,	STABILIZE_PITCH_IMAX * 100),
	pi_stabilize_yaw	(STABILIZE_YAW_P,	STABILIZE_YAW_I,	STABILIZE_YAW_IMAX * 100),

	pi_alt_hold			(ALT_HOLD_P,		ALT_HOLD_I,			ALT_HOLD_IMAX),
	pi_loiter_lat		(LOITER_P,			LOITER_I,			LOITER_IMAX * 100),
	pi_loiter_lon		(LOITER_P,			LOITER_I,			LOITER_IMAX * 100)
	{
	}
};

#endif // PARAMETERS_H

