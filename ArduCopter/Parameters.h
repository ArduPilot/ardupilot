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
	static const uint16_t k_format_version = 115;

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
	k_param_heli_coll_yaw_effect, // 97
	#endif

	// 110: Telemetry control
	//
	k_param_streamrates_port0 = 110,
	k_param_streamrates_port3,
	k_param_sysid_this_mav,
	k_param_sysid_my_gcs,
    k_param_serial3_baud,

	//
	// 140: Sensor parameters
	//
	k_param_IMU_calibration = 140,
    k_param_battery_monitoring,
    k_param_volt_div_ratio,
    k_param_curr_amp_per_volt,
    k_param_input_voltage,
	k_param_pack_capacity,
	k_param_compass_enabled,
	k_param_compass,
	k_param_sonar,
	k_param_frame_orientation,
	k_param_top_bottom_ratio,
	k_param_optflow_enabled,
	k_param_low_voltage,
	k_param_ch7_option,
	k_param_sonar_type,
	k_param_super_simple, //155

	//
	// 160: Navigation parameters
	//
	k_param_RTL_altitude = 160,
	k_param_crosstrack_gain,
	k_param_auto_land_timeout,


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
	k_param_esc_calibrate,
	k_param_radio_tuning,
	k_param_camera_pitch_gain,
	k_param_camera_roll_gain,

    //
    // 210: flight modes
    //
    k_param_flight_mode1,
    k_param_flight_mode2,
    k_param_flight_mode3,
    k_param_flight_mode4,
    k_param_flight_mode5,
    k_param_flight_mode6,
    k_param_simple_modes,

	//
	// 220: Waypoint data
	//
	k_param_waypoint_mode = 220,
	k_param_command_total,
	k_param_command_index,
	k_param_command_nav_index,
	k_param_waypoint_radius,
	k_param_loiter_radius,
	k_param_waypoint_speed_max,

	//
	// 235: PI/D Controllers
	//
	k_param_stabilize_d = 234,
	k_param_pid_rate_roll = 235,
	k_param_pid_rate_pitch,
	k_param_pid_rate_yaw,
	k_param_pi_stabilize_roll,
	k_param_pi_stabilize_pitch,
	k_param_pi_stabilize_yaw,
	k_param_pi_loiter_lat,
	k_param_pi_loiter_lon,
	k_param_pid_nav_lat,
	k_param_pid_nav_lon,
	k_param_pi_alt_hold,
	k_param_pid_throttle,
	k_param_pid_optflow_roll,
	k_param_pid_optflow_pitch,  // 250

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
	AP_Int8		frame_orientation;
	AP_Float	top_bottom_ratio;
	AP_Int8		ch7_option;


	#if FRAME_CONFIG ==	HELI_FRAME
	// Heli
	RC_Channel	heli_servo_1, heli_servo_2, heli_servo_3, heli_servo_4;	// servos for swash plate and tail
	AP_Int16	heli_servo1_pos, heli_servo2_pos, heli_servo3_pos;		// servo positions (3 because we don't need pos for tail servo)
	AP_Int16	heli_roll_max, heli_pitch_max;	// maximum allowed roll and pitch of swashplate
	AP_Int16	heli_coll_min, heli_coll_max, heli_coll_mid;		// min and max collective.	mid = main blades at zero pitch
	AP_Int8		heli_ext_gyro_enabled;	// 0 = no external tail gyro, 1 = external tail gyro
	AP_Int16	heli_ext_gyro_gain;		// radio output 1000~2000 (value output on CH_7)
	AP_Int8		heli_servo_averaging;	// 0 or 1 = no averaging (250hz) for **digital servos**, 2=average of two samples (125hz), 3=three samples (83.3hz) **analog servos**, 4=four samples (62.5hz), 5=5 samples(50hz)
	AP_Int8		heli_servo_manual;	    // 0 = normal mode, 1 = radio inputs directly control swash.  required for swash set-up
	AP_Int16	heli_phase_angle;		// 0 to 360 degrees.  specifies mixing between roll and pitch for helis
	AP_Float	heli_coll_yaw_effect;	// -5.0 ~ 5.0.  Feed forward control from collective to yaw.  1.0 = move rudder right 1% for every 1% of collective above the mid point
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

	AP_Float	camera_pitch_gain;
	AP_Float	camera_roll_gain;
	AP_Float	stablize_d;

	// PI/D controllers
	AC_PID		pid_rate_roll;
	AC_PID		pid_rate_pitch;
	AC_PID		pid_rate_yaw;
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

	uint8_t		junk;

	// Note: keep initializers here in the same order as they are declared above.
	Parameters() :
	// variable				default						key										name
	//-------------------------------------------------------------------------------------------------------------------
	format_version			(k_format_version,			k_param_format_version,					PSTR("SYSID_SW_MREV")),
	software_type			(k_software_type,			k_param_software_type,					PSTR("SYSID_SW_TYPE")),

	sysid_this_mav			(MAV_SYSTEM_ID,				k_param_sysid_this_mav,					PSTR("SYSID_THISMAV")),
	sysid_my_gcs			(255,						k_param_sysid_my_gcs,					PSTR("SYSID_MYGCS")),
    serial3_baud			(SERIAL3_BAUD/1000,         k_param_serial3_baud,					PSTR("SERIAL3_BAUD")),

	RTL_altitude			(ALT_HOLD_HOME * 100,		k_param_RTL_altitude,					PSTR("ALT_HOLD_RTL")),
	sonar_enabled			(DISABLED,					k_param_sonar,							PSTR("SONAR_ENABLE")),
	sonar_type				(AP_RANGEFINDER_MAXSONARXL,	k_param_sonar_type,						PSTR("SONAR_TYPE")),
    battery_monitoring 		(DISABLED,					k_param_battery_monitoring,		        PSTR("BATT_MONITOR")),
    volt_div_ratio			(VOLT_DIV_RATIO,			k_param_volt_div_ratio,			        PSTR("VOLT_DIVIDER")),
    curr_amp_per_volt		(CURR_AMP_PER_VOLT,			k_param_curr_amp_per_volt,		        PSTR("AMP_PER_VOLT")),
    input_voltage			(INPUT_VOLTAGE,				k_param_input_voltage,			        PSTR("INPUT_VOLTS")),
	pack_capacity			(HIGH_DISCHARGE,			k_param_pack_capacity,					PSTR("BATT_CAPACITY")),
	compass_enabled			(MAGNETOMETER,				k_param_compass_enabled,				PSTR("MAG_ENABLE")),
	optflow_enabled			(OPTFLOW,					k_param_optflow_enabled,				PSTR("FLOW_ENABLE")),
	low_voltage				(LOW_VOLTAGE,				k_param_low_voltage,					PSTR("LOW_VOLT")),
	super_simple			(SUPER_SIMPLE,				k_param_super_simple,					PSTR("SUPER_SIMPLE")),

	waypoint_mode			(0,							k_param_waypoint_mode,					PSTR("WP_MODE")),
	command_total			(0,							k_param_command_total,					PSTR("WP_TOTAL")),
	command_index			(0,							k_param_command_index,					PSTR("WP_INDEX")),
	command_nav_index		(0,							k_param_command_nav_index,				PSTR("WP_MUST_INDEX")),
	waypoint_radius			(WP_RADIUS_DEFAULT * 100,	k_param_waypoint_radius,				PSTR("WP_RADIUS")),
	loiter_radius			(LOITER_RADIUS,	    		k_param_loiter_radius,					PSTR("WP_LOITER_RAD")),
	waypoint_speed_max		(WAYPOINT_SPEED_MAX,		k_param_waypoint_speed_max,				PSTR("WP_SPEED_MAX")),
	crosstrack_gain			(CROSSTRACK_GAIN,			k_param_crosstrack_gain,				PSTR("XTRK_GAIN_SC")),
	auto_land_timeout		(AUTO_LAND_TIME * 1000,		k_param_auto_land_timeout,				PSTR("AUTO_LAND")),

	throttle_min			(0,							k_param_throttle_min,					PSTR("THR_MIN")),
	throttle_max			(1000, 						k_param_throttle_max,					PSTR("THR_MAX")),
	throttle_fs_enabled		(THROTTLE_FAILSAFE,			k_param_throttle_fs_enabled,			PSTR("THR_FAILSAFE")),
	throttle_fs_action		(THROTTLE_FAILSAFE_ACTION,	k_param_throttle_fs_action, 			PSTR("THR_FS_ACTION")),
	throttle_fs_value 		(THROTTLE_FS_VALUE,			k_param_throttle_fs_value, 				PSTR("THR_FS_VALUE")),
	throttle_cruise			(THROTTLE_CRUISE,			k_param_throttle_cruise,				PSTR("TRIM_THROTTLE")),

    flight_mode1            (FLIGHT_MODE_1,             k_param_flight_mode1,					PSTR("FLTMODE1")),
    flight_mode2            (FLIGHT_MODE_2,             k_param_flight_mode2,					PSTR("FLTMODE2")),
    flight_mode3            (FLIGHT_MODE_3,             k_param_flight_mode3,					PSTR("FLTMODE3")),
    flight_mode4            (FLIGHT_MODE_4,             k_param_flight_mode4,					PSTR("FLTMODE4")),
    flight_mode5            (FLIGHT_MODE_5,             k_param_flight_mode5,					PSTR("FLTMODE5")),
    flight_mode6            (FLIGHT_MODE_6,             k_param_flight_mode6,					PSTR("FLTMODE6")),
    simple_modes            (0,             			k_param_simple_modes,					PSTR("SIMPLE")),

	log_bitmask				(DEFAULT_LOG_BITMASK,		k_param_log_bitmask,					PSTR("LOG_BITMASK")),
    log_last_filenumber     (0,							k_param_log_last_filenumber,    		PSTR("LOG_LASTFILE")),
	esc_calibrate 			(0, 						k_param_esc_calibrate, 					PSTR("ESC")),
	radio_tuning 			(0, 						k_param_radio_tuning, 					PSTR("TUNE")),
	frame_orientation 		(FRAME_ORIENTATION, 		k_param_frame_orientation, 				PSTR("FRAME")),
	top_bottom_ratio 		(TOP_BOTTOM_RATIO, 			k_param_top_bottom_ratio, 				PSTR("TB_RATIO")),
	ch7_option 				(CH7_OPTION, 				k_param_ch7_option, 					PSTR("CH7_OPT")),

	#if FRAME_CONFIG ==	HELI_FRAME
	heli_servo_1			(k_param_heli_servo_1,		PSTR("HS1_")),
	heli_servo_2			(k_param_heli_servo_2,		PSTR("HS2_")),
	heli_servo_3			(k_param_heli_servo_3,		PSTR("HS3_")),
	heli_servo_4			(k_param_heli_servo_4,		PSTR("HS4_")),
	heli_servo1_pos			(-60,						k_param_heli_servo1_pos,				PSTR("SV1_POS_")),
	heli_servo2_pos			(60,						k_param_heli_servo2_pos,				PSTR("SV2_POS_")),
	heli_servo3_pos			(180,						k_param_heli_servo3_pos,				PSTR("SV3_POS_")),
	heli_roll_max			(4500,						k_param_heli_roll_max,					PSTR("ROL_MAX_")),
	heli_pitch_max			(4500,						k_param_heli_pitch_max,					PSTR("PIT_MAX_")),
	heli_coll_min			(1250,						k_param_heli_collective_min,			PSTR("COL_MIN_")),
	heli_coll_max			(1750,						k_param_heli_collective_max,			PSTR("COL_MAX_")),
	heli_coll_mid			(1500,						k_param_heli_collective_mid,			PSTR("COL_MID_")),
	heli_ext_gyro_enabled	(0,							k_param_heli_ext_gyro_enabled,			PSTR("GYR_ENABLE_")),
	heli_ext_gyro_gain		(1350,						k_param_heli_ext_gyro_gain,				PSTR("GYR_GAIN_")),
	heli_servo_averaging	(0,							k_param_heli_servo_averaging,			PSTR("SV_AVG")),
	heli_servo_manual		(0,							k_param_heli_servo_manual,				PSTR("HSV_MAN")),
	heli_phase_angle		(0,							k_param_heli_phase_angle,				PSTR("H_PHANG")),
	heli_coll_yaw_effect	(0,							k_param_heli_coll_yaw_effect,			PSTR("H_COLYAW")),
	#endif

	// RC channel			group key					name
	//----------------------------------------------------------------------
	rc_1					(k_param_rc_1,		PSTR("RC1_")),
	rc_2					(k_param_rc_2,		PSTR("RC2_")),
	rc_3					(k_param_rc_3,		PSTR("RC3_")),
	rc_4					(k_param_rc_4,		PSTR("RC4_")),
	rc_5					(k_param_rc_5,		PSTR("RC5_")),
	rc_6					(k_param_rc_6,		PSTR("RC6_")),
	rc_7					(k_param_rc_7,		PSTR("RC7_")),
	rc_8					(k_param_rc_8,		PSTR("RC8_")),
	rc_camera_pitch			(k_param_rc_9,		PSTR("CAM_P_")),
	rc_camera_roll			(k_param_rc_10,		PSTR("CAM_R_")),

	// variable				default						key										name
	//-------------------------------------------------------------------------------------------------------------------
	camera_pitch_gain 		(CAM_PITCH_GAIN, 			k_param_camera_pitch_gain, 				PSTR("CAM_P_G")),
	camera_roll_gain 		(CAM_ROLL_GAIN, 			k_param_camera_roll_gain,	 			PSTR("CAM_R_G")),
	stablize_d 				(STABILIZE_D, 				k_param_stabilize_d,	 				PSTR("STAB_D")),

	// PID controller	group key						name				initial P	   	    initial I		 	initial D       initial imax
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
	pid_rate_roll       (k_param_pid_rate_roll,         PSTR("RATE_RLL_"),  RATE_ROLL_P,        RATE_ROLL_I,        RATE_ROLL_D,    RATE_ROLL_IMAX * 100),
	pid_rate_pitch      (k_param_pid_rate_pitch,        PSTR("RATE_PIT_"),  RATE_PITCH_P,       RATE_PITCH_I,       RATE_PITCH_D,   RATE_PITCH_IMAX * 100),
	pid_rate_yaw        (k_param_pid_rate_yaw,          PSTR("RATE_YAW_"),  RATE_YAW_P,         RATE_YAW_I,         RATE_YAW_D,     RATE_YAW_IMAX * 100),

	pid_nav_lat			(k_param_pid_nav_lat,			PSTR("NAV_LAT_"),	NAV_P,				NAV_I,				NAV_D,			NAV_IMAX * 100),
	pid_nav_lon			(k_param_pid_nav_lon,			PSTR("NAV_LON_"),	NAV_P,				NAV_I,				NAV_D,			NAV_IMAX * 100),

	pid_throttle		(k_param_pid_throttle,			PSTR("THR_RATE_"),	THROTTLE_P,			THROTTLE_I,			THROTTLE_D,		THROTTLE_IMAX),
	pid_optflow_roll	(k_param_pid_optflow_roll,		PSTR("OF_RLL_"),	OPTFLOW_ROLL_P,		OPTFLOW_ROLL_I,		OPTFLOW_ROLL_D,	OPTFLOW_IMAX * 100),
	pid_optflow_pitch	(k_param_pid_optflow_pitch,		PSTR("OF_PIT_"),	OPTFLOW_PITCH_P,	OPTFLOW_PITCH_I,	OPTFLOW_PITCH_D,OPTFLOW_IMAX * 100),


	// PI controller	group key						name				initial P			initial I			initial imax
	//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
	pi_stabilize_roll	(k_param_pi_stabilize_roll,		PSTR("STB_RLL_"),	STABILIZE_ROLL_P,	STABILIZE_ROLL_I,	STABILIZE_ROLL_IMAX * 100),
	pi_stabilize_pitch	(k_param_pi_stabilize_pitch,	PSTR("STB_PIT_"),	STABILIZE_PITCH_P,	STABILIZE_PITCH_I,	STABILIZE_PITCH_IMAX * 100),
	pi_stabilize_yaw	(k_param_pi_stabilize_yaw,		PSTR("STB_YAW_"),	STABILIZE_YAW_P,	STABILIZE_YAW_I,	STABILIZE_YAW_IMAX * 100),

	pi_alt_hold			(k_param_pi_alt_hold,			PSTR("THR_ALT_"),	ALT_HOLD_P,			ALT_HOLD_I,			ALT_HOLD_IMAX),
	pi_loiter_lat		(k_param_pi_loiter_lat,			PSTR("HLD_LAT_"),	LOITER_P,			LOITER_I,			LOITER_IMAX * 100),
	pi_loiter_lon		(k_param_pi_loiter_lon,			PSTR("HLD_LON_"),	LOITER_P,			LOITER_I,			LOITER_IMAX * 100),


	junk(0)		// XXX just so that we can add things without worrying about the trailing comma
	{
	}
};

#endif // PARAMETERS_H

