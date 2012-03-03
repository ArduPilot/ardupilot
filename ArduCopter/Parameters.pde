/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  ArduCopter parameter definitions

  This firmware is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#define GSCALAR(v, name) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, class::var_info }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, class::var_info }

static const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version, "SYSID_SW_MREV"),
	GSCALAR(software_type,  "SYSID_SW_TYPE"),

	GSCALAR(sysid_this_mav,	"SYSID_THISMAV"),
	GSCALAR(sysid_my_gcs,	"SYSID_MYGCS"),
	GSCALAR(serial3_baud,	"SERIAL3_BAUD"),
	GSCALAR(RTL_altitude,	"ALT_HOLD_RTL"),
	GSCALAR(sonar_enabled,	"SONAR_ENABLE"),
	GSCALAR(sonar_type,	"SONAR_TYPE"),
	GSCALAR(battery_monitoring, "BATT_MONITOR"),
	GSCALAR(volt_div_ratio,	"VOLT_DIVIDER"),
	GSCALAR(curr_amp_per_volt,	"AMP_PER_VOLT"),
	GSCALAR(input_voltage,	"INPUT_VOLTS"),
	GSCALAR(pack_capacity,	"BATT_CAPACITY"),
	GSCALAR(compass_enabled,	"MAG_ENABLE"),
	GSCALAR(optflow_enabled,	"FLOW_ENABLE"),
	GSCALAR(low_voltage,	"LOW_VOLT"),
	GSCALAR(super_simple,	"SUPER_SIMPLE"),
	GSCALAR(rtl_land_enabled,	"RTL_LAND"),


	GSCALAR(waypoint_mode,	"WP_MODE"),
	GSCALAR(command_total,	"WP_TOTAL"),
	GSCALAR(command_index,	"WP_INDEX"),
	GSCALAR(command_nav_index,	"WP_MUST_INDEX"),
	GSCALAR(waypoint_radius,	"WP_RADIUS"),
	GSCALAR(loiter_radius,	"WP_LOITER_RAD"),
	GSCALAR(waypoint_speed_max,	"WP_SPEED_MAX"),
	GSCALAR(crosstrack_gain,	"XTRK_GAIN_SC"),
	GSCALAR(auto_land_timeout,	"AUTO_LAND"),

	GSCALAR(throttle_min,	"THR_MIN"),
	GSCALAR(throttle_max,	"THR_MAX"),
	GSCALAR(throttle_fs_enabled,	"THR_FAILSAFE"),
	GSCALAR(throttle_fs_action,	"THR_FS_ACTION"),
	GSCALAR(throttle_fs_value, "THR_FS_VALUE"),
	GSCALAR(throttle_cruise,	"TRIM_THROTTLE"),

	GSCALAR(flight_mode1, "FLTMODE1"),
	GSCALAR(flight_mode2, "FLTMODE2"),
	GSCALAR(flight_mode3, "FLTMODE3"),
	GSCALAR(flight_mode4, "FLTMODE4"),
	GSCALAR(flight_mode5, "FLTMODE5"),
	GSCALAR(flight_mode6, "FLTMODE6"),
	GSCALAR(simple_modes, "SIMPLE"),

	GSCALAR(log_bitmask,	"LOG_BITMASK"),
	GSCALAR(log_last_filenumber, "LOG_LASTFILE"),
	GSCALAR(esc_calibrate, "ESC"),
	GSCALAR(radio_tuning, "TUNE"),
	GSCALAR(radio_tuning_low, "TUNE_LOW"),
	GSCALAR(radio_tuning_high, "TUNE_HIGH"),
	GSCALAR(frame_orientation, "FRAME"),
	GSCALAR(top_bottom_ratio, "TB_RATIO"),
	GSCALAR(ch7_option, "CH7_OPT"),
	GSCALAR(auto_slew_rate, "AUTO_SLEW"),

	#if FRAME_CONFIG ==	HELI_FRAME
	GGROUP(heli_servo_1,	"HS1_", RC_Channel),
	GGROUP(heli_servo_2,	"HS2_", RC_Channel),
	GGROUP(heli_servo_3,	"HS3_", RC_Channel),
	GGROUP(heli_servo_4,	"HS4_", RC_Channel),
	GSCALAR(heli_servo1_pos,	"SV1_POS"),
	GSCALAR(heli_servo2_pos,	"SV2_POS"),
	GSCALAR(heli_servo3_pos,	"SV3_POS"),
	GSCALAR(heli_roll_max,	"ROL_MAX"),
	GSCALAR(heli_pitch_max,	"PIT_MAX"),
	GSCALAR(heli_collective_min,	"COL_MIN"),
	GSCALAR(heli_collective_max,	"COL_MAX"),
	GSCALAR(heli_collective_mid,	"COL_MID"),
	GSCALAR(heli_ext_gyro_enabled,	"GYR_ENABLE"),
	GSCALAR(heli_h1_swash_enabled,	"H1_ENABLE"),
	GSCALAR(heli_ext_gyro_gain,	"GYR_GAIN"),
	GSCALAR(heli_servo_averaging,	"SV_AVG"),
	GSCALAR(heli_servo_manual,	"HSV_MAN"),
	GSCALAR(heli_phase_angle,	"H_PHANG"),
	GSCALAR(heli_collective_yaw_effect,	"H_COLYAW"),
	#endif

	// RC channel
	//-----------
	GGROUP(rc_1,	"RC1_", RC_Channel),
	GGROUP(rc_2,	"RC2_", RC_Channel),
	GGROUP(rc_3,	"RC3_", RC_Channel),
	GGROUP(rc_4,	"RC4_", RC_Channel),
	GGROUP(rc_5,	"RC5_", RC_Channel),
	GGROUP(rc_6,	"RC6_", RC_Channel),
	GGROUP(rc_7,	"RC7_", RC_Channel),
	GGROUP(rc_8,	"RC8_", RC_Channel),
	GGROUP(rc_camera_pitch,	"CAM_P_", RC_Channel),
	GGROUP(rc_camera_roll,	"CAM_R_", RC_Channel),

	// speed of fast RC channels in Hz
	GSCALAR(rc_speed, "RC_SPEED"),

	// variable
	//---------
	GSCALAR(camera_pitch_gain, 	"CAM_P_G"),
	GSCALAR(camera_roll_gain, 	"CAM_R_G"),
	GSCALAR(stabilize_d, 		"STAB_D"),
	GSCALAR(stabilize_d_schedule, "STAB_D_S"),
	GSCALAR(acro_p, 			"ACRO_P"),
	GSCALAR(axis_lock_p, 		"AXIS_P"),
	GSCALAR(axis_enabled, 		"AXIS_ENABLE"),

	// PID controller
	//---------------
	GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),
	GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),
	GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),


	GGROUP(pid_loiter_rate_lat,	 "LOITER_LAT_",  AC_PID),
	GGROUP(pid_loiter_rate_lon,	 "LOITER_LON_",  AC_PID),

	GGROUP(pid_nav_lat,	  	"NAV_LAT_",  AC_PID),
	GGROUP(pid_nav_lon,	  	"NAV_LON_",  AC_PID),

	GGROUP(pid_throttle,	  "THR_RATE_", AC_PID),
	GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),
	GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

	// PI controller
	//--------------
	GGROUP(pi_stabilize_roll,	"STB_RLL_", APM_PI),
	GGROUP(pi_stabilize_pitch,	"STB_PIT_", APM_PI),
	GGROUP(pi_stabilize_yaw,	"STB_YAW_", APM_PI),

	GGROUP(pi_alt_hold,		"THR_ALT_", APM_PI),
	GGROUP(pi_loiter_lat,	"HLD_LAT_", APM_PI),
	GGROUP(pi_loiter_lon,	"HLD_LON_", APM_PI),

	// variables not in the g class which contain EEPROM saved variables
	GOBJECT(compass,        "COMPASS_", Compass),
	GOBJECT(gcs0,			"SR0_",     GCS_MAVLINK),
	GOBJECT(gcs3,			"SR3_",     GCS_MAVLINK),
	GOBJECT(imu,			"IMU_",     IMU)
};


static void load_parameters(void)
{
	// setup the AP_Var subsystem for storage to EEPROM
	if (!AP_Param::setup(var_info, sizeof(var_info)/sizeof(var_info[0]), WP_START_BYTE)) {
		// this can only happen on startup, and its a definate coding
		// error. Best not to continue so the programmer catches it
		while (1) {
			Serial.println_P(PSTR("ERROR: Failed to setup AP_Param"));
			delay(1000);
		}
	}

	if (!g.format_version.load() ||
	    g.format_version != Parameters::k_format_version) {

		// erase all parameters
		Serial.printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		default_dead_zones();
		Serial.println_P(PSTR("done."));
	} else {
		unsigned long before = micros();
		// Load all auto-loaded EEPROM variables
		AP_Param::load_all();

		Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
	}
}
