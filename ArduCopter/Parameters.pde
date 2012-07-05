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

    // @Param: SERIAL3_BAUD
	// @DisplayName: Telemetry Baud Rate
	// @Description: The baud rate used on the telemetry port
	// @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
	// @User: Standard
	GSCALAR(serial3_baud,	"SERIAL3_BAUD"),

	// @Param: ALT_HOLD_RTL
	// @DisplayName: Alt Hold RTL
	// @Description: This is the altitude the model will move to before Returning to Launch
	// @Units: Meters
	// @Range: 0 400
	// @Increment: 1
	// @User: Standard
	GSCALAR(RTL_altitude,	"ALT_HOLD_RTL"),

	// @Param: SONAR_ENABLE
	// @DisplayName: Enable Sonar
	// @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(sonar_enabled,	"SONAR_ENABLE"),

	GSCALAR(sonar_type,	"SONAR_TYPE"),
	GSCALAR(battery_monitoring, "BATT_MONITOR"),

	// @Param: VOLT_DIVIDER
	// @DisplayName: Voltage Divider
	// @Description: TODO
	GSCALAR(volt_div_ratio,	"VOLT_DIVIDER"),

	GSCALAR(curr_amp_per_volt,	"AMP_PER_VOLT"),
	GSCALAR(input_voltage,	"INPUT_VOLTS"),

	// @Param: BATT_CAPACITY
	// @DisplayName: Battery Capacity
	// @Description: Battery capacity in milliamp-hours (mAh)
	// @Units: mAh 
	GSCALAR(pack_capacity,	"BATT_CAPACITY"),

	// @Param: MAG_ENABLE
	// @DisplayName: Enable Compass
	// @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(compass_enabled,	"MAG_ENABLE"),

	// @Param: FLOW_ENABLE
	// @DisplayName: Enable Optical Flow
	// @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(optflow_enabled,	"FLOW_ENABLE"),

	// @Param: LOW_VOLT
	// @DisplayName: Low Voltage
	// @Description: Set this to the voltage you want to represent low voltage
	// @Range: 0 20
	// @Increment: .1
	// @User: Standard
	GSCALAR(low_voltage,	"LOW_VOLT"),

	// @Param: SUPER_SIMPLE
	// @DisplayName: Enable Super Simple Mode
	// @Description: Setting this to Enabled(1) will enable Super Simple Mode. Setting this to Disabled(0) will disable Super Simple Mode
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(super_simple,	"SUPER_SIMPLE"),

	// @Param: RTL_LAND
	// @DisplayName: RTL Land
	// @Description: Setting this to Enabled(1) will enable landing after RTL. Setting this to Disabled(0) will disable landing after RTL.
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	// @ DEPRICATED
	GSCALAR(rtl_land_enabled,	"RTL_LAND"),

	// @Param: APPROACH_ALT
	// @DisplayName: Alt Hold RTL
	// @Description: This is the altitude the vehicle will move to before Returning to Launch
	// @Units: Meters
	// @Range: 1 10
	// @Increment: .1
	// @User: Standard
	GSCALAR(rtl_approach_alt,	"APPROACH_ALT"),

	GSCALAR(tilt_comp,	"TILT"),

	GSCALAR(waypoint_mode,	"WP_MODE"),
	GSCALAR(command_total,	"WP_TOTAL"),
	GSCALAR(command_index,	"WP_INDEX"),
	GSCALAR(command_nav_index,	"WP_MUST_INDEX"),

    // @Param: WP_RADIUS
	// @DisplayName: Waypoint Radius
	// @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
	// @Units: Meters
	// @Range: 1 127
	// @Increment: 1
	// @User: Standard
	GSCALAR(waypoint_radius,	"WP_RADIUS"),

    // @Param: WP_LOITER_RAD
	// @DisplayName: Waypoint Loiter Radius
	// @Description: Defines the distance from the waypoint center, the vehicle will maintain during a loiter
	// @Units: Meters
	// @Range: 1 127
	// @Increment: 1
	// @User: Standard
	GSCALAR(loiter_radius,	"WP_LOITER_RAD"),
	GSCALAR(waypoint_speed_max,	"WP_SPEED_MAX"),
	GSCALAR(crosstrack_gain,	"XTRK_GAIN_SC"),
	GSCALAR(auto_land_timeout,	"AUTO_LAND"),

    // @Param: THR_MIN
	// @DisplayName: Minimum Throttle
	// @Description: The minimum throttle which the autopilot will apply.
	// @Units: Percent
	// @Range: 0 100
	// @Increment: 1
	// @User: Standard
	GSCALAR(throttle_min,	"THR_MIN"),

    // @Param: THR_MAX
	// @DisplayName: Maximum Throttle
	// @Description: The maximum throttle which the autopilot will apply.
	// @Units: Percent
	// @Range: 0 100
	// @Increment: 1
	// @User: Standard
	GSCALAR(throttle_max,	"THR_MAX"),

    // @Param: THR_FAILSAFE
	// @DisplayName: Throttle Failsafe Enable
	// @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(throttle_fs_enabled,	"THR_FAILSAFE"),

	GSCALAR(throttle_fs_action,	"THR_FS_ACTION"),

    // @Param: THR_FS_VALUE
	// @DisplayName: Throttle Failsafe Value
	// @Description: The PWM level on channel 3 below which throttle sailsafe triggers
	// @User: Standard
	GSCALAR(throttle_fs_value, "THR_FS_VALUE"),

	GSCALAR(throttle_cruise,	"TRIM_THROTTLE"),

	GSCALAR(flight_mode1, "FLTMODE1"),
	GSCALAR(flight_mode2, "FLTMODE2"),
	GSCALAR(flight_mode3, "FLTMODE3"),
	GSCALAR(flight_mode4, "FLTMODE4"),
	GSCALAR(flight_mode5, "FLTMODE5"),
	GSCALAR(flight_mode6, "FLTMODE6"),
	GSCALAR(simple_modes, "SIMPLE"),

    // @Param: LOG_BITMASK
	// @DisplayName: Log bitmask
	// @Description: bitmap of log fields to enable
	// @User: Advanced
	GSCALAR(log_bitmask,	"LOG_BITMASK"),
	GSCALAR(log_last_filenumber, "LOG_LASTFILE"),
	// THOR
	// Added to allow change of Rate in the Mission planner
	GSCALAR(toy_yaw_rate, "TOY_RATE"),

	GSCALAR(esc_calibrate, "ESC"),
	GSCALAR(radio_tuning, "TUNE"),
	GSCALAR(radio_tuning_low, "TUNE_LOW"),
	GSCALAR(radio_tuning_high, "TUNE_HIGH"),
	GSCALAR(frame_orientation, "FRAME"),
	GSCALAR(ch7_option, "CH7_OPT"),
	GSCALAR(auto_slew_rate, "AUTO_SLEW"),

	#if FRAME_CONFIG ==	HELI_FRAME
	GGROUP(heli_servo_1,	"HS1_", RC_Channel),
	GGROUP(heli_servo_2,	"HS2_", RC_Channel),
	GGROUP(heli_servo_3,	"HS3_", RC_Channel),
	GGROUP(heli_servo_4,	"HS4_", RC_Channel),
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

	// @Param: RC_SPEED
	// @DisplayName: ESC Update Speed
	// @Description: This is the speed in Hertz that your ESCs will receive updates
	// @Units: Hertz (Hz)
	// @Values: 125,400,490
	// @User: Advanced
	GSCALAR(rc_speed, "RC_SPEED"),

	// variable
	//---------
	GSCALAR(camera_pitch_gain,		"CAM_P_G"),
	GSCALAR(camera_roll_gain, 		"CAM_R_G"),
	GSCALAR(camera_pitch_continuous,"CAM_P_CONT"),
	GSCALAR(camera_roll_continuous,	"CAM_R_CONT"),
	GSCALAR(stabilize_d, 			"STAB_D"),

	// @Param: STAB_D_S
	// @DisplayName: Stabilize D Schedule
	// @Description: This value is a percentage of scheduling applied to the Stabilize D term.
	// @Range: 0 1
	// @Increment: .01
	// @User: Advanced
	GSCALAR(stabilize_d_schedule, "STAB_D_S"),

	GSCALAR(acro_p, 			"ACRO_P"),
	GSCALAR(axis_lock_p, 		"AXIS_P"),
	GSCALAR(axis_enabled, 		"AXIS_ENABLE"),
	GSCALAR(copter_leds_mode,	"LED_MODE"),

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

	// @Group: COMPASS_
	// @Path: ../libraries/AP_Compass/Compass.cpp
	GOBJECT(compass,        "COMPASS_", Compass),

	GOBJECT(gcs0,			"SR0_",     GCS_MAVLINK),
	GOBJECT(gcs3,			"SR3_",     GCS_MAVLINK),

	// @Group: IMU_
	// @Path: ../libraries/AP_IMU/IMU.cpp
	GOBJECT(imu,			"IMU_",     IMU),

	// @Group: AHRS_
	// @Path: ../libraries/AP_AHRS/AP_AHRS_DCM.cpp, ../libraries/AP_AHRS/AP_AHRS_Quaternion.cpp
	GOBJECT(ahrs,			"AHRS_",    AP_AHRS),

#ifdef DESKTOP_BUILD
	GOBJECT(sitl, "SIM_", SITL),
#endif

	#if FRAME_CONFIG ==	HELI_FRAME
	// @Group: H_
	// @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
	GOBJECT(motors,	"H_",		AP_MotorsHeli),
	#else
	GOBJECT(motors,	"MOT_",		AP_Motors),
	#endif
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
