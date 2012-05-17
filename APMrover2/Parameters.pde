/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  ArduPlane parameter definitions

  This firmware is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#define GSCALAR(v, name) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, class::var_info }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, class::var_info }

static const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION"),
	GSCALAR(software_type,          "SYSID_SW_TYPE"),
	GSCALAR(sysid_this_mav,         "SYSID_THISMAV"),
	GSCALAR(sysid_my_gcs,           "SYSID_MYGCS"),
	GSCALAR(serial3_baud,           "SERIAL3_BAUD"),
	GSCALAR(kff_pitch_compensation, "KFF_PTCHCOMP"),
	GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX"),
	GSCALAR(kff_pitch_to_throttle,  "KFF_PTCH2THR"),
	GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH"),
	GSCALAR(manual_level,           "MANUAL_LEVEL"),
	
	GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC"),
	GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD"),

	GSCALAR(altitude_mix,           "ALT_MIX"),
	GSCALAR(airspeed_ratio,         "ARSPD_RATIO"),
	GSCALAR(airspeed_offset,        "ARSPD_OFFSET"),

	GSCALAR(command_total,          "CMD_TOTAL"),
	GSCALAR(command_index,          "CMD_INDEX"),
	GSCALAR(waypoint_radius,        "WP_RADIUS"),
	GSCALAR(loiter_radius,          "WP_LOITER_RAD"),

#if GEOFENCE_ENABLED == ENABLED
	GSCALAR(fence_action,           "FENCE_ACTION"),
	GSCALAR(fence_total,            "FENCE_TOTAL"),
	GSCALAR(fence_channel,          "FENCE_CHANNEL"),
	GSCALAR(fence_minalt,           "FENCE_MINALT"),
	GSCALAR(fence_maxalt,           "FENCE_MAXALT"),
#endif

	GSCALAR(flybywire_airspeed_min, "ARSPD_FBW_MIN"),
	GSCALAR(flybywire_airspeed_max, "ARSPD_FBW_MAX"),

	GSCALAR(throttle_min,           "THR_MIN"),
	GSCALAR(throttle_max,           "THR_MAX"),
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE"),
	GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE"),
	GSCALAR(throttle_fs_value,      "THR_FS_VALUE"),
	GSCALAR(throttle_cruise,        "TRIM_THROTTLE"),

	GSCALAR(short_fs_action,        "FS_SHORT_ACTN"),
	GSCALAR(long_fs_action,         "FS_LONG_ACTN"),
	GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL"),

	GSCALAR(flight_mode_channel,    "FLTMODE_CH"),
	GSCALAR(flight_mode1,           "FLTMODE1"),
	GSCALAR(flight_mode2,           "FLTMODE2"),
	GSCALAR(flight_mode3,           "FLTMODE3"),
	GSCALAR(flight_mode4,           "FLTMODE4"),
	GSCALAR(flight_mode5,           "FLTMODE5"),
	GSCALAR(flight_mode6,           "FLTMODE6"),

	GSCALAR(roll_limit,             "LIM_ROLL_CD"),
	GSCALAR(pitch_limit_max,        "LIM_PITCH_MAX"),
	GSCALAR(pitch_limit_min,        "LIM_PITCH_MIN"),

	GSCALAR(auto_trim,              "TRIM_AUTO"),
	GSCALAR(switch_enable,          "SWITCH_ENABLE"),
	GSCALAR(mix_mode,               "ELEVON_MIXING"),
	GSCALAR(reverse_elevons,        "ELEVON_REVERSE"),
	GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV"),
	GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV"),
	GSCALAR(num_resets,             "SYS_NUM_RESETS"),
	GSCALAR(log_bitmask,            "LOG_BITMASK"),
	GSCALAR(log_last_filenumber,    "LOG_LASTFILE"),
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH"),
	GSCALAR(airspeed_cruise,        "TRIM_ARSPD_CM"),
	GSCALAR(min_gndspeed,           "MIN_GNDSPD_CM"),
	GSCALAR(ch7_option,             "CH7_OPT"),

	GSCALAR(pitch_trim,             "TRIM_PITCH_CD"),
	GSCALAR(RTL_altitude,           "ALT_HOLD_RTL"),
	GSCALAR(FBWB_min_altitude,      "ALT_HOLD_FBWCM"),

	GSCALAR(ground_temperature,     "GND_TEMP"),
	GSCALAR(ground_pressure,        "GND_ABS_PRESS"),
	GSCALAR(compass_enabled,        "MAG_ENABLE"),
	GSCALAR(flap_1_percent,         "FLAP_1_PERCNT"),
	GSCALAR(flap_1_speed,           "FLAP_1_SPEED"),
	GSCALAR(flap_2_percent,         "FLAP_2_PERCNT"),
	GSCALAR(flap_2_speed,           "FLAP_2_SPEED"),


	GSCALAR(battery_monitoring,     "BATT_MONITOR"),
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER"),
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT"),
	GSCALAR(input_voltage,          "INPUT_VOLTS"),
	GSCALAR(pack_capacity,          "BATT_CAPACITY"),
	GSCALAR(inverted_flight_ch,     "INVERTEDFLT_CH"),
#if HIL_MODE != HIL_MODE_ATTITUDE
#if LITE == DISABLED   
	// @Param: SONAR_ENABLE
	// @DisplayName: Enable Sonar
	// @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(sonar_enabled,	        "SONAR_ENABLE"),
	GSCALAR(sonar_type,	        "SONAR_TYPE"),
#endif	
#endif
	GSCALAR(airspeed_enabled,       "ARSPD_ENABLE"),

 // ************************************************************
        // APMrover parameters - JLN update
        
        GSCALAR(closed_loop_nav,        "ROV_CL_NAV"),
        GSCALAR(auto_wp_radius,         "ROV_AWPR_NAV"),
        GSCALAR(sonar_trigger,          "ROV_SONAR_TRIG"),
        GSCALAR(turn_gain,              "ROV_GAIN"),
        GSCALAR(booster,                 "ROV_BOOSTER"),
        
// ************************************************************

	GGROUP(channel_roll,            "RC1_", RC_Channel),
	GGROUP(channel_pitch,           "RC2_", RC_Channel),
	GGROUP(channel_throttle,        "RC3_", RC_Channel),
	GGROUP(channel_rudder,          "RC4_", RC_Channel),
	GGROUP(rc_5,                    "RC5_", RC_Channel_aux),
	GGROUP(rc_6,                    "RC6_", RC_Channel_aux),
	GGROUP(rc_7,                    "RC7_", RC_Channel_aux),
	GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

	GGROUP(pidNavRoll,              "HDNG2RLL_",  PID),
	GGROUP(pidServoRoll,            "RLL2SRV_",   PID),
	GGROUP(pidServoPitch,           "PTCH2SRV_",  PID),
	GGROUP(pidNavPitchAirspeed,     "ARSP2PTCH_", PID),
	GGROUP(pidServoRudder,          "YW2SRV_",    PID),
	GGROUP(pidTeThrottle,           "ENRGY2THR_", PID),
	GGROUP(pidNavPitchAltitude,     "ALT2PTCH_",  PID),

	// variables not in the g class which contain EEPROM saved variables
	GOBJECT(compass,                "COMPASS_",	Compass),
	GOBJECT(gcs0,					"SR0_",     GCS_MAVLINK),
	GOBJECT(gcs3,					"SR3_",     GCS_MAVLINK),
	GOBJECT(imu,					"IMU_",     IMU)
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
		Serial.println_P(PSTR("done."));
    } else {
	    unsigned long before = micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
	}
}
