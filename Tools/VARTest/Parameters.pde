/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  ArduPlane parameter definitions

  This firmware is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, { def_value:def } }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, { group_info: class::var_info } }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v,  { group_info: class::var_info } }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION", 0),
	GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),
	GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),
	GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),
	GSCALAR(serial3_baud,           "SERIAL3_BAUD",   SERIAL3_BAUD/1000),
	GSCALAR(kff_pitch_compensation, "KFF_PTCHCOMP",   PITCH_COMP),
	GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),
	GSCALAR(kff_pitch_to_throttle,  "KFF_PTCH2THR",   P_TO_T),
	GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   T_TO_P),

	GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC",   XTRACK_GAIN_SCALED),
	GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD",  XTRACK_ENTRY_ANGLE_CENTIDEGREE),

	GSCALAR(altitude_mix,           "ALT_MIX",        ALTITUDE_MIX),
	GSCALAR(airspeed_ratio,         "ARSPD_RATIO",    AIRSPEED_RATIO),
	GSCALAR(airspeed_offset,        "ARSPD_OFFSET",   0),

	GSCALAR(command_total,          "CMD_TOTAL",      0),
	GSCALAR(command_index,          "CMD_INDEX",      0),
	GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),
	GSCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

#if GEOFENCE_ENABLED == ENABLED
	GSCALAR(fence_action,           "FENCE_ACTION",   0),
	GSCALAR(fence_total,            "FENCE_TOTAL",    0),
	GSCALAR(fence_channel,          "FENCE_CHANNEL",  0),
	GSCALAR(fence_minalt,           "FENCE_MINALT",   0),
	GSCALAR(fence_maxalt,           "FENCE_MAXALT",   0),
#endif

	GSCALAR(flybywire_airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),
	GSCALAR(flybywire_airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

	GSCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),
	GSCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE",   THROTTLE_SLEW_LIMIT),
	GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   THROTTLE_FAILSAFE),
	GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   THROTTLE_FS_VALUE),
	GSCALAR(throttle_cruise,        "TRIM_THROTTLE",  THROTTLE_CRUISE),

	GSCALAR(short_fs_action,        "FS_SHORT_ACTN",  SHORT_FAILSAFE_ACTION),
	GSCALAR(long_fs_action,         "FS_LONG_ACTN",   LONG_FAILSAFE_ACTION),
	GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_HEARTBEAT_FAILSAFE),

	GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),
	GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),
	GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),
	GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),
	GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),
	GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),
	GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

	GSCALAR(roll_limit,             "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),
	GSCALAR(pitch_limit_max,        "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),
	GSCALAR(pitch_limit_min,        "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

	GSCALAR(auto_trim,              "TRIM_AUTO",      AUTO_TRIM),
	GSCALAR(switch_enable,          "SWITCH_ENABLE",  REVERSE_SWITCH),
	GSCALAR(mix_mode,               "ELEVON_MIXING",  ELEVON_MIXING),
	GSCALAR(reverse_elevons,        "ELEVON_REVERSE", ELEVON_REVERSE),
	GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV", ELEVON_CH1_REVERSE),
	GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV", ELEVON_CH2_REVERSE),
	GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),
	GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),
	GSCALAR(log_last_filenumber,    "LOG_LASTFILE",   0),
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",  0),
	GSCALAR(airspeed_cruise,        "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),
	GSCALAR(min_gndspeed,           "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),
	GSCALAR(pitch_trim,             "TRIM_PITCH_CD",  0),
	GSCALAR(RTL_altitude,           "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),
	GSCALAR(FBWB_min_altitude,      "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),
	GSCALAR(ground_temperature,     "GND_TEMP",       0),
	GSCALAR(ground_pressure,        "GND_ABS_PRESS",  0),
	GSCALAR(compass_enabled,        "MAG_ENABLE",     MAGNETOMETER),
	GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),
	GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),
	GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),
	GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),


	GSCALAR(battery_monitoring,     "BATT_MONITOR",   DISABLED),
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",   VOLT_DIV_RATIO),
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",   CURR_AMP_PER_VOLT),
	GSCALAR(input_voltage,          "INPUT_VOLTS",    INPUT_VOLTAGE),
	GSCALAR(pack_capacity,          "BATT_CAPACITY",  HIGH_DISCHARGE),
	GSCALAR(inverted_flight_ch,     "INVERTEDFLT_CH", 0),
	GSCALAR(sonar_enabled,          "SONAR_ENABLE",   SONAR_ENABLED),
	GSCALAR(airspeed_enabled,       "ARSPD_ENABLE",   AIRSPEED_SENSOR),
#if 1

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
#if 0
	// VARTest doesn't have these
	GOBJECT(gcs0,					"SR0_",     GCS_MAVLINK),
	GOBJECT(gcs3,					"SR3_",     GCS_MAVLINK),
#endif
#endif
	AP_VAREND
};


static void load_parameters(void)
{
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {

		// erase all parameters
		cliSerial->printf_P(PSTR("Firmware change (%u -> %u): erasing EEPROM...\n"),
						g.format_version.get(), Parameters::k_format_version);
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		cliSerial->println_P(PSTR("done."));
    } else {
	    unsigned long before = hal.scheduler->micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    cliSerial->printf_P(PSTR("load_all took %luus\n"), hal.scheduler->micros() - before);
	}
}
