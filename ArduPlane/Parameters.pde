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
    
    // @Param: SERIAL3_BAUD
	// @DisplayName: Telemetry Baud Rate
	// @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
	// @User: Standard
	GSCALAR(serial3_baud,           "SERIAL3_BAUD"),
        
    // @Param: KFF_PTCHCOMP
	// @DisplayName: Pitch Compensation
	// @Description: Adds pitch input to compensate for the loss of lift due to roll control.
	// @Range: 0 1
	// @Increment: 0.1
	// @User: Advanced
	GSCALAR(kff_pitch_compensation, "KFF_PTCHCOMP"),
    
    // @Param: KFF_RDDRMIX
	// @DisplayName: Rudder Mix
	// @Description: The ammount of rudder mix to apply during aileron movement
	// @Range: 0 1
	// @Increment: 0.1
	// @User: Standard
	GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX"),
        
    // @Param: KFF_PTCH2THR
	// @DisplayName: Pitch to Throttle Mix
	// @Description: Pitch to throttle feed-forward gain.
	// @Range: 0 5
	// @Increment: 0.1
	// @User: Advanced  
	GSCALAR(kff_pitch_to_throttle,  "KFF_PTCH2THR"),
        
    // @Param: KFF_THR2PTCH
	// @DisplayName: Throttle to Pitch Mix
	// @Description: Throttle to pitch feed-forward gain.
	// @Range: 0 5
	// @Increment: 0.1
	// @User: Advanced   
	GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH"),
    
    // @Param: MANUAL_LEVEL
	// @DisplayName: Manual Level
	// @Description: Setting this to Disabled(0) will enable autolevel on every boot. Setting it to Enabled(1) will do a calibration only when you tell it to
	// @Values: 0:Disabled,1:Enabled
	// @User: Advanced
	GSCALAR(manual_level,           "MANUAL_LEVEL"),

    // @Param: XTRK_GAIN_SC
	// @DisplayName: Crosstrack Gain
	// @Description: The scale between distance off the line and angle to meet the line (in Degrees * 100)
	// @Range: 0 2000
	// @Increment: 1
	// @User: Standard
	GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC"),
        
    // @Param: XTRK_ANGLE_CD
	// @DisplayName: Crosstrack Entry Angle
	// @Description: Maximum angle used to correct for track following.
	// @Units: Degrees
	// @Range: 0 90
	// @Increment: 1
	// @User: Standard 
	GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD"),

    // @Param: ALT_MIX
	// @DisplayName: Gps to Baro Mix
	// @Description: The percent of mixing between gps altitude and baro altitude. 0 = 100% gps, 1 = 100% baro
	// @Units: Percent
	// @Range: 0 1
	// @Increment: 0.1
	// @User: Advanced
	GSCALAR(altitude_mix,           "ALT_MIX"),
        
    // @Param: ARSPD_RATIO
	// @DisplayName: Airspeed Ratio
	// @Description: Used to scale raw adc airspeed sensor to a SI Unit (m/s)
	// @Units: Scale
	// @Range: 0 5
	// @Increment: 0.001
	// @User: Advanced   
	GSCALAR(airspeed_ratio,         "ARSPD_RATIO"),
        
	GSCALAR(airspeed_offset,        "ARSPD_OFFSET"),

	GSCALAR(command_total,          "CMD_TOTAL"),
	GSCALAR(command_index,          "CMD_INDEX"),
    
    // @Param: WP_RADIUS
	// @DisplayName: Waypoint Radius
	// @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
	// @Units: Meters
	// @Range: 1 127
	// @Increment: 1
	// @User: Standard 
	GSCALAR(waypoint_radius,        "WP_RADIUS"),
        
    // @Param: WP_LOITER_RAD
	// @DisplayName: Waypoint Loiter Radius
	// @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter
	// @Units: Meters
	// @Range: 1 127
	// @Increment: 1
	// @User: Standard   
	GSCALAR(loiter_radius,          "WP_LOITER_RAD"),

#if GEOFENCE_ENABLED == ENABLED
	GSCALAR(fence_action,           "FENCE_ACTION"),
	GSCALAR(fence_total,            "FENCE_TOTAL"),
	GSCALAR(fence_channel,          "FENCE_CHANNEL"),
	GSCALAR(fence_minalt,           "FENCE_MINALT"),
	GSCALAR(fence_maxalt,           "FENCE_MAXALT"),
#endif
        
    // @Param: ARSPD_FBW_MIN
	// @DisplayName: Fly By Wire Minimum Airspeed
	// @Description: Airspeed corresponding to minimum throttle in Fly By Wire B mode.
	// @Units: m/s
	// @Range: 5 50
	// @Increment: 1
	// @User: Standard   
	GSCALAR(flybywire_airspeed_min, "ARSPD_FBW_MIN"),
        
    // @Param: ARSPD_FBW_MAX
	// @DisplayName: Fly By Wire Maximum Airspeed
	// @Description: Airspeed corresponding to maximum throttle in Fly By Wire B mode.
	// @Units: m/s
	// @Range: 5 50
	// @Increment: 1
	// @User: Standard
	GSCALAR(flybywire_airspeed_max, "ARSPD_FBW_MAX"),

    // @Param: THR_MIN
	// @DisplayName: Minimum Throttle
	// @Description: The minimum throttle setting to which the autopilot will apply.
	// @Units: Percent
	// @Range: 0 100
	// @Increment: 1
	// @User: Standard
	GSCALAR(throttle_min,           "THR_MIN"),
        
    // @Param: THR_MAX
	// @DisplayName: Maximum Throttle
	// @Description: The maximum throttle setting to which the autopilot will apply.
	// @Units: Percent
	// @Range: 0 100
	// @Increment: 1
	// @User: Standard
	GSCALAR(throttle_max,           "THR_MAX"),
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE"),
    
    // @Param: THR_FAILSAFE
	// @DisplayName: Throttle Failsafe Enable
	// @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
	// @Units: Percent
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
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

    // @Param: LIM_ROLL_CD
	// @DisplayName: Maximum Bank Angle
	// @Description: The maximum commanded bank angle in either direction
	// @Units: Degrees
	// @Range: 0 90
	// @Increment: 1
	// @User: Standard
	GSCALAR(roll_limit,             "LIM_ROLL_CD"),
        
    // @Param: LIM_PITCH_MAX
	// @DisplayName: Maximum Pitch Angle
	// @Description: The maximum commanded pitch up angle
	// @Units: Degrees
	// @Range: 0 90
	// @Increment: 1
	// @User: Standard
	GSCALAR(pitch_limit_max,        "LIM_PITCH_MAX"),
        
    // @Param: LIM_PITCH_MIN
	// @DisplayName: Minimum Pitch Angle
	// @Description: The minimum commanded pitch down angle
	// @Units: Degrees
	// @Range: 0 90
	// @Increment: 1
	// @User: Standard
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
	GSCALAR(pitch_trim,             "TRIM_PITCH_CD"),
	GSCALAR(RTL_altitude,           "ALT_HOLD_RTL"),
	GSCALAR(FBWB_min_altitude,      "ALT_HOLD_FBWCM"),
	GSCALAR(ground_temperature,     "GND_TEMP"),
	GSCALAR(ground_pressure,        "GND_ABS_PRESS"),
    
    // @Param: MAG_ENABLE
	// @DisplayName: Enable Compass
	// @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
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
    
    // @Param: SONAR_ENABLE
	// @DisplayName: Enable Sonar
	// @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(sonar_enabled,          "SONAR_ENABLE"),
        
    // @Param: ARSPD_ENABLE
	// @DisplayName: Enable Airspeed
	// @Description: Setting this to Enabled(1) will enable the Airspeed sensor. Setting this to Disabled(0) will disable the Airspeed sensor
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(airspeed_enabled,       "ARSPD_ENABLE"),

    // @Param: ARSPD_USE
	// @DisplayName: Use Airspeed if enabled
	// @Description: Setting this to Enabled(1) will enable use of the Airspeed sensor for flight control when ARSPD_ENABLE is also true. This is separate from ARSPD_ENABLE to allow for the airspeed value to be logged without it being used for flight control
	// @Values: 0:Disabled,1:Enabled
	// @User: Advanced
	GSCALAR(airspeed_use,       	"ARSPD_USE"),

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
    
	// @Group: COMPASS_
	// @Path: ../libraries/AP_Compass/Compass.cpp
	GOBJECT(compass,                "COMPASS_",	Compass),
	GOBJECT(gcs0,					"SR0_",     GCS_MAVLINK),
	GOBJECT(gcs3,					"SR3_",     GCS_MAVLINK),
    
	// @Group: IMU_
	// @Path: ../libraries/AP_IMU/IMU.cpp
	GOBJECT(imu,					"IMU_",     IMU),
    
    // @Group: AHRS_
	// @Path: ../libraries/AP_AHRS/AP_AHRS_DCM.cpp, ../libraries/AP_AHRS/AP_AHRS_Quaternion.cpp
	GOBJECT(ahrs,					"AHRS_",    AP_AHRS)
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
