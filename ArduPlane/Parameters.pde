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
	// @Description: Adds pitch input to compensate for the loss of lift due to roll control. 0 = 0 %, 1 = 100%
	// @Range: 0 1
	// @Increment: 0.01
	// @User: Advanced
	GSCALAR(kff_pitch_compensation, "KFF_PTCHCOMP"),

    // @Param: KFF_RDDRMIX
	// @DisplayName: Rudder Mix
	// @Description: The amount of rudder mix to apply during aileron movement 0 = 0 %, 1 = 100%
	// @Range: 0 1
	// @Increment: 0.01
	// @User: Standard
	GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX"),

    // @Param: KFF_PTCH2THR
	// @DisplayName: Pitch to Throttle Mix
	// @Description: Pitch to throttle feed-forward gain.
	// @Range: 0 5
	// @Increment: 0.01
	// @User: Advanced
	GSCALAR(kff_pitch_to_throttle,  "KFF_PTCH2THR"),

    // @Param: KFF_THR2PTCH
	// @DisplayName: Throttle to Pitch Mix
	// @Description: Throttle to pitch feed-forward gain.
	// @Range: 0 5
	// @Increment: 0.01
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
	// @Units: centi-Degrees
	// @Range: 0 9000
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
    // @Param: FENCE_ACTION
	// @DisplayName: Action on geofence breach
	// @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter
    // @Values: 0:None,1:GuidedMode
	// @User: Standard
	GSCALAR(fence_action,           "FENCE_ACTION"),

    // @Param: FENCE_TOTAL
	// @DisplayName: Fence Total
	// @Description: Number of geofence points currently loaded
	// @User: Standard
	GSCALAR(fence_total,            "FENCE_TOTAL"),

    // @Param: FENCE_CHANNEL
	// @DisplayName: Fence Channel
	// @Description: RC Channel to use to enable geofence. PWM input above 1750 enables the geofence
	// @User: Standard
	GSCALAR(fence_channel,          "FENCE_CHANNEL"),

    // @Param: FENCE_MINALT
	// @DisplayName: Fence Minimum Altitude
	// @Description: Minimum altitude allowed before geofence triggers
	// @Units: meters
	// @Range: 0 32767
	// @Increment: 1
	// @User: Standard
	GSCALAR(fence_minalt,           "FENCE_MINALT"),

    // @Param: FENCE_MAXALT
	// @DisplayName: Fence Maximum Altitude
	// @Description: Maximum altitude allowed before geofence triggers
	// @Units: meters
	// @Range: 0 32767
	// @Increment: 1
	// @User: Standard
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

	// @Param: THR_SLEWRATE
	// @DisplayName: Throttlw slew rate
	// @Description: maximum percentage change in throttle per second
	// @Units: Percent
	// @Range: 0 100
	// @Increment: 1
	// @User: Standard
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE"),

    // @Param: THR_FAILSAFE
	// @DisplayName: Throttle Failsafe Enable
	// @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE"),


    // @Param: THR_FS_VALUE
	// @DisplayName: Throttle Failsafe Value
	// @Description: The PWM level on channel 3 below which throttle sailsafe triggers
	// @User: Standard
	GSCALAR(throttle_fs_value,      "THR_FS_VALUE"),

    // @Param: TRIM_THROTTLE
	// @DisplayName: Throttle cruise percentage
	// @Description: The target percentage of throttle to apply for normal flight
	// @Units: Percent
	// @Range: 0 100
	// @Increment: 1
	// @User: Standard
	GSCALAR(throttle_cruise,        "TRIM_THROTTLE"),

    // @Param: FS_SHORT_ACTN
	// @DisplayName: Short failsafe action
	// @Description: The action to take on a short (1 second) failsafe event
	// @Values: 0:None,1:ReturnToLaunch
	// @User: Standard
	GSCALAR(short_fs_action,        "FS_SHORT_ACTN"),

    // @Param: FS_LONG_ACTN
	// @DisplayName: Long failsafe action
	// @Description: The action to take on a long (20 second) failsafe event
	// @Values: 0:None,1:ReturnToLaunch
	// @User: Standard
	GSCALAR(long_fs_action,         "FS_LONG_ACTN"),

    // @Param: FS_GCS_ENABL
	// @DisplayName: GCS failsafe enable
	// @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after 20 seconds of no MAVLink heartbeat messages
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL"),

    // @Param: FLTMODE_CH
	// @DisplayName: Flightmode channel
	// @Description: RC Channel to use for flight mode control
	// @User: Advanced
	GSCALAR(flight_mode_channel,    "FLTMODE_CH"),

    // @Param: FLTMODE1
	// @DisplayName: FlightMode1
	// @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
	// @User: Standard
	// @Description: Flight mode for switch position 2 (910 to 1230 and above 2049)
	GSCALAR(flight_mode1,           "FLTMODE1"),

    // @Param: FLTMODE3
	// @DisplayName: FlightMode1
	// @Description: Flight mode for switch position 2 (1231 to 1360)
	// @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
	// @User: Standard
	GSCALAR(flight_mode2,           "FLTMODE2"),

    // @Param: FLTMODE3
	// @DisplayName: FlightMode1
	// @Description: Flight mode for switch position 3 (1361 to 1490)
	// @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
	// @User: Standard
	GSCALAR(flight_mode3,           "FLTMODE3"),

    // @Param: FLTMODE4
	// @DisplayName: FlightMode4
	// @Description: Flight mode for switch position 4 (1491 to 1620)
	// @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
	// @User: Standard
	GSCALAR(flight_mode4,           "FLTMODE4"),

    // @Param: FLTMODE5
	// @DisplayName: FlightMode5
	// @Description: Flight mode for switch position 5 (1621 to 1749)
	// @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
	// @User: Standard
	GSCALAR(flight_mode5,           "FLTMODE5"),

    // @Param: FLTMODE6
	// @DisplayName: FlightMode6
	// @Description: Flight mode for switch position 6 (1750 to 2049)
	// @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
	// @User: Standard
	GSCALAR(flight_mode6,           "FLTMODE6"),

    // @Param: LIM_ROLL_CD
	// @DisplayName: Maximum Bank Angle
	// @Description: The maximum commanded bank angle in either direction
	// @Units: centi-Degrees
	// @Range: 0 9000
	// @Increment: 1
	// @User: Standard
	GSCALAR(roll_limit,             "LIM_ROLL_CD"),

    // @Param: LIM_PITCH_MAX
	// @DisplayName: Maximum Pitch Angle
	// @Description: The maximum commanded pitch up angle
	// @Units: centi-Degrees
	// @Range: 0 9000
	// @Increment: 1
	// @User: Standard
	GSCALAR(pitch_limit_max,        "LIM_PITCH_MAX"),

    // @Param: LIM_PITCH_MIN
	// @DisplayName: Minimum Pitch Angle
	// @Description: The minimum commanded pitch down angle
	// @Units: centi-Degrees
	// @Range: 0 9000
	// @Increment: 1
	// @User: Standard
	GSCALAR(pitch_limit_min,        "LIM_PITCH_MIN"),

    // @Param: AUTO_TRIM
	// @DisplayName: Auto trim
	// @Description: Set RC trim PWM levels to current levels when switching away from manual mode
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(auto_trim,              "TRIM_AUTO"),

    // @Param: SWITCH_ENABLE
	// @DisplayName: Switch enable
	// @Description: Enable dip switches on APM1
	// @Values: 0:Disabled,1:Enabled
	// @User: Advanced
	GSCALAR(switch_enable,          "SWITCH_ENABLE"),

    // @Param: MIX_MODE
	// @DisplayName: Elevon mixing
	// @Description: Enable elevon mixing
	// @Values: 0:Disabled,1:Enabled
	// @User: User
	GSCALAR(mix_mode,               "ELEVON_MIXING"),

    // @Param: ELEVON_REVERSE
	// @DisplayName: Elevon reverse
	// @Description: Reverse elevon mixing
	// @Values: 0:Disabled,1:Enabled
	// @User: User
	GSCALAR(reverse_elevons,        "ELEVON_REVERSE"),


    // @Param: ELEVON_REVERSE
	// @DisplayName: Elevon reverse
	// @Description: Reverse elevon channel 1
	// @Values: 0:Disabled,1:Enabled
	// @User: User
	GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV"),

    // @Param: ELEVON_REVERSE
	// @DisplayName: Elevon reverse
	// @Description: Reverse elevon channel 2
	// @Values: 0:Disabled,1:Enabled
	// @User: User
	GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV"),

    // @Param: SYS_NUM_RESETS
	// @DisplayName: Num Resets
	// @Description: Number of APM board resets
	// @User: Advanced
	GSCALAR(num_resets,             "SYS_NUM_RESETS"),

    // @Param: LOG_BITMASK
	// @DisplayName: Log bitmask
	// @Description: bitmap of log fields to enable
	// @User: Advanced
	GSCALAR(log_bitmask,            "LOG_BITMASK"),

	GSCALAR(log_last_filenumber,    "LOG_LASTFILE"),

    // @Param: RST_SWITCH_CH
	// @DisplayName: RC channel to use to reset to last flight mode after geofence takeover
	// @User: Advanced
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH"),

    // @Param: TRIM_ARSPD_CM
	// @DisplayName: Airspeed in cm/s to aim for when airspeed is enabled in auto mode
	// @Units: cm/s
	// @User: User
	GSCALAR(airspeed_cruise,        "TRIM_ARSPD_CM"),

    // @Param: MIN_GNDSPD_CM
	// @DisplayName: Minimum ground speed in cm/s when under airspeed control
	// @Units: cm/s
	// @User: Advanced
	GSCALAR(min_gndspeed,           "MIN_GNDSPD_CM"),

    // @Param: TRIM_PITCH_CD
	// @DisplayName: Pitch angle offset 
	// @Units: centi-Degrees
	// @User: Advanced
	GSCALAR(pitch_trim,             "TRIM_PITCH_CD"),

    // @Param: ALT_HOLD_RTL
	// @DisplayName: Return to launch target altitude
	// @Units: centimeters
	// @User: User
	GSCALAR(RTL_altitude,           "ALT_HOLD_RTL"),

	GSCALAR(FBWB_min_altitude,      "ALT_HOLD_FBWCM"),

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

	// barometer ground calibration. The GND_ prefix is chosen for
	// compatibility with previous releases of ArduPlane
	GOBJECT(barometer, "GND_", AP_Baro),

#if CAMERA == ENABLED
	// @Group: CAM_
	// @Path: ../libraries/AP_Camera/AP_Camera.cpp
	GGROUP(camera,                  "CAM_",	AP_Camera),
#endif

	// RC channel
	//-----------
	GGROUP(channel_roll,            "RC1_", RC_Channel),
	GGROUP(channel_pitch,           "RC2_", RC_Channel),
	GGROUP(channel_throttle,        "RC3_", RC_Channel),
	GGROUP(channel_rudder,          "RC4_", RC_Channel),
	// @Group: RC5_
	// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_5,                    "RC5_", RC_Channel_aux),
	// @Group: RC6_
	// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_6,                    "RC6_", RC_Channel_aux),
	// @Group: RC7_
	// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
	GGROUP(rc_7,                    "RC7_", RC_Channel_aux),
	// @Group: RC8_
	// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
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
	GOBJECT(ahrs,					"AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
	// @Group: MNT_
	// @Path: ../libraries/AP_Mount/AP_Mount.cpp
	GOBJECT(camera_mount,           "MNT_",	AP_Mount),
#endif


#ifdef DESKTOP_BUILD
	// @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
	GOBJECT(sitl, "SIM_", SITL),
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
		Serial.println_P(PSTR("done."));
    } else {
	    unsigned long before = micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
	}
}
