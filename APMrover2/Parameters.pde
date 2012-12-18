/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  ArduPlane parameter definitions

  This firmware is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value:def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info:class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info:class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION",   0),
	GSCALAR(software_type,          "SYSID_SW_TYPE",    Parameters::k_software_type),
	GSCALAR(sysid_this_mav,         "SYSID_THISMAV",    MAV_SYSTEM_ID),
	GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",      255),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial3_baud,           "SERIAL3_BAUD",     SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: MANUAL_LEVEL
    // @DisplayName: Manual Level
    // @Description: Setting this to Disabled(0) will enable autolevel on every boot. Setting it to Enabled(1) will do a calibration only when you tell it to
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
	GSCALAR(manual_level,           "MANUAL_LEVEL",     0),
	
    // @Param: XTRK_GAIN_SC
    // @DisplayName: Crosstrack Gain
    // @Description: The scale between distance off the line and angle to meet the line (in Degrees * 100)
    // @Range: 0 2000
    // @Increment: 1
    // @User: Standard
	GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC",     XTRACK_GAIN_SCALED),

    // @Param: XTRK_ANGLE_CD
    // @DisplayName: Crosstrack Entry Angle
    // @Description: Maximum angle used to correct for track following.
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
	GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD",    XTRACK_ENTRY_ANGLE_CENTIDEGREE),

	GSCALAR(command_total,          "CMD_TOTAL",        0),
	GSCALAR(command_index,          "CMD_INDEX",        0),
	GSCALAR(waypoint_radius,        "WP_RADIUS",        WP_RADIUS_DEFAULT),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_min,           "THR_MIN",          THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_max,           "THR_MAX",          THROTTLE_MAX),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: maximum percentage change in throttle per second. A setting of 10 means to not change the throttle by more than 10% of the full throttle range in one second
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE",     THROTTLE_SLEW_LIMIT),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",     THROTTLE_FAILSAFE),

    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
	GSCALAR(throttle_fs_value,      "THR_FS_VALUE",     THROTTLE_FS_VALUE),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle cruise percentage
    // @Description: The target percentage of throttle to apply for normal flight
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_cruise,        "TRIM_THROTTLE",    THROTTLE_CRUISE),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: Short failsafe action
    // @Description: The action to take on a short (1 second) failsafe event
    // @Values: 0:None,1:ReturnToLaunch
    // @User: Standard
	GSCALAR(short_fs_action,        "FS_SHORT_ACTN",    SHORT_FAILSAFE_ACTION),

    // @Param: FS_LONG_ACTN
    // @DisplayName: Long failsafe action
    // @Description: The action to take on a long (20 second) failsafe event
    // @Values: 0:None,1:ReturnToLaunch
    // @User: Standard
	GSCALAR(long_fs_action,         "FS_LONG_ACTN",     LONG_FAILSAFE_ACTION),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after 20 seconds of no MAVLink heartbeat messages
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL",   GCS_HEARTBEAT_FAILSAFE),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @User: Advanced
	GSCALAR(flight_mode_channel,    "FLTMODE_CH",       FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: FlightMode1
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    // @Description: Flight mode for switch position 1 (910 to 1230 and above 2049)
	GSCALAR(flight_mode1,           "FLTMODE1",         FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: FlightMode2
    // @Description: Flight mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
	GSCALAR(flight_mode2,           "FLTMODE2",         FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: FlightMode3
    // @Description: Flight mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
	GSCALAR(flight_mode3,           "FLTMODE3",         FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: FlightMode4
    // @Description: Flight mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
	GSCALAR(flight_mode4,           "FLTMODE4",         FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: FlightMode5
    // @Description: Flight mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
	GSCALAR(flight_mode5,           "FLTMODE5",         FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: FlightMode6
    // @Description: Flight mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
	GSCALAR(flight_mode6,           "FLTMODE6",         FLIGHT_MODE_6),

	GSCALAR(roll_limit,             "LIM_ROLL_CD",      HEAD_MAX_CENTIDEGREE),
	GSCALAR(pitch_limit_max,        "LIM_PITCH_MAX",    PITCH_MAX_CENTIDEGREE),
	GSCALAR(pitch_limit_min,        "LIM_PITCH_MIN",    PITCH_MIN_CENTIDEGREE),

	GSCALAR(auto_trim,              "TRIM_AUTO",        AUTO_TRIM),
	GSCALAR(num_resets,             "SYS_NUM_RESETS",   0),
	GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),
	GSCALAR(log_last_filenumber,    "LOG_LASTFILE",     0),
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),
	GSCALAR(airspeed_cruise,        "TRIM_ARSPD_CM",    AIRSPEED_CRUISE_CM),
	GSCALAR(min_gndspeed,           "MIN_GNDSPD_CM",    MIN_GNDSPEED_CM),
	GSCALAR(ch7_option,             "CH7_OPT",          CH7_OPTION),

	GSCALAR(compass_enabled,        "MAG_ENABLE",       MAGNETOMETER),

	GSCALAR(battery_monitoring,     "BATT_MONITOR",     DISABLED),
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",     VOLT_DIV_RATIO),
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",     CURR_AMP_PER_VOLT),
	GSCALAR(input_voltage,          "INPUT_VOLTS",      INPUT_VOLTAGE),
	GSCALAR(pack_capacity,          "BATT_CAPACITY",    HIGH_DISCHARGE),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    1),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 12:A12
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    2),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

#if HIL_MODE != HIL_MODE_ATTITUDE
#if CONFIG_SONAR == ENABLED     
	// @Param: SONAR_ENABLE
	// @DisplayName: Enable Sonar
	// @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
	// @Values: 0:Disabled,1:Enabled
	// @User: Standard
	GSCALAR(sonar_enabled,	        "SONAR_ENABLE",     SONAR_ENABLED),
	GSCALAR(sonar_type,	        "SONAR_TYPE",           AP_RANGEFINDER_MAXSONARXL),
#endif	
#endif

 // ************************************************************
        // APMrover parameters - JLN update
	GSCALAR(auto_wp_radius,         "ROV_AWPR_NAV",     AUTO_WP_RADIUS),
	GSCALAR(sonar_trigger,          "ROV_SONAR_TRIG",   SONAR_TRIGGER),
	GSCALAR(turn_gain,              "ROV_GAIN",         TURN_GAIN),
	GSCALAR(booster,                 "ROV_BOOSTER",     BOOSTER),
        
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

#if HIL_MODE == HIL_MODE_DISABLED
    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                            "INS_", AP_InertialSensor),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

	AP_VAREND
};


static void load_parameters(void)
{
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {

		// erase all parameters
		cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		cliSerial->println_P(PSTR("done."));
    } else {
	    unsigned long before = micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
	}
}
