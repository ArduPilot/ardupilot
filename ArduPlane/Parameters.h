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
    static const uint16_t k_format_version = 12;

	// The parameter software_type is set up solely for ground station use
	// and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
	// GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
	// values within that range to identify different branches.
	//
    static const uint16_t k_software_type = 0;		// 0 for APM trunk

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
		k_param_software_type,


        // Misc
        //

        k_param_auto_trim,
        k_param_switch_enable,
        k_param_log_bitmask,
        k_param_pitch_trim,
        k_param_mix_mode,
        k_param_reverse_elevons,
        k_param_reverse_ch1_elevon,
        k_param_reverse_ch2_elevon,
        k_param_flap_1_percent,
        k_param_flap_1_speed,
        k_param_flap_2_percent,
        k_param_flap_2_speed,
        k_param_num_resets,


		// 110: Telemetry control
		//
		k_param_streamrates_port0 = 110,
		k_param_streamrates_port3,
		k_param_sysid_this_mav,
		k_param_sysid_my_gcs,
        k_param_serial3_baud,

        // 120: Fly-by-wire control
        //
        k_param_flybywire_airspeed_min = 120,
        k_param_flybywire_airspeed_max,
        k_param_FBWB_min_altitude,  // -1=disabled, minimum value for altitude in cm (default 30m)

        //
        // 130: Sensor parameters
        //
        k_param_IMU_calibration = 130,
        k_param_altitude_mix,
        k_param_airspeed_ratio,
        k_param_ground_temperature,
        k_param_ground_pressure,
		k_param_compass_enabled,
		k_param_compass,
		k_param_battery_monitoring,
		k_param_volt_div_ratio,
		k_param_curr_amp_per_volt,
		k_param_input_voltage,
		k_param_pack_capacity,
        k_param_airspeed_offset,
		k_param_sonar_enabled,
		k_param_airspeed_enabled,

        //
        // 150: Navigation parameters
        //
        k_param_crosstrack_gain = 150,
        k_param_crosstrack_entry_angle,
        k_param_roll_limit,
        k_param_pitch_limit_max,
        k_param_pitch_limit_min,
        k_param_airspeed_cruise,
        k_param_RTL_altitude,
        k_param_inverted_flight_ch,

        //
        // 170: Radio settings
        //
        k_param_channel_roll = 170,
        k_param_channel_pitch,
        k_param_channel_throttle,
        k_param_channel_yaw,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_short_fs_action,
        k_param_long_fs_action,
		k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,

        //
        // 200: Feed-forward gains
        //
        k_param_kff_pitch_compensation = 200,
        k_param_kff_rudder_mix,
        k_param_kff_pitch_to_throttle,
        k_param_kff_throttle_to_pitch,

        //
        // 210: flight modes
        //
        k_param_flight_mode_channel = 210,
        k_param_flight_mode1,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,

        //
        // 220: Waypoint data
        //
        k_param_waypoint_mode = 220,
        k_param_command_total,
        k_param_command_index,
        k_param_waypoint_radius,
        k_param_loiter_radius,

        //
        // 240: PID Controllers
        //
        // Heading-to-roll PID:
        // heading error from command to roll command deviation from trim
        // (bank to turn strategy)
        //
        k_param_heading_to_roll_PID = 240,

        // Roll-to-servo PID:
        // roll error from command to roll servo deviation from trim
        // (tracks commanded bank angle)
        //
        k_param_roll_to_servo_PID,

        //
        // Pitch control
        //
        // Pitch-to-servo PID:
        // pitch error from command to pitch servo deviation from trim
        // (front-side strategy)
        //
        k_param_pitch_to_servo_PID,

        // Airspeed-to-pitch PID:
        // airspeed error from command to pitch servo deviation from trim
        // (back-side strategy)
        //
        k_param_airspeed_to_pitch_PID,

        //
        // Yaw control
        //
        // Yaw-to-servo PID:
        // yaw rate error from command to yaw servo deviation from trim
        // (stabilizes dutch roll)
        //
        k_param_yaw_to_servo_PID,

        //
        // Throttle control
        //
        // Energy-to-throttle PID:
        // total energy error from command to throttle servo deviation from trim
        // (throttle back-side strategy alternative)
        //
        k_param_energy_to_throttle_PID,

        // Altitude-to-pitch PID:
        // altitude error from command to pitch servo deviation from trim
        // (throttle front-side strategy alternative)
        //
        k_param_altitude_to_pitch_PID,

        // 254,255: reserved
    };

    AP_Int16    format_version;
	AP_Int8		software_type;

	// Telemetry control
	//
	AP_Int16		sysid_this_mav;
	AP_Int16		sysid_my_gcs;
    AP_Int8			serial3_baud;

    // Feed-forward gains
    //
    AP_Float    kff_pitch_compensation;
    AP_Float    kff_rudder_mix;
    AP_Float    kff_pitch_to_throttle;
    AP_Float    kff_throttle_to_pitch;

    // Crosstrack navigation
    //
    AP_Float    crosstrack_gain;
    AP_Int16    crosstrack_entry_angle;

    // Estimation
    //
    AP_Float    altitude_mix;
    AP_Float    airspeed_ratio;
	AP_Int16	airspeed_offset;

    // Waypoints
    //
    AP_Int8     waypoint_mode;
    AP_Int8     command_total;
    AP_Int8     command_index;
    AP_Int8     waypoint_radius;
    AP_Int8     loiter_radius;

    // Fly-by-wire
    //
    AP_Int8     flybywire_airspeed_min;
    AP_Int8     flybywire_airspeed_max;
    AP_Int16    FBWB_min_altitude;

    // Throttle
    //
    AP_Int8     throttle_min;
    AP_Int8     throttle_max;
    AP_Int8     throttle_slewrate;
	AP_Int8     throttle_fs_enabled;
    AP_Int16    throttle_fs_value;
    AP_Int8     throttle_cruise;

	// Failsafe
    AP_Int8     short_fs_action;
    AP_Int8     long_fs_action;
	AP_Int8		gcs_heartbeat_fs_enabled;

    // Flight modes
    //
    AP_Int8     flight_mode_channel;
    AP_Int8     flight_mode1;
    AP_Int8     flight_mode2;
    AP_Int8     flight_mode3;
    AP_Int8     flight_mode4;
    AP_Int8     flight_mode5;
    AP_Int8     flight_mode6;

    // Navigational maneuvering limits
    //
    AP_Int16    roll_limit;
    AP_Int16    pitch_limit_max;
    AP_Int16    pitch_limit_min;

    // Misc
    //
    AP_Int8     auto_trim;
    AP_Int8     switch_enable;
    AP_Int8     mix_mode;
    AP_Int8     reverse_elevons;
    AP_Int8     reverse_ch1_elevon;
    AP_Int8     reverse_ch2_elevon;
    AP_Int16    num_resets;
    AP_Int16    log_bitmask;
    AP_Int16    airspeed_cruise;
    AP_Int16    pitch_trim;
    AP_Int16    RTL_altitude;
    AP_Int16    ground_temperature;
    AP_Int32    ground_pressure;
    AP_Int8		compass_enabled;
    AP_Int16    angle_of_attack;
    AP_Int8		battery_monitoring;	// 0=disabled, 1=3 cell lipo, 2=4 cell lipo, 3=total voltage only, 4=total voltage and current
    AP_Float	volt_div_ratio;
    AP_Float	curr_amp_per_volt;
    AP_Float	input_voltage;
	AP_Int16	pack_capacity;		// Battery pack capacity less reserve
    AP_Int8		inverted_flight_ch; // 0=disabled, 1-8 is channel for inverted flight trigger
    AP_Int8		sonar_enabled;
    AP_Int8		airspeed_enabled;
    AP_Int8		flap_1_percent;
    AP_Int8		flap_1_speed;
    AP_Int8		flap_2_percent;
    AP_Int8		flap_2_speed;

    // RC channels
    RC_Channel  channel_roll;
    RC_Channel  channel_pitch;
    RC_Channel  channel_throttle;
    RC_Channel  channel_rudder;
	RC_Channel_aux	rc_5;
	RC_Channel_aux	rc_6;
	RC_Channel_aux	rc_7;
	RC_Channel_aux	rc_8;

    // PID controllers
    //
    PID         pidNavRoll;
    PID         pidServoRoll;
    PID         pidServoPitch;
    PID         pidNavPitchAirspeed;
    PID         pidServoRudder;
    PID         pidTeThrottle;
    PID         pidNavPitchAltitude;

    uint8_t     junk;

    // Note: keep initializers here in the same order as they are declared above.
    Parameters() :
        // variable             default                     key                             name
        //-------------------------------------------------------------------------------------------------------
        format_version          (k_format_version,          k_param_format_version,         PSTR("SYSID_SW_MREV")),
        software_type			(k_software_type,			k_param_software_type,			PSTR("SYSID_SW_TYPE")),

        sysid_this_mav			(MAV_SYSTEM_ID,				k_param_sysid_this_mav,			PSTR("SYSID_THISMAV")),
        sysid_my_gcs			(255,		                k_param_sysid_my_gcs,			PSTR("SYSID_MYGCS")),
        serial3_baud			(SERIAL3_BAUD/1000,         k_param_serial3_baud,			PSTR("SERIAL3_BAUD")),

        kff_pitch_compensation  (PITCH_COMP,                k_param_kff_pitch_compensation, PSTR("KFF_PTCHCOMP")),
        kff_rudder_mix          (RUDDER_MIX,                k_param_kff_rudder_mix,         PSTR("KFF_RDDRMIX")),
        kff_pitch_to_throttle   (P_TO_T,                    k_param_kff_pitch_to_throttle,  PSTR("KFF_PTCH2THR")),
        kff_throttle_to_pitch   (T_TO_P,                    k_param_kff_throttle_to_pitch,  PSTR("KFF_THR2PTCH")),

        crosstrack_gain         (XTRACK_GAIN_SCALED,        k_param_crosstrack_gain,        PSTR("XTRK_GAIN_SC")),
        crosstrack_entry_angle  (XTRACK_ENTRY_ANGLE_CENTIDEGREE,k_param_crosstrack_entry_angle, PSTR("XTRK_ANGLE_CD")),

        altitude_mix            (ALTITUDE_MIX,              k_param_altitude_mix,           PSTR("ALT_MIX")),
        airspeed_ratio          (AIRSPEED_RATIO,            k_param_airspeed_ratio,         PSTR("ARSPD_RATIO")),
        airspeed_offset			(0,				            k_param_airspeed_offset,		PSTR("ARSPD_OFFSET")),

        /* XXX waypoint_mode missing here */
        command_total           (0,                         k_param_command_total,         PSTR("CMD_TOTAL")),
        command_index           (0,                         k_param_command_index,         PSTR("CMD_INDEX")),
        waypoint_radius         (WP_RADIUS_DEFAULT,         k_param_waypoint_radius,        PSTR("WP_RADIUS")),
        loiter_radius           (LOITER_RADIUS_DEFAULT,     k_param_loiter_radius,          PSTR("WP_LOITER_RAD")),

        flybywire_airspeed_min  (AIRSPEED_FBW_MIN,          k_param_flybywire_airspeed_min, PSTR("ARSPD_FBW_MIN")),
        flybywire_airspeed_max  (AIRSPEED_FBW_MAX,          k_param_flybywire_airspeed_max, PSTR("ARSPD_FBW_MAX")),

        throttle_min            (THROTTLE_MIN,              k_param_throttle_min,           PSTR("THR_MIN")),
        throttle_max            (THROTTLE_MAX,              k_param_throttle_max,           PSTR("THR_MAX")),
        throttle_slewrate		(THROTTLE_SLEW_LIMIT,		k_param_throttle_slewrate,		PSTR("THR_SLEWRATE")),
        throttle_fs_enabled   	(THROTTLE_FAILSAFE,         k_param_throttle_fs_enabled,	PSTR("THR_FAILSAFE")),
        throttle_fs_value 		(THROTTLE_FS_VALUE,         k_param_throttle_fs_value, 		PSTR("THR_FS_VALUE")),
        throttle_cruise         (THROTTLE_CRUISE,           k_param_throttle_cruise,        PSTR("TRIM_THROTTLE")),

        short_fs_action			(SHORT_FAILSAFE_ACTION,		k_param_short_fs_action, 	PSTR("FS_SHORT_ACTN")),
        long_fs_action			(LONG_FAILSAFE_ACTION,		k_param_long_fs_action, 	PSTR("FS_LONG_ACTN")),
        gcs_heartbeat_fs_enabled(GCS_HEARTBEAT_FAILSAFE,	k_param_gcs_heartbeat_fs_enabled, 	PSTR("FS_GCS_ENABL")),

        flight_mode_channel     (FLIGHT_MODE_CHANNEL,       k_param_flight_mode_channel,    PSTR("FLTMODE_CH")),
        flight_mode1            (FLIGHT_MODE_1,             k_param_flight_mode1,           PSTR("FLTMODE1")),
        flight_mode2            (FLIGHT_MODE_2,             k_param_flight_mode2,           PSTR("FLTMODE2")),
        flight_mode3            (FLIGHT_MODE_3,             k_param_flight_mode3,           PSTR("FLTMODE3")),
        flight_mode4            (FLIGHT_MODE_4,             k_param_flight_mode4,           PSTR("FLTMODE4")),
        flight_mode5            (FLIGHT_MODE_5,             k_param_flight_mode5,           PSTR("FLTMODE5")),
        flight_mode6            (FLIGHT_MODE_6,             k_param_flight_mode6,           PSTR("FLTMODE6")),

        roll_limit              (HEAD_MAX_CENTIDEGREE,      k_param_roll_limit,             PSTR("LIM_ROLL_CD")),
        pitch_limit_max         (PITCH_MAX_CENTIDEGREE,     k_param_pitch_limit_max,        PSTR("LIM_PITCH_MAX")),
        pitch_limit_min         (PITCH_MIN_CENTIDEGREE,     k_param_pitch_limit_min,        PSTR("LIM_PITCH_MIN")),

        auto_trim               (AUTO_TRIM,                 k_param_auto_trim,              PSTR("TRIM_AUTO")),
        switch_enable           (REVERSE_SWITCH,            k_param_switch_enable,          PSTR("SWITCH_ENABLE")),
        mix_mode                (ELEVON_MIXING,             k_param_mix_mode,               PSTR("ELEVON_MIXING")),
        reverse_elevons         (ELEVON_REVERSE,            k_param_reverse_elevons,        PSTR("ELEVON_REVERSE")),
        reverse_ch1_elevon      (ELEVON_CH1_REVERSE,        k_param_reverse_ch1_elevon,     PSTR("ELEVON_CH1_REV")),
        reverse_ch2_elevon      (ELEVON_CH2_REVERSE,        k_param_reverse_ch2_elevon,     PSTR("ELEVON_CH2_REV")),
        num_resets              (0,                         k_param_num_resets,             PSTR("SYS_NUM_RESETS")),
        log_bitmask             (DEFAULT_LOG_BITMASK,		k_param_log_bitmask,            PSTR("LOG_BITMASK")),
        airspeed_cruise         (AIRSPEED_CRUISE_CM,        k_param_airspeed_cruise,        PSTR("TRIM_ARSPD_CM")),
        pitch_trim              (0,                         k_param_pitch_trim,             PSTR("TRIM_PITCH_CD")),
        RTL_altitude            (ALT_HOLD_HOME_CM,          k_param_RTL_altitude,           PSTR("ALT_HOLD_RTL")),
        FBWB_min_altitude       (ALT_HOLD_FBW_CM,           k_param_FBWB_min_altitude,      PSTR("ALT_HOLD_FBWCM")),
        ground_temperature      (0,                         k_param_ground_temperature,     PSTR("GND_TEMP")),
        ground_pressure         (0,                         k_param_ground_pressure,        PSTR("GND_ABS_PRESS")),
        compass_enabled			(MAGNETOMETER,				k_param_compass_enabled,		PSTR("MAG_ENABLE")),
        flap_1_percent			(FLAP_1_PERCENT,			k_param_flap_1_percent,			PSTR("FLAP_1_PERCNT")),
        flap_1_speed			(FLAP_1_SPEED,				k_param_flap_1_speed,			PSTR("FLAP_1_SPEED")),
        flap_2_percent			(FLAP_2_PERCENT,			k_param_flap_2_percent,			PSTR("FLAP_2_PERCNT")),
        flap_2_speed			(FLAP_2_SPEED,				k_param_flap_2_speed,			PSTR("FLAP_2_SPEED")),


        battery_monitoring 		(DISABLED,					k_param_battery_monitoring,		PSTR("BATT_MONITOR")),
        volt_div_ratio			(VOLT_DIV_RATIO,			k_param_volt_div_ratio,			PSTR("VOLT_DIVIDER")),
        curr_amp_per_volt		(CURR_AMP_PER_VOLT,			k_param_curr_amp_per_volt,		PSTR("AMP_PER_VOLT")),
        input_voltage			(INPUT_VOLTAGE,				k_param_input_voltage,			PSTR("INPUT_VOLTS")),
        pack_capacity	 		(HIGH_DISCHARGE,			k_param_pack_capacity,			PSTR("BATT_CAPACITY")),
        inverted_flight_ch      (0,                         k_param_inverted_flight_ch,		PSTR("INVERTEDFLT_CH")),
        sonar_enabled			(SONAR_ENABLED,				k_param_sonar_enabled,			PSTR("SONAR_ENABLE")),
        airspeed_enabled		(AIRSPEED_SENSOR,			k_param_airspeed_enabled,		PSTR("ARSPD_ENABLE")),

	// Note - total parameter name length must be less than 14 characters for MAVLink compatibility!

        // RC channel           group key                   name
        //----------------------------------------------------------------------
        channel_roll            (k_param_channel_roll,      PSTR("RC1_")),
        channel_pitch           (k_param_channel_pitch,     PSTR("RC2_")),
        channel_throttle        (k_param_channel_throttle,  PSTR("RC3_")),
        channel_rudder          (k_param_channel_yaw,       PSTR("RC4_")),
        rc_5					(k_param_rc_5,				PSTR("RC5_")),
        rc_6					(k_param_rc_6,				PSTR("RC6_")),
        rc_7					(k_param_rc_7,				PSTR("RC7_")),
        rc_8					(k_param_rc_8,				PSTR("RC8_")),

        // PID controller   group key                       name                  initial P        initial I        initial D        initial imax
        //---------------------------------------------------------------------------------------------------------------------------------------
        pidNavRoll          (k_param_heading_to_roll_PID,   PSTR("HDNG2RLL_"),    NAV_ROLL_P,      NAV_ROLL_I,      NAV_ROLL_D,      NAV_ROLL_INT_MAX_CENTIDEGREE),
        pidServoRoll        (k_param_roll_to_servo_PID,     PSTR("RLL2SRV_"),     SERVO_ROLL_P,    SERVO_ROLL_I,    SERVO_ROLL_D,    SERVO_ROLL_INT_MAX_CENTIDEGREE),
        pidServoPitch       (k_param_pitch_to_servo_PID,    PSTR("PTCH2SRV_"),    SERVO_PITCH_P,   SERVO_PITCH_I,   SERVO_PITCH_D,   SERVO_PITCH_INT_MAX_CENTIDEGREE),
        pidNavPitchAirspeed (k_param_airspeed_to_pitch_PID, PSTR("ARSP2PTCH_"),  NAV_PITCH_ASP_P, NAV_PITCH_ASP_I, NAV_PITCH_ASP_D, NAV_PITCH_ASP_INT_MAX_CMSEC),
        pidServoRudder      (k_param_yaw_to_servo_PID,      PSTR("YW2SRV_"),      SERVO_YAW_P,     SERVO_YAW_I,     SERVO_YAW_D,     SERVO_YAW_INT_MAX),
        pidTeThrottle       (k_param_energy_to_throttle_PID, PSTR("ENRGY2THR_"), THROTTLE_TE_P,   THROTTLE_TE_I,   THROTTLE_TE_D,   THROTTLE_TE_INT_MAX),
        pidNavPitchAltitude (k_param_altitude_to_pitch_PID, PSTR("ALT2PTCH_"),    NAV_PITCH_ALT_P, NAV_PITCH_ALT_I, NAV_PITCH_ALT_D, NAV_PITCH_ALT_INT_MAX_CM),


        junk(0)     // XXX just so that we can add things without worrying about the trailing comma
    {
    }
};

#endif // PARAMETERS_H
