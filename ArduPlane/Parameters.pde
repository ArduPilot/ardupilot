/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduPlane parameter definitions
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    GSCALAR(format_version,         "FORMAT_VERSION", 0),
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,           "SERIAL3_BAUD",   SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: KFF_PTCHCOMP
    // @DisplayName: Pitch Compensation
    // @Description: Adds pitch input to compensate for the loss of lift due to roll control. 0 = 0 %, 1 = 100%
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_pitch_compensation, "KFF_PTCHCOMP",   PITCH_COMP),

    // @Param: KFF_RDDRMIX
    // @DisplayName: Rudder Mix
    // @Description: The amount of rudder mix to apply during aileron movement 0 = 0 %, 1 = 100%
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_PTCH2THR
    // @DisplayName: Pitch to Throttle Mix
    // @Description: Pitch to throttle feed-forward gain.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_pitch_to_throttle,  "KFF_PTCH2THR",   P_TO_T),

    // @Param: KFF_THR2PTCH
    // @DisplayName: Throttle to Pitch Mix
    // @Description: Throttle to pitch feed-forward gain.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   T_TO_P),

    // @Param: MANUAL_LEVEL
    // @DisplayName: Manual Level
    // @Description: Setting this to Disabled(0) will enable autolevel on every boot. Setting it to Enabled(1) will do a calibration only when you tell it to
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(manual_level,           "MANUAL_LEVEL",   MANUAL_LEVEL),

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds user stick input to the control surfaces in auto modes, allowing the user to have some degree of flight control without changing modes
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   1),

    // @Param: land_pitch_cd
    // @DisplayName: Landing Pitch
    // @Description: Used in autoland for planes without airspeed sensors in hundredths of a degree
    // @Units: centi-Degrees
    // @User: Advanced
    GSCALAR(land_pitch_cd,          "LAND_PITCH_CD",  0),

    // @Param: land_flare_alt
    // @DisplayName: Landing flare altitude
    // @Description: Altitude in autoland at which to lock heading and flare to the LAND_PITCH_CD pitch
    // @Units: meters
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_alt,          "LAND_FLARE_ALT",  3.0),

    // @Param: land_flare_sec
    // @DisplayName: Landing flare time
    // @Description: Time before landing point at which to lock heading and flare to the LAND_PITCH_CD pitch
    // @Units: seconds
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_sec,          "LAND_FLARE_SEC",  2.0),

    // @Param: XTRK_GAIN_SC
    // @DisplayName: Crosstrack Gain
    // @Description: The scale between distance off the line and angle to meet the line (in Degrees * 100)
    // @Range: 0 2000
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC",   XTRACK_GAIN_SCALED),

    // @Param: XTRK_ANGLE_CD
    // @DisplayName: Crosstrack Entry Angle
    // @Description: Maximum angle used to correct for track following.
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_entry_angle, "XTRK_ANGLE_CD",  XTRACK_ENTRY_ANGLE_CENTIDEGREE),

    // @Param: XTRK_USE_WIND
    // @DisplayName: Crosstrack Wind correction
    // @Description: If enabled, use wind estimation for navigation crosstrack when using a compass for yaw
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(crosstrack_use_wind, "XTRK_USE_WIND",  1),

    // @Param: ALT_MIX
    // @DisplayName: Gps to Baro Mix
    // @Description: The percent of mixing between gps altitude and baro altitude. 0 = 100% gps, 1 = 100% baro
    // @Units: Percent
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(altitude_mix,           "ALT_MIX",        ALTITUDE_MIX),

    // @Param: ALT_CTRL_ALG
    // @DisplayName: Altitude control algorithm
    // @Description: This sets what algorithm will be used for altitude control. The default is to select the algorithm based on whether airspeed is enabled. If you set it to 1, then the airspeed based algorithm won't be used for altitude control, but airspeed can be used for other flight control functions
    // @Values: 0:Default Method,1:non-airspeed
    // @User: Advanced
    GSCALAR(alt_control_algorithm, "ALT_CTRL_ALG",    ALT_CONTROL_DEFAULT),

    GSCALAR(command_total,          "CMD_TOTAL",      0),
    GSCALAR(command_index,          "CMD_INDEX",      0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_LOITER_RAD
    // @DisplayName: Waypoint Loiter Radius
    // @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

#if GEOFENCE_ENABLED == ENABLED
    // @Param: FENCE_ACTION
    // @DisplayName: Action on geofence breach
    // @Description: What to do on fence breach
    // @Values: 0:None,1:GuidedMode,2:ReportOnly
    // @User: Standard
    GSCALAR(fence_action,           "FENCE_ACTION",   0),

    // @Param: FENCE_TOTAL
    // @DisplayName: Fence Total
    // @Description: Number of geofence points currently loaded
    // @User: Standard
    GSCALAR(fence_total,            "FENCE_TOTAL",    0),

    // @Param: FENCE_CHANNEL
    // @DisplayName: Fence Channel
    // @Description: RC Channel to use to enable geofence. PWM input above 1750 enables the geofence
    // @User: Standard
    GSCALAR(fence_channel,          "FENCE_CHANNEL",  0),

    // @Param: FENCE_MINALT
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_minalt,           "FENCE_MINALT",   0),

    // @Param: FENCE_MAXALT
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_maxalt,           "FENCE_MAXALT",   0),
#endif

    // @Param: ARSPD_FBW_MIN
    // @DisplayName: Fly By Wire Minimum Airspeed
    // @Description: Airspeed corresponding to minimum throttle in Fly By Wire B mode.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Standard
    GSCALAR(flybywire_airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),

    // @Param: ARSPD_FBW_MAX
    // @DisplayName: Fly By Wire Maximum Airspeed
    // @Description: Airspeed corresponding to maximum throttle in Fly By Wire B mode.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Standard
    GSCALAR(flybywire_airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: Fly By Wire elevator reverse
    // @Description: Reverse sense of elevator in FBWB. When set to 0 up elevator (pulling back on the stick) means to lower altitude. When set to 1, up elevator means to raise altitude.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttlw slew rate
    // @Description: maximum percentage change in throttle per second
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_slewrate,      "THR_SLEWRATE",   THROTTLE_SLEW_LIMIT),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   THROTTLE_FAILSAFE),


    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   THROTTLE_FS_VALUE),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle cruise percentage
    // @Description: The target percentage of throttle to apply for normal flight
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_cruise,        "TRIM_THROTTLE",  THROTTLE_CRUISE),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: Short failsafe action
    // @Description: The action to take on a short (1 second) failsafe event
    // @Values: 0:None,1:ReturnToLaunch
    // @User: Standard
    GSCALAR(short_fs_action,        "FS_SHORT_ACTN",  SHORT_FAILSAFE_ACTION),

    // @Param: FS_LONG_ACTN
    // @DisplayName: Long failsafe action
    // @Description: The action to take on a long (20 second) failsafe event
    // @Values: 0:None,1:ReturnToLaunch
    // @User: Standard
    GSCALAR(long_fs_action,         "FS_LONG_ACTN",   LONG_FAILSAFE_ACTION),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after 20 seconds of no MAVLink heartbeat messages
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_HEARTBEAT_FAILSAFE),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: FlightMode1
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    // @Description: Flight mode for switch position 1 (910 to 1230 and above 2049)
    GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: FlightMode2
    // @Description: Flight mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: FlightMode3
    // @Description: Flight mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: FlightMode4
    // @Description: Flight mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: FlightMode5
    // @Description: Flight mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: FlightMode6
    // @Description: Flight mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,5:FBWA,6:FBWB,10:Auto,11:RTL,12:Loiter,15:Guided
    // @User: Standard
    GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

    // @Param: LIM_ROLL_CD
    // @DisplayName: Maximum Bank Angle
    // @Description: The maximum commanded bank angle in either direction
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(roll_limit_cd,          "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MAX
    // @DisplayName: Maximum Pitch Angle
    // @Description: The maximum commanded pitch up angle
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(pitch_limit_max_cd,     "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MIN
    // @DisplayName: Minimum Pitch Angle
    // @Description: The minimum commanded pitch down angle
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(pitch_limit_min_cd,     "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

    // @Param: AUTO_TRIM
    // @DisplayName: Auto trim
    // @Description: Set RC trim PWM levels to current levels when switching away from manual mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auto_trim,              "TRIM_AUTO",      AUTO_TRIM),

    // @Param: MIX_MODE
    // @DisplayName: Elevon mixing
    // @Description: Enable elevon mixing
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(mix_mode,               "ELEVON_MIXING",  ELEVON_MIXING),

    // @Param: ELEVON_REVERSE
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon mixing
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_elevons,        "ELEVON_REVERSE", ELEVON_REVERSE),


    // @Param: ELEVON_CH1_REV
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon channel 1
    // @Values: -1:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV", ELEVON_CH1_REVERSE),

    // @Param: ELEVON_CH2_REV
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon channel 2
    // @Values: -1:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV", ELEVON_CH2_REVERSE),

    // @Param: SYS_NUM_RESETS
    // @DisplayName: Num Resets
    // @Description: Number of APM board resets
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: bitmap of log fields to enable
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode	after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",  0),

    // @Param: RST_MISSION_CH
    // @DisplayName: Reset Mission Channel
    // @Description: RC channel to use to reset the mission to the first waypoint. When this channel goes above 1750 the mission is reset. Set RST_MISSION_CH to 0 to disable.
    // @User: Advanced
    GSCALAR(reset_mission_chan,      "RST_MISSION_CH",  0),

    // @Param: TRIM_ARSPD_CM
    // @DisplayName: Target airspeed
    // @Description: Airspeed in cm/s to aim for when airspeed is enabled in auto mode
    // @Units: cm/s
    // @User: User
    GSCALAR(airspeed_cruise_cm,     "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),

    // @Param: SCALING_SPEED
    // @DisplayName: speed used for speed scaling calculations
    // @Description: Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values
    // @Units: m/s
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GNDSPD_CM
    // @DisplayName: Minimum ground speed
    // @Description: Minimum ground speed in cm/s when under airspeed control
    // @Units: cm/s
    // @User: Advanced
    GSCALAR(min_gndspeed_cm,      "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),

    // @Param: TRIM_PITCH_CD
    // @DisplayName: Pitch angle offset
    // @Description: offset to add to pitch - used for trimming tail draggers
    // @Units: centi-Degrees
    // @User: Advanced
    GSCALAR(pitch_trim_cd,        "TRIM_PITCH_CD",  0),

    // @Param: ALT_HOLD_RTL
    // @DisplayName: RTL altitude
    // @Description: Return to launch target altitude
    // @Units: centimeters
    // @User: User
    GSCALAR(RTL_altitude_cm,        "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),

    GSCALAR(FBWB_min_altitude_cm,   "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",     MAGNETOMETER),

    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),


    GSCALAR(battery_monitoring,     "BATT_MONITOR",   0),
    GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",   VOLT_DIV_RATIO),
    GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",   CURR_AMP_PER_VOLT),
    GSCALAR(input_voltage,          "INPUT_VOLTS",    INPUT_VOLTAGE),
    GSCALAR(pack_capacity,          "BATT_CAPACITY",  HIGH_DISCHARGE),
    GSCALAR(inverted_flight_ch,     "INVERTEDFLT_CH", 0),

    // @Param: SONAR_ENABLE
    // @DisplayName: Enable Sonar
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(sonar_enabled,          "SONAR_ENABLE",   SONAR_ENABLED),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    GOBJECT(barometer, "GND_", AP_Baro),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GGROUP(camera,                  "CAM_", AP_Camera),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_roll,            "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_pitch,           "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_throttle,        "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(channel_rudder,          "RC4_", RC_Channel),

    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_5,                    "RC5_", RC_Channel_aux),

    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_6,                    "RC6_", RC_Channel_aux),

    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_7,                    "RC7_", RC_Channel_aux),

    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),

    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp, ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

	GGROUP(pidNavRoll,              "HDNG2RLL_",  PID),
	GGROUP(pidNavPitchAirspeed,     "ARSP2PTCH_", PID),
	GGROUP(pidTeThrottle,           "ENRGY2THR_", PID),
	GGROUP(pidNavPitchAltitude,     "ALT2PTCH_",  PID),
	GGROUP(pidWheelSteer,           "WHEELSTEER_",PID),

#if APM_CONTROL == DISABLED
	GGROUP(pidServoRoll,            "RLL2SRV_",   PID),
	GGROUP(pidServoPitch,           "PTCH2SRV_",  PID),
	GGROUP(pidServoRudder,          "YW2SRV_",    PID),
#else
	GGROUP(rollController,          "RLL_",       AP_RollController),
	GGROUP(pitchController,         "PTCH_",      AP_PitchController),
	GGROUP(yawController,           "YWCTL_",     AP_YawController),
#endif

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),
    GOBJECT(gcs0,                                   "SR0_",     GCS_MAVLINK),
    GOBJECT(gcs3,                                   "SR3_",     GCS_MAVLINK),

#if HIL_MODE == HIL_MODE_DISABLED && CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor_Oilpan.cpp
    GOBJECT(ins,                            "INS_", AP_InertialSensor_Oilpan),
#endif

    // @Group: IMU_
    // @Path: ../libraries/AP_IMU/IMU.cpp
    GOBJECT(imu,                                    "IMU_",     IMU),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: ARSPD_
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSPD_",   AP_Airspeed),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

#ifdef DESKTOP_BUILD
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

#if OBC_FAILSAFE == ENABLED
    GOBJECT(obc,  "FS_", APM_OBC),
#endif

    AP_VAREND
};


static void load_parameters(void)
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        Serial.printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        Serial.println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();

        Serial.printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
