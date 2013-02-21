/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduBalance parameter definitions
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
    // @Param: SYSID_SW_MREV
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "SYSID_SW_MREV",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @User: Advanced
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: Mavlink version
    // @Description: Allows reconising the mavlink version
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,   "SERIAL3_BAUD",     SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: SONAR_ENABLE
    // @DisplayName: Enable Sonar
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(sonar_enabled,  "SONAR_ENABLE",     DISABLED),

    // @Param: SONAR_TYPE
    // @DisplayName: Sonar type
    // @Description: Used to adjust scaling to match the sonar used (only Maxbotix sonars are supported at this time)
    // @Values: 0:XL-EZ0,1:LV-EZ0,2:XLL-EZ0,3:HRLV
    // @User: Standard
    GSCALAR(sonar_type,     "SONAR_TYPE",           AP_RANGEFINDER_MAXSONARXL),

    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
    GSCALAR(battery_monitoring, "BATT_MONITOR", DISABLED),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATTERY),

    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin voltage * INPUT_VOLTS/1024 * VOLT_DIVIDER)
    // @User: Advanced
    GSCALAR(volt_div_ratio, "VOLT_DIVIDER",     VOLT_DIV_RATIO),

    // @Param: AMP_PER_VOLT
    // @DisplayName: Current Amps per volt
    // @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps (curr pin voltage * INPUT_VOLTS/1024 * AMP_PER_VOLT )
    // @User: Advanced
    GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT", CURR_AMP_PER_VOLT),

    // @Param: INPUT_VOLTS
    // @DisplayName: Max internal voltage of the battery voltage and current sensing pins
    // @Description: Used to convert the voltage read in on the voltage and current pins for battery monitoring.  Normally 5 meaning 5 volts.
    // @User: Advanced
    GSCALAR(input_voltage,  "INPUT_VOLTS",      INPUT_VOLTAGE),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
    GSCALAR(pack_capacity,  "BATT_CAPACITY",    HIGH_DISCHARGE),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: FLOW_ENABLE
    // @DisplayName: Enable Optical Flow
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(optflow_enabled,        "FLOW_ENABLE",  DISABLED),

    // @Param: LOW_VOLT
    // @DisplayName: Low Voltage
    // @Description: Set this to the voltage you want to represent low voltage
    // @Range: 0 20
    // @Increment: .1
    // @User: Standard
    GSCALAR(low_voltage,    "LOW_VOLT",    LOW_VOLTAGE),
    GSCALAR(fbw_speed,   	"FBW_SPEED",   12),			// 500 / 12 = 42
    GSCALAR(throttle,   	"P_THROT",     0.03333), 	// ~3 second time constant
    GSCALAR(dead_zone,   	"P_DEAD_Z",    50), 		//
    GSCALAR(p_vel,    	 	"P_VEL",       VELOCITY_P),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    BATTERY_VOLT_PIN),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    BATTERY_CURR_PIN),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: WP_TOTAL
    // @DisplayName: Waypoint Total
    // @Description: Total number of commands in the mission stored in the eeprom.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_total,  "WP_TOTAL",         0),

    // @Param: WP_INDEX
    // @DisplayName: Waypoint Index
    // @Description: The index number of the command that is currently being executed.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_index,  "WP_INDEX",         0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",    WP_RADIUS_DEFAULT),

    // @Param: CIRCLE_RADIUS
    // @DisplayName: Circle radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_radius,  "CIRCLE_RADIUS",    CIRCLE_RADIUS),

	// @Param: WP_SPEED_MAX
    // @DisplayName: Waypoint Max Speed Target
    // @Description: Defines the speed which the aircraft will attempt to maintain during a WP mission.
    // @Units: Centimeters/Second
    // @Increment: 100
    // @User: Standard
    GSCALAR(waypoint_speed_max,     "WP_SPEED_MAX", WAYPOINT_SPEED_MAX),

	// @Param: XTRK_GAIN_SC
    // @DisplayName: Cross-Track Gain
    // @Description: This controls the rate that the Auto Controller will attempt to return original track
    // @Units: Dimensionless
	// @User: Standard
    GSCALAR(crosstrack_gain,        "XTRK_GAIN_SC", CROSSTRACK_GAIN),

    // @Param: XTRK_MIN_DIST
    // @DisplayName: Crosstrack mininum distance
    // @Description: Minimum distance in meters between waypoints to do crosstrack correction.
    // @Units: Meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(crosstrack_min_distance, "XTRK_MIN_DIST",  CROSSTRACK_MIN_DISTANCE),


    GSCALAR(wheel_encoder_speed, "WHEEL_ENC",  WHEEL_ENCODER_SPEED),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_DISABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 2 byte bitmap of log types to enable
    // @User: Advanced
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:CH6_NONE,1:CH6_STABILIZE_KP,2:CH6_STABILIZE_KI,3:CH6_YAW_KP,4:CH6_RATE_KP,5:CH6_RATE_KI,6:CH6_YAW_RATE_KP,7:CH6_THROTTLE_KP,8:CH6_TOP_BOTTOM_RATIO,9:CH6_RELAY,10:CH6_TRAVERSE_SPEED,11:CH6_NAV_KP,12:CH6_LOITER_KP,13:CH6_HELI_EXTERNAL_GYRO,14:CH6_THR_HOLD_KP,17:CH6_OPTFLOW_KP,18:CH6_OPTFLOW_KI,19:CH6_OPTFLOW_KD,20:CH6_NAV_KI,21:CH6_RATE_KD,22:CH6_LOITER_RATE_KP,23:CH6_LOITER_RATE_KD,24:CH6_YAW_KI,25:CH6_ACRO_KP,26:CH6_YAW_RATE_KD,27:CH6_LOITER_KI,28:CH6_LOITER_RATE_KI,29:CH6_STABILIZE_KD,30:CH6_AHRS_YAW_KP,31:CH6_AHRS_KP,32:CH6_INAV_TC,33:CH6_THROTTLE_KI,34:CH6_THR_ACCEL_KP,35:CH6_THR_ACCEL_KI,36:CH6_THR_ACCEL_KD
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: TUNE_LOW
    // @DisplayName: Tuning minimum
    // @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_low, "TUNE_LOW",           0),

    // @Param: TUNE_HIGH
    // @DisplayName: Tuning maximum
    // @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_high, "TUNE_HIGH",         1000),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  CH7_OPTION),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    //GGROUP(camera,                  "CAM_", AP_Camera),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,    "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,    "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,    "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,    "RC4_", RC_Channel),
    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if MOUNT == ENABLED
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hertz (Hz)
    // @Values: 125,400,490
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: LED_MODE
    // @DisplayName: Copter LED Mode
    // @Description: bitmap to control the copter led mode
    // @Values: 0:Disabled,1:Enable,2:GPS On,4:Aux,8:Buzzer,16:Oscillate,32:Nav Blink,64:GPS Nav Blink
    // @User: Standard
    GSCALAR(copter_leds_mode,       "LED_MODE",         9),

    // PID controller
    //---------------
    GGROUP(pid_balance,    			"PID_BAL", 	AC_PID),
    GGROUP(pid_yaw,   				"PID_YAW_", AC_PID),
    GGROUP(pid_wheel_left_mixer,	"PID_LW_",  AC_PID),
    GGROUP(pid_wheel_right_mixer,   "PID_RW_",  AC_PID),
    GGROUP(pid_nav,     			"PID_NAV_", AC_PID),


    // variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
#if HIL_MODE == HIL_MODE_DISABLED
    GOBJECT(ins,            "INS_", AP_InertialSensor),
#endif

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    // @Group: INAV_
    // @Path: ../libraries/AP_InertialNav/AP_InertialNav.cpp
    GOBJECT(inertial_nav,           "INAV_",    AP_InertialNav),
#endif

    GOBJECT(gcs0,                   "SR0_",     GCS_MAVLINK),
    GOBJECT(gcs3,                   "SR3_",     GCS_MAVLINK),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    //GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    //GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

#ifdef DESKTOP_BUILD
    GOBJECT(sitl, "SIM_", SITL),
#endif

    //@Group: LIM_
    //@Path: ../libraries/AP_Limits/AP_Limits.cpp,../libraries/AP_Limits/AP_Limit_GPSLock.cpp, ../libraries/AP_Limits/AP_Limit_Geofence.cpp, ../libraries/AP_Limits/AP_Limit_Altitude.cpp, ../libraries/AP_Limits/AP_Limit_Module.cpp
    GOBJECT(limits,                 "LIM_",    AP_Limits),
    GOBJECT(gpslock_limit,          "LIM_",    AP_Limit_GPSLock),
    GOBJECT(geofence_limit,         "LIM_",    AP_Limit_Geofence),
    GOBJECT(altitude_limit,         "LIM_",    AP_Limit_Altitude),

    AP_VAREND
};


static void load_parameters(void)
{
    // change the default for the AHRS_GPS_GAIN for ArduCopter
    // if it hasn't been set by the user
    if (!ahrs.gps_gain.load()) {
        ahrs.gps_gain.set_and_save(1.0);
    }

    // setup different AHRS gains for ArduCopter than the default
    // but allow users to override in their config
    if (!ahrs._kp.load()) {
        ahrs._kp.set_and_save(0.1);
    }
    if (!ahrs._kp_yaw.load()) {
        ahrs._kp_yaw.set_and_save(0.1);
    }

#if SECONDARY_DMP_ENABLED == ENABLED
    if (!ahrs2._kp.load()) {
        ahrs2._kp.set(0.1);
    }
    if (!ahrs2._kp_yaw.load()) {
        ahrs2._kp_yaw.set(0.1);
    }
#endif


    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        default_dead_zones();
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();

        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
