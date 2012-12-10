/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduCopter parameter definitions
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

    // @Param: ALT_RTL
    // @DisplayName: RTL Altitude
    // @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
    // @Units: Centimeters
    // @Range: 0 4000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

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

    // @Param: BATT_FAILSAFE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(battery_fs_enabled, "BATT_FAILSAFE", BATTERY_FAILSAFE),

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
    GSCALAR(optflow_enabled,        "FLOW_ENABLE",  OPTFLOW),

    // @Param: LOW_VOLT
    // @DisplayName: Low Voltage
    // @Description: Set this to the voltage you want to represent low voltage
    // @Range: 0 20
    // @Increment: .1
    // @User: Standard
    GSCALAR(low_voltage,    "LOW_VOLT",         LOW_VOLTAGE),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Enable Super Simple Mode
    // @Description: Setting this to Enabled(1) will enable Super Simple Mode. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     SUPER_SIMPLE),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to -1 to disable, zero to land.
    // @Units: Centimeters
    // @Range: -1 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

	// @Param: TILT
    // @DisplayName: Auto Tilt Compensation
    // @Description: This is a feed-forward compensation which helps the aircraft achieve target waypoint speed.
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
	GSCALAR(tilt_comp,      "TILT",     TILT_COMPENSATION),

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

    // @Param: THR_ACC_ENABLE
    // @DisplayName: Enable Accel based throttle controller
    // @Description: This allows enabling and disabling the accelerometer based throttle controller.  If disabled a velocity based controller is used.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(throttle_accel_enabled,  "THR_ACC_ENABLE",   1),

    // @Param: YAW_OVR_BEHAVE
    // @DisplayName: Yaw override behaviour
    // @Description: Controls when autopilot takes back normal control of yaw after pilot overrides
    // @Values: 0:At Next WP, 1:On Mission Restart
    // @User: Advanced
    GSCALAR(yaw_override_behaviour,  "YAW_OVR_BEHAVE",   YAW_OVERRIDE_BEHAVIOUR_AT_NEXT_WAYPOINT),

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

    // @Param: RTL_LOITER_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before begining final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: Centimeters/Second
    // @Range: 10 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle that will be sent to the motors to keep them spinning
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,   "THR_MIN",          MINIMUM_THROTTLE),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle that will be sent to the motors
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_max,   "THR_MAX",          MAXIMUM_THROTTLE),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE", THROTTLE_FAILSAFE),

    GSCALAR(throttle_fs_action,     "THR_FS_ACTION",    THROTTLE_FAILSAFE_ACTION),

    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(throttle_fs_value, "THR_FS_VALUE",      THROTTLE_FS_VALUE),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle Trim
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(throttle_cruise,        "TRIM_THROTTLE",    THROTTLE_CRUISE),

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

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 2 byte bitmap of log types to enable
    // @User: Advanced
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: TOY_RATE
    // @DisplayName: Toy Yaw Rate
    // @Description: Controls yaw rate in Toy mode.  Higher values will cause a slower yaw rate.  Do not set to zero!
    // @User: Advanced
    // @Range: 1 10
    GSCALAR(toy_yaw_rate, "TOY_RATE",               1),

    // @Param: ESC
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up,1:Start-up in ESC Calibration mode
    GSCALAR(esc_calibrate, "ESC",                   0),

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

    // @Param: FRAME
    // @DisplayName: Frame Orientation
    // @Description: Congrols motor mixing The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(frame_orientation, "FRAME",             FRAME_ORIENTATION),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  CH7_OPTION),

	// @Param: AUTO_SLEW
    // @DisplayName: Auto Slew Rate
    // @Description: This restricts the rate of change of the roll and pitch attitude commanded by the auto pilot
    // @Units: Degrees/Second
	// @Range: 1 45
    // @Increment: 1
    // @User: Advanced
    GSCALAR(auto_slew_rate, "AUTO_SLEW",            AUTO_SLEW_RATE),

#if FRAME_CONFIG ==     HELI_FRAME
    GGROUP(heli_servo_1,    "HS1_", RC_Channel),
    GGROUP(heli_servo_2,    "HS2_", RC_Channel),
    GGROUP(heli_servo_3,    "HS3_", RC_Channel),
    GGROUP(heli_servo_4,    "HS4_", RC_Channel),
	GSCALAR(heli_pitch_ff, "H_PITCH_FF",            HELI_PITCH_FF),
	GSCALAR(heli_roll_ff, "H_ROLL_FF",            HELI_ROLL_FF),
	GSCALAR(heli_yaw_ff, "H_YAW_FF",            HELI_YAW_FF),
#endif

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GGROUP(camera,                  "CAM_", AP_Camera),
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

    // @Param: ACRO_P
    // @DisplayName: Acro P gain
    // @Description: Used to convert pilot roll, pitch and yaw input into a dssired rate of rotation in ACRO mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_p,                 "ACRO_P",           ACRO_P),

    // @Param: AXIS_ENABLE
    // @DisplayName: Acro Axis
    // @Description: Used to control whether acro mode actively maintains the current angle when control sticks are released (Enabled = maintains current angle)
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(axis_enabled,           "AXIS_ENABLE",      AXIS_LOCK_ENABLED),

    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),

    // @Param: LED_MODE
    // @DisplayName: Copter LED Mode
    // @Description: bitmap to control the copter led mode
    // @Values: 0:Disabled,1:Enable,2:GPS On,4:Aux,8:Buzzer,16:Oscillate,32:Nav Blink,64:GPS Nav Blink
    // @User: Standard
    GSCALAR(copter_leds_mode,       "LED_MODE",         9),

    // PID controller
    //---------------
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),


    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),
    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

    GGROUP(pid_nav_lat,             "NAV_LAT_",  AC_PID),
    GGROUP(pid_nav_lon,             "NAV_LON_",  AC_PID),

    GGROUP(pid_throttle,      "THR_RATE_", AC_PID),
    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

    GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),
    GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

    // PI controller
    //--------------
    GGROUP(pi_stabilize_roll,       "STB_RLL_", APM_PI),
    GGROUP(pi_stabilize_pitch,      "STB_PIT_", APM_PI),
    GGROUP(pi_stabilize_yaw,        "STB_YAW_", APM_PI),

    GGROUP(pi_alt_hold,             "THR_ALT_", APM_PI),
    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),
    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

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
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
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

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECT(motors, "H_",           AP_MotorsHeli),
#else
    GOBJECT(motors, "MOT_",         AP_Motors),
#endif

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
