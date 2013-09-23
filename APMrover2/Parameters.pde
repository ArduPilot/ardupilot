/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  ArduPlane parameter definitions
*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value:def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info:class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info:class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION",   1),
	GSCALAR(software_type,          "SYSID_SW_TYPE",    Parameters::k_software_type),

	// misc
    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Two byte bitmap of log types to enable in dataflash
    // @Values: 0:Disabled,3950:Default,4078:Default+IMU
    // @User: Advanced
	GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),
	GSCALAR(num_resets,             "SYS_NUM_RESETS",   0),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode	after geofence takeover.
    // @User: Advanced
	GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial driving mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver. Usuallly used in combination with when AUTO_TRIGGER_PIN or AUTO_KICKSTART.
    // @Values: 0:MANUAL,2:LEARNING,3:STEERING,4:HOLD,10:AUTO,11:RTL,15:GUIDED
    // @User: Advanced
	GSCALAR(initial_mode,        "INITIAL_MODE",     MANUAL),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:Pixhawk, 13:A13, 100:PX4
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    1),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk by the PM connector it should be set to 3.
    // @Values: -1:Disabled, 1:A1, 2:A2, 3:Pixhawk, 12:A12, 101:PX4
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    2),

    // @Param: SYSID_THIS_MAV
    // @DisplayName: MAVLink system ID
    // @Description: ID used in MAVLink protocol to identify this vehicle
    // @User: Advanced
	GSCALAR(sysid_this_mav,         "SYSID_THISMAV",    MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: MAVLink ground station ID
    // @Description: ID used in MAVLink protocol to identify the controlling ground station
    // @User: Advanced
	GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",      255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: USB Console Baud Rate
    // @Description: The baud rate used on the first serial port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
	GSCALAR(serial0_baud,           "SERIAL0_BAUD",     SERIAL0_BAUD/1000),

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

    // @Param: MAG_ENABLED
    // @DisplayName: Magnetometer (compass) enabled
    // @Description: This should be set to 1 if a compass is installed
    // @User: Standard
    // @Values: 0:Disabled,1:Enabled
	GSCALAR(compass_enabled,        "MAG_ENABLE",       MAGNETOMETER),

    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
	GSCALAR(battery_monitoring,     "BATT_MONITOR",     DISABLED),

    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_DIVIDER). For the 3DR Power brick on APM2 or Pixhawk this should be set to 10.1. For the PX4 using the PX4IO power supply this should be set to 1.
    // @User: Advanced
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",     VOLT_DIV_RATIO),

    // @Param: AMP_PER_VOLT
    // @DisplayName: Current Amps per volt
    // @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps. For the 3DR Power brick on APM2 or Pixhawk this should be set to 17
    // @User: Advanced
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",     CURR_AMP_PER_VOLT),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
	// @User: Standard
	GSCALAR(pack_capacity,          "BATT_CAPACITY",    HIGH_DISCHARGE),

	// @Param: AUTO_TRIGGER_PIN
	// @DisplayName: Auto mode trigger pin
	// @Description: pin number to use to enable the throttle in auto mode. If set to -1 then don't use a trigger, otherwise this is a pin number which if held low in auto mode will enable the motor to run. If the switch is released while in AUTO then the motor will stop again. This can be used in combination with INITIAL_MODE to give a 'press button to start' rover with no receiver.
	// @Values: -1:Disabled,0-8:TiggerPin
	// @User: standard
	GSCALAR(auto_trigger_pin,        "AUTO_TRIGGER_PIN", -1),

	// @Param: AUTO_KICKSTART
	// @DisplayName: Auto mode trigger kickstart acceleration
	// @Description: X acceleration in meters/second/second to use to trigger the motor start in auto mode. If set to zero then auto throttle starts immediately when the mode switch happens, otherwise the rover waits for the X acceleration to go above this value before it will start the motor
	// @Units: m/s/s
	// @Range: 0 20
	// @Increment: 0.1
	// @User: standard
	GSCALAR(auto_kickstart,          "AUTO_KICKSTART", 0.0f),

    // @Param: CRUISE_SPEED
    // @DisplayName: Target cruise speed in auto modes
    // @Description: The target speed in auto missions.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(speed_cruise,        "CRUISE_SPEED",    5),

    // @Param: SPEED_TURN_GAIN
    // @DisplayName: Target speed reduction while turning
    // @Description: The percentage to reduce the throttle while turning. If this is 100% then the target speed is not reduced while turning. If this is 50% then the target speed is reduced in proportion to the turn rate, with a reduction of 50% when the steering is maximally deflected.
    // @Units: percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(speed_turn_gain,    "SPEED_TURN_GAIN",  50),

    // @Param: SPEED_TURN_DIST
    // @DisplayName: Distance to turn to start reducing speed
    // @Description: The distance to the next turn at which the rover reduces its target speed by the SPEED_TURN_GAIN
    // @Units: meters
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(speed_turn_dist,    "SPEED_TURN_DIST",  2.0f),

    // @Param: CH7_OPTION
    // @DisplayName: Channel 7 option
    // @Description: What to do use channel 7 for
    // @Values: 0:Nothing,1:LearnWaypoint
    // @User: Standard
	GSCALAR(ch7_option,             "CH7_OPTION",          CH7_OPTION),

    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_1,                    "RC1_", RC_Channel),

    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_2,                    "RC2_", RC_Channel_aux),

    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_3,                    "RC3_", RC_Channel),

    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_4,                    "RC4_", RC_Channel_aux),

    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_5,                    "RC5_", RC_Channel_aux),

    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_6,                    "RC6_", RC_Channel_aux),

    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_7,                    "RC7_", RC_Channel_aux),

    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
	GGROUP(rc_8,                    "RC8_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                    "RC12_", RC_Channel_aux),
#endif

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply. This is mostly useful for rovers with internal combustion motors, to prevent the motor from cutting out in auto mode.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_min,           "THR_MIN",          THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply. This can be used to prevent overheating a ESC or motor on an electric rover.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_max,           "THR_MAX",          THROTTLE_MAX),

    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode. The CRUISE_SPEED parameter controls the target speed, but the rover starts with the CRUISE_THROTTLE setting as the initial estimate for how much throttle is needed to achieve that speed. It then adjusts the throttle based on how fast the rover is actually going.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_cruise,        "CRUISE_THROTTLE",    50),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: maximum percentage change in throttle per second. A setting of 10 means to not change the throttle by more than 10% of the full throttle range in one second. A value of zero means no limit.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
	GSCALAR(throttle_slewrate,      "THR_SLEWRATE",     0),

    // @Param: SKID_STEER_OUT
    // @DisplayName: Skid steering output
    // @Description: Set this to 1 for skid steering controlled rovers (tank track style). When enabled, servo1 is used for the left track control, servo3 is used for right track control
    // @Values: 0:Disabled, 1:SkidSteeringOutput
    // @User: Standard
	GSCALAR(skid_steer_out,          "SKID_STEER_OUT",     0),

    // @Param: SKID_STEER_IN
    // @DisplayName: Skid steering input
    // @Description: Set this to 1 for skid steering input rovers (tank track style in RC controller). When enabled, servo1 is used for the left track control, servo3 is used for right track control
    // @Values: 0:Disabled, 1:SkidSteeringInput
    // @User: Standard
	GSCALAR(skid_steer_in,           "SKID_STEER_IN",     0),

    // @Param: FS_ACTION
    // @DisplayName: Failsafe Action
    // @Description: What to do on a failsafe event
    // @Values: 0:Nothing,1:RTL,2:HOLD
    // @User: Standard
	GSCALAR(fs_action,    "FS_ACTION",     2),

    // @Param: FS_TIMEOUT
    // @DisplayName: Failsafe timeout
    // @Description: How long a failsafe event need to happen for before we trigger the failsafe action
	// @Units: seconds
    // @User: Standard
	GSCALAR(fs_timeout,    "FS_TIMEOUT",     5),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel to a low value. This can be used to detect the RC transmitter going out of range. Failsafe will be triggered when the throttle channel goes below the FS_THR_VALUE for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(fs_throttle_enabled,    "FS_THR_ENABLE",     1),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers.
    // @Range: 925 1100
    // @Increment: 1
    // @User: Standard
	GSCALAR(fs_throttle_value,      "FS_THR_VALUE",     910),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. When enabled the Rover will execute the FS_ACTION when it fails to receive MAVLink heartbeat packets for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
	GSCALAR(fs_gcs_enabled, "FS_GCS_ENABLE",   0),

	// @Param: SONAR_TRIGGER_CM
	// @DisplayName: Sonar trigger distance
	// @Description: The distance from an obstacle in centimeters at which the sonar triggers a turn to avoid the obstacle
	// @Units: centimeters
    // @Range: 0 1000
    // @Increment: 1
	// @User: Standard
	GSCALAR(sonar_trigger_cm,   "SONAR_TRIGGER_CM",    100),

	// @Param: SONAR_TURN_ANGLE
	// @DisplayName: Sonar trigger angle
	// @Description: The course deviation in degrees to apply while avoiding an obstacle detected with the sonar. A positive number means to turn right, and a negative angle means to turn left.
	// @Units: centimeters
    // @Range: -45 45
    // @Increment: 1
	// @User: Standard
	GSCALAR(sonar_turn_angle,   "SONAR_TURN_ANGLE",    45),

	// @Param: SONAR_TURN_TIME
	// @DisplayName: Sonar turn time
	// @Description: The amount of time in seconds to apply the SONAR_TURN_ANGLE after detecting an obstacle.
	// @Units: seconds
    // @Range: 0 100
    // @Increment: 0.1
	// @User: Standard
	GSCALAR(sonar_turn_time,    "SONAR_TURN_TIME",     1.0f),

	// @Param: SONAR_DEBOUNCE
	// @DisplayName: Sonar debounce count
	// @Description: The number of 50Hz sonar hits needed to trigger an obstacle avoidance event. If you get a lot of false sonar events then raise this number, but if you make it too large then it will cause lag in detecting obstacles, which could cause you go hit the obstacle.
    // @Range: 1 100
    // @Increment: 1
	// @User: Standard
	GSCALAR(sonar_debounce,   "SONAR_DEBOUNCE",    2),

    // @Param: LEARN_CH
    // @DisplayName: Learning channel
    // @Description: RC Channel to use for learning waypoints
    // @User: Advanced
	GSCALAR(learn_channel,    "LEARN_CH",       7),

    // @Param: MODE_CH
    // @DisplayName: Mode channel
    // @Description: RC Channel to use for driving mode control
    // @User: Advanced
	GSCALAR(mode_channel,    "MODE_CH",       MODE_CHANNEL),

    // @Param: MODE1
    // @DisplayName: Mode1
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
    // @Description: Driving mode for switch position 1 (910 to 1230 and above 2049)
	GSCALAR(mode1,           "MODE1",         MODE_1),

    // @Param: MODE2
    // @DisplayName: Mode2
    // @Description: Driving mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode2,           "MODE2",         MODE_2),

    // @Param: MODE3
    // @DisplayName: Mode3
    // @Description: Driving mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode3,           "MODE3",         MODE_3),

    // @Param: MODE4
    // @DisplayName: Mode4
    // @Description: Driving mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode4,           "MODE4",         MODE_4),

    // @Param: MODE5
    // @DisplayName: Mode5
    // @Description: Driving mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode5,           "MODE5",         MODE_5),

    // @Param: MODE6
    // @DisplayName: Mode6
    // @Description: Driving mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,2:LEARNING,3:STEERING,4:HOLD,10:Auto,11:RTL,15:Guided
    // @User: Standard
	GSCALAR(mode6,           "MODE6",         MODE_6),

	GSCALAR(command_total,          "CMD_TOTAL",        0),
	GSCALAR(command_index,          "CMD_INDEX",        0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint radius
    // @Description: The distance in meters from a waypoint when we consider the waypoint has been reached. This determines when the rover will turn along the next waypoint path.
    // @Units: meters
    // @Range: 0 1000
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(waypoint_radius,        "WP_RADIUS",        2.0f),

    // @Param: TURN_MAX_G
    // @DisplayName: Turning maximum G force
    // @Description: The maximum turning acceleration (in units of gravities) that the rover can handle while remaining stable. The navigation code will keep the lateral acceleration below this level to avoid rolling over or slipping the wheels in turns
    // @Units: gravities
    // @Range: 0.2 10
    // @Increment: 0.1
    // @User: Standard
	GSCALAR(turn_max_g,             "TURN_MAX_G",      2.0f),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),

	GGROUP(pidSpeedThrottle,        "SPEED2THR_", PID),

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
	GOBJECT(compass,                "COMPASS_",	Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                 "RCMAP_",         RCMapper),

    // @Group: SR0_
    // @Path: GCS_Mavlink.pde
	GOBJECT(gcs0,					"SR0_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: GCS_Mavlink.pde
	GOBJECT(gcs3,					"SR3_",     GCS_MAVLINK),

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: SONAR_
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp
    GOBJECT(sonar,                  "SONAR_", AP_RangeFinder_analog),

    // @Group: SONAR2_
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder_analog.cpp
    GOBJECT(sonar2,                 "SONAR2_", AP_RangeFinder_analog),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                            "INS_", AP_InertialSensor),

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,                  "CAM_", AP_Camera),
#endif

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

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

    // set a lower default filter frequency for rovers, due to very
    // high vibration levels on rough surfaces
    ins.set_default_filter(5);

    // set a more reasonable default NAVL1_PERIOD for rovers
    L1_controller.set_default_period(8);
}
