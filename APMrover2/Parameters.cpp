#include "Rover.h"

/*
  APMRover2 parameter definitions
*/

#define GSCALAR(v, name, def) { rover.g.v.vtype, name, Parameters::k_param_ ## v, &rover.g.v, {def_value:def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &rover.g.v, {group_info:class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &rover.v, {group_info:class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &rover.v, {group_info : class::var_info} }

const AP_Param::Info Rover::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION",   1),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @User: Advanced
    // @ReadOnly: True
    GSCALAR(software_type,          "SYSID_SW_TYPE",    Parameters::k_software_type),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in dataflash. This values is made up of the sum of each of the log types you want to be saved on dataflash. On a PX4 or Pixhawk the large storage size of a microSD card means it is usually best just to enable all log types by setting this to 65535. On APM2 the smaller 4 MByte dataflash means you need to be more selective in your logging or you may run out of log space while flying (in which case it will wrap and overwrite the start of the log). The individual bits are ATTITUDE_FAST=1, ATTITUDE_MEDIUM=2, GPS=4, PerformanceMonitoring=8, ControlTuning=16, NavigationTuning=32, Mode=64, IMU=128, Commands=256, Battery=512, Compass=1024, TECS=2048, Camera=4096, RCandServo=8192, Rangefinder=16384, Arming=32768, FullLogs=65535
    // @Values: 0:Disabled,5190:APM2-Default,65535:PX4/Pixhawk-Default
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:MODE,7:IMU,8:CMD,9:CURRENT,10:COMPASS,11:TECS,12:CAMERA,13:RC,14:RANGEFINDER,15:ARM/DISARM,19:IMU_RAW
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),

    // @Param: SYS_NUM_RESETS
    // @DisplayName: Num Resets
    // @Description: Number of APM board resets
    // @ReadOnly: True
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS",   0),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial driving mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver. Usually used in combination with when AUTO_TRIGGER_PIN or AUTO_KICKSTART.
    // @Values: 0:MANUAL,2:LEARNING,3:STEERING,4:HOLD,10:AUTO,11:RTL,15:GUIDED
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     MANUAL),

    // @Param: SYSID_THIS_MAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",    MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: MAVLink ground station ID
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",      255),

#if CLI_ENABLED == ENABLED
    // @Param: CLI_ENABLED
    // @DisplayName: CLI Enable
    // @Description: This enables/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(cli_enabled,            "CLI_ENABLED",    0),
#endif

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Values: 0:None,1:Steering,2:Throttle
    // @Bitmask: 0:Steering,1:Throttle
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass. Note that this is separate from COMPASS_USE. This will enable the low level senor, and will enable logging of magnetometer data. To use the compass for navigation you must also set COMPASS_USE to 1.
    // @User: Standard
    // @Values: 0:Disabled,1:Enabled
    GSCALAR(compass_enabled,        "MAG_ENABLE",       MAGNETOMETER),

    // @Param: AUTO_TRIGGER_PIN
    // @DisplayName: Auto mode trigger pin
    // @Description: pin number to use to enable the throttle in auto mode. If set to -1 then don't use a trigger, otherwise this is a pin number which if held low in auto mode will enable the motor to run. If the switch is released while in AUTO then the motor will stop again. This can be used in combination with INITIAL_MODE to give a 'press button to start' rover with no receiver.
    // @Values: -1:Disabled,0:APM TriggerPin0,1:APM TriggerPin1,2:APM TriggerPin2,3:APM TriggerPin3,4:APM TriggerPin4,5:APM TriggerPin5,6:APM TriggerPin6,7:APM TriggerPin7,8:APM TriggerPin8,50:Pixhawk TriggerPin50,51:Pixhawk TriggerPin51,52:Pixhawk TriggerPin52,53:Pixhawk TriggerPin53,54:Pixhawk TriggerPin54,55:Pixhawk TriggerPin55
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
    GSCALAR(speed_cruise,        "CRUISE_SPEED",    SPEED_CRUISE),

    // @Param: SPEED_TURN_GAIN
    // @DisplayName: Target speed reduction while turning
    // @Description: The percentage to reduce the throttle to while turning. If this is 100% then the target speed is not reduced while turning. If this is 50% then the target speed is reduced in proportion to the turn rate, with a reduction of 50% when the steering is maximally deflected.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(speed_turn_gain,    "SPEED_TURN_GAIN",  50),

    // @Param: SPEED_TURN_DIST
    // @DisplayName: Distance to turn to start reducing speed
    // @Description: The distance to the next turn at which the rover reduces its target speed by the SPEED_TURN_GAIN
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(speed_turn_dist,    "SPEED_TURN_DIST",  2.0f),

    // @Param: BRAKING_PERCENT
    // @DisplayName: Percentage braking to apply
    // @Description: The maximum reverse throttle braking percentage to apply when cornering
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(braking_percent,    "BRAKING_PERCENT",  0),

    // @Param: BRAKING_SPEEDERR
    // @DisplayName: Speed error at which to apply braking
    // @Description: The amount of overspeed error at which to start applying braking
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(braking_speederr,   "BRAKING_SPEEDERR",  3),

    // @Param: PIVOT_TURN_ANGLE
    // @DisplayName: Pivot turn angle
    // @Description: Navigation angle threshold in degrees to switch to pivot steering when SKID_STEER_OUT is 1. This allows you to setup a skid steering rover to turn on the spot in auto mode when the angle it needs to turn it greater than this angle. An angle of zero means to disable pivot turning. Note that you will probably also want to set a low value for WP_RADIUS to get neat turns.
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    GSCALAR(pivot_turn_angle,   "PIVOT_TURN_ANGLE",  30),

    // @Param: CH7_OPTION
    // @DisplayName: Channel 7 option
    // @Description: What to do use channel 7 for
    // @Values: 0:Nothing,1:LearnWaypoint
    // @User: Standard
    GSCALAR(ch7_option,             "CH7_OPTION",          CH7_OPTION),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply. This is mostly useful for rovers with internal combustion motors, to prevent the motor from cutting out in auto mode.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,           "THR_MIN",          THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply. This can be used to prevent overheating a ESC or motor on an electric rover.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_max,           "THR_MAX",          THROTTLE_MAX),

    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode. The CRUISE_SPEED parameter controls the target speed, but the rover starts with the CRUISE_THROTTLE setting as the initial estimate for how much throttle is needed to achieve that speed. It then adjusts the throttle based on how fast the rover is actually going.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_cruise,        "CRUISE_THROTTLE",    50),

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
    // @Units: s
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
    // @Description: The PWM level on the throttle channel below which throttle failsafe triggers.
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

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check action
    // @Description: What to do on a crash event. When enabled the rover will go to hold if a crash is detected.
    // @Values: 0:Disabled,1:HOLD,2:HoldAndDisarm
    // @User: Standard
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    FS_CRASH_DISABLE),

    // @Param: RNGFND_TRIGGR_CM
    // @DisplayName: Rangefinder trigger distance
    // @Description: The distance from an obstacle in centimeters at which the rangefinder triggers a turn to avoid the obstacle
    // @Units: cm
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rangefinder_trigger_cm,   "RNGFND_TRIGGR_CM",    100),

    // @Param: RNGFND_TURN_ANGL
    // @DisplayName: Rangefinder trigger angle
    // @Description: The course deviation in degrees to apply while avoiding an obstacle detected with the rangefinder. A positive number means to turn right, and a negative angle means to turn left.
    // @Units: deg
    // @Range: -45 45
    // @Increment: 1
    // @User: Standard
    GSCALAR(rangefinder_turn_angle,   "RNGFND_TURN_ANGL",    45),

    // @Param: RNGFND_TURN_TIME
    // @DisplayName: Rangefinder turn time
    // @Description: The amount of time in seconds to apply the RNGFND_TURN_ANGL after detecting an obstacle.
    // @Units: s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(rangefinder_turn_time,    "RNGFND_TURN_TIME",     1.0f),

    // @Param: RNGFND_DEBOUNCE
    // @DisplayName: Rangefinder debounce count
    // @Description: The number of 50Hz rangefinder hits needed to trigger an obstacle avoidance event. If you get a lot of false rangefinder events then raise this number, but if you make it too large then it will cause lag in detecting obstacles, which could cause you go hit the obstacle.
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(rangefinder_debounce,   "RNGFND_DEBOUNCE",    2),

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

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint radius
    // @Description: The distance in meters from a waypoint when we consider the waypoint has been reached. This determines when the rover will turn along the next waypoint path.
    // @Units: m
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

    // @Group: SPEED2THR_
    // @Path: ../libraries/PID/PID.cpp
    GGROUP(pidSpeedThrottle,        "SPEED2THR_", PID),

    // variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,                "COMPASS_", Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                 "RCMAP_",         RCMapper),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[0], gcs0,        "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[1],  gcs1,       "SR1_",     GCS_MAVLINK),

    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[2],  gcs2,       "SR2_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[3],  gcs3,       "SR3_",     GCS_MAVLINK),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager,         "SERIAL",   AP_SerialManager),

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/RangeFinder.cpp
    GOBJECT(rangefinder,                 "RNGFND", RangeFinder),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                            "INS_", AP_InertialSensor),

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SITL),
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
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming),

    // @Group: LOG
    // @Path: ../libraries/DataFlash/DataFlash.cpp
    GOBJECT(DataFlash,           "LOG",  DataFlash_Class),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT", AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_WITH_UAVCAN
    // @Group: CAN_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig_CAN.cpp
    GOBJECT(BoardConfig_CAN,        "CAN_",       AP_BoardConfig_CAN),
#endif

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

    #if AP_AHRS_NAVEKF_AVAILABLE
        // @Group: EK2_
        // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
        GOBJECTN(EKF2, NavEKF2, "EK2_", NavEKF2),

        // @Group: EK3_
        // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
        GOBJECTN(EKF3, NavEKF3, "EK3_", NavEKF3),
    #endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    GOBJECT(button, "BTN_",  AP_Button),

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // @Group: STAT
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 1, ParametersG2, AP_Stats),

    // @Param: SYSID_ENFORCE
    // @DisplayName: GCS sysid enforcement
    // @Description: This controls whether packets from other than the expected GCS system ID will be accepted
    // @Values: 0:NotEnforced,1:Enforced
    // @User: Advanced
    AP_GROUPINFO("SYSID_ENFORCE", 2, ParametersG2, sysid_enforce, 0),

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 3, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels.cpp
    AP_SUBGROUPINFO(rc_channels, "RC", 4, ParametersG2, RC_Channels),

#if ADVANCED_FAILSAFE == ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    AP_SUBGROUPINFO(afs, "AFS_", 5, ParametersG2, AP_AdvancedFailsafe),
#endif

    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 6, ParametersG2, AP_Beacon),

    // @Group: VISO
    // @Path: ../libraries/AP_VisualOdom/AP_VisualOdom.cpp
    AP_SUBGROUPINFO(visual_odom, "VISO", 7, ParametersG2, AP_VisualOdom),

    // @Group: MOT_
    // @Path: AP_MotorsUGV.cpp
    AP_SUBGROUPINFO(motors, "MOT_", 8, ParametersG2, AP_MotorsUGV),

    // @Group: WENC_
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
    AP_SUBGROUPINFO(wheel_encoder, "WENC_", 9, ParametersG2, AP_WheelEncoder),

    AP_GROUPEND
};


ParametersG2::ParametersG2(void)
    : beacon(rover.serial_manager),
      motors(rover.ServoRelayEvents)
#if ADVANCED_FAILSAFE == ENABLED
    , afs(rover.mission, rover.barometer, rover.gps, rover.rcmap)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}


/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] = {
    { Parameters::k_param_battery_monitoring, 0,      AP_PARAM_INT8,  "BATT_MONITOR" },
    { Parameters::k_param_battery_volt_pin,   0,      AP_PARAM_INT8,  "BATT_VOLT_PIN" },
    { Parameters::k_param_battery_curr_pin,   0,      AP_PARAM_INT8,  "BATT_CURR_PIN" },
    { Parameters::k_param_volt_div_ratio,     0,      AP_PARAM_FLOAT, "BATT_VOLT_MULT" },
    { Parameters::k_param_curr_amp_per_volt,  0,      AP_PARAM_FLOAT, "BATT_AMP_PERVOLT" },
    { Parameters::k_param_pack_capacity,      0,      AP_PARAM_INT32, "BATT_CAPACITY" },
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },
};

void Rover::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        cliSerial->printf("Bad var table\n");
        AP_HAL::panic("Bad var table");
    }

    if (!g.format_version.load() ||
         g.format_version != Parameters::k_format_version) {
        // erase all parameters
        cliSerial->printf("Firmware change: erasing EEPROM...\n");
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        cliSerial->printf("done.\n");
    }

    const uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROVER);

    SRV_Channels::set_default_function(CH_1, SRV_Channel::k_steering);
    SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttle);

    const uint8_t old_rc_keys[14] = { Parameters::k_param_rc_1_old,  Parameters::k_param_rc_2_old,
                                      Parameters::k_param_rc_3_old,  Parameters::k_param_rc_4_old,
                                      Parameters::k_param_rc_5_old,  Parameters::k_param_rc_6_old,
                                      Parameters::k_param_rc_7_old,  Parameters::k_param_rc_8_old,
                                      Parameters::k_param_rc_9_old,  Parameters::k_param_rc_10_old,
                                      Parameters::k_param_rc_11_old, Parameters::k_param_rc_12_old,
                                      Parameters::k_param_rc_13_old, Parameters::k_param_rc_14_old };
    const uint16_t old_aux_chan_mask = 0x3FFA;
    SRV_Channels::upgrade_parameters(old_rc_keys, old_aux_chan_mask, &rcmap);
    cliSerial->printf("load_all took %uus\n", micros() - before);
    // set a more reasonable default NAVL1_PERIOD for rovers
    L1_controller.set_default_period(NAVL1_PERIOD);
}
