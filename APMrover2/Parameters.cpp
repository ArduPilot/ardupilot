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

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in on-board logger. This value is made up of the sum of each of the log types you want to be saved. On boards supporting microSD cards or other large block-storage devices it is usually best just to enable all log types by setting this to 65535. The individual bits are ATTITUDE_FAST=1, ATTITUDE_MEDIUM=2, GPS=4, PerformanceMonitoring=8, ControlTuning=16, NavigationTuning=32, Mode=64, IMU=128, Commands=256, Battery=512, Compass=1024, TECS=2048, Camera=4096, RCandServo=8192, Rangefinder=16384, Arming=32768, FullLogs=65535
    // @Values: 0:Disabled,65535:Default
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:THR,5:NTUN,7:IMU,8:CMD,9:CURRENT,10:RANGEFINDER,11:COMPASS,12:CAMERA,13:STEERING,14:RC,15:ARM/DISARM,19:IMU_RAW
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial driving mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver. Usually used in combination with when AUTO_TRIGGER_PIN or AUTO_KICKSTART.
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     Mode::Number::MANUAL),

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
    // @Values: 0:None,1:Steering,2:Throttle,4:Pitch,8:Left Wheel,16:Right Wheel,32:Sailboat Heel
    // @Bitmask: 0:Steering,1:Throttle,2:Pitch,3:Left Wheel,4:Right Wheel,5:Sailboat Heel
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: AUTO_TRIGGER_PIN
    // @DisplayName: Auto mode trigger pin
    // @Description: pin number to use to enable the throttle in auto mode. If set to -1 then don't use a trigger, otherwise this is a pin number which if held low in auto mode will enable the motor to run. If the switch is released while in AUTO then the motor will stop again. This can be used in combination with INITIAL_MODE to give a 'press button to start' rover with no receiver.
    // @Values: -1:Disabled,0:APM TriggerPin0,1:APM TriggerPin1,2:APM TriggerPin2,3:APM TriggerPin3,4:APM TriggerPin4,5:APM TriggerPin5,6:APM TriggerPin6,7:APM TriggerPin7,8:APM TriggerPin8,50:Pixhawk TriggerPin50,51:Pixhawk TriggerPin51,52:Pixhawk TriggerPin52,53:Pixhawk TriggerPin53,54:Pixhawk TriggerPin54,55:Pixhawk TriggerPin55
    // @User: Standard
    GSCALAR(auto_trigger_pin,        "AUTO_TRIGGER_PIN", -1),

    // @Param: AUTO_KICKSTART
    // @DisplayName: Auto mode trigger kickstart acceleration
    // @Description: X acceleration in meters/second/second to use to trigger the motor start in auto mode. If set to zero then auto throttle starts immediately when the mode switch happens, otherwise the rover waits for the X acceleration to go above this value before it will start the motor
    // @Units: m/s/s
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(auto_kickstart,          "AUTO_KICKSTART", 0.0f),

    // @Param: CRUISE_SPEED
    // @DisplayName: Target cruise speed in auto modes
    // @Description: The target speed in auto missions.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(speed_cruise,        "CRUISE_SPEED",    CRUISE_SPEED),


    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode. The CRUISE_SPEED parameter controls the target speed, but the rover starts with the CRUISE_THROTTLE setting as the initial estimate for how much throttle is needed to achieve that speed. It then adjusts the throttle based on how fast the rover is actually going.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_cruise,        "CRUISE_THROTTLE",    50),

    // @Param: PILOT_STEER_TYPE
    // @DisplayName: Pilot input steering type
    // @Description: Set this to 1 for skid steering input rovers (tank track style in RC controller). When enabled, servo1 is used for the left track control, servo3 is used for right track control
    // @Values: 0:Default,1:Two Paddles Input,2:Direction reversed when backing up,3:Direction unchanged when backing up
    // @User: Standard
    GSCALAR(pilot_steer_type, "PILOT_STEER_TYPE", 0),

    // @Param: FS_ACTION
    // @DisplayName: Failsafe Action
    // @Description: What to do on a failsafe event
    // @Values: 0:Nothing,1:RTL,2:Hold,3:SmartRTL or RTL,4:SmartRTL or Hold
    // @User: Standard
    GSCALAR(fs_action,    "FS_ACTION",     Failsafe_Action_Hold),

    // @Param: FS_TIMEOUT
    // @DisplayName: Failsafe timeout
    // @Description: The time in seconds that a failsafe condition must persist before the failsafe action is triggered
    // @Units: s
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout,    "FS_TIMEOUT",     1.5),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel to a low value. This can be used to detect the RC transmitter going out of range. Failsafe will be triggered when the throttle channel goes below the FS_THR_VALUE for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled,2:Enabled Continue with Mission in Auto
    // @User: Standard
    GSCALAR(fs_throttle_enabled,    "FS_THR_ENABLE",     FS_THR_ENABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on the throttle channel below which throttle failsafe triggers.
    // @Range: 910 1100
    // @Increment: 1
    // @User: Standard
    GSCALAR(fs_throttle_value,      "FS_THR_VALUE",     910),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. When enabled the Rover will execute the FS_ACTION when it fails to receive MAVLink heartbeat packets for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled,2:Enabled Continue with Mission in Auto
    // @User: Standard
    GSCALAR(fs_gcs_enabled, "FS_GCS_ENABLE",   FS_GCS_DISABLED),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check action
    // @Description: What to do on a crash event. When enabled the rover will go to hold if a crash is detected.
    // @Values: 0:Disabled,1:Hold,2:HoldAndDisarm
    // @User: Standard
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    FS_CRASH_DISABLE),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 0:Disabled,1:Hold
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION", 1),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH", 0.8f),

    // @Param: MODE_CH
    // @DisplayName: Mode channel
    // @Description: RC Channel to use for driving mode control
    // @User: Advanced
    GSCALAR(mode_channel,    "MODE_CH",       MODE_CHANNEL),

    // @Param: MODE1
    // @DisplayName: Mode1
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    // @Description: Driving mode for switch position 1 (910 to 1230 and above 2049)
    GSCALAR(mode1,           "MODE1",         Mode::Number::MANUAL),

    // @Param: MODE2
    // @DisplayName: Mode2
    // @Description: Driving mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    GSCALAR(mode2,           "MODE2",         Mode::Number::MANUAL),

    // @Param: MODE3
    // @DisplayName: Mode3
    // @Description: Driving mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    GSCALAR(mode3,           "MODE3",         Mode::Number::MANUAL),

    // @Param: MODE4
    // @DisplayName: Mode4
    // @Description: Driving mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    GSCALAR(mode4,           "MODE4",         Mode::Number::MANUAL),

    // @Param: MODE5
    // @DisplayName: Mode5
    // @Description: Driving mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    GSCALAR(mode5,           "MODE5",         Mode::Number::MANUAL),

    // @Param: MODE6
    // @DisplayName: Mode6
    // @Description: Driving mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    GSCALAR(mode6,           "MODE6",         Mode::Number::MANUAL),

    // @Param: TURN_MAX_G
    // @DisplayName: Turning maximum G force
    // @Description: The maximum turning acceleration (in units of gravities) that the rover can handle while remaining stable. The navigation code will keep the lateral acceleration below this level to avoid rolling over or slipping the wheels in turns
    // @Units: gravities
    // @Range: 0.1 10
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(turn_max_g,             "TURN_MAX_G",      0.6f),

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
    GOBJECTN(_gcs.chan_parameters[0], gcs0,        "SR0_",     GCS_MAVLINK_Parameters),

    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[1],  gcs1,       "SR1_",     GCS_MAVLINK_Parameters),

    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[2],  gcs2,       "SR2_",     GCS_MAVLINK_Parameters),

    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[3],  gcs3,       "SR3_",     GCS_MAVLINK_Parameters),

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
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger,           "LOG",  AP_Logger),

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

    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECTN(mode_auto.mission, mission, "MIS_", AP_Mission),

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

#if OSD_ENABLED == ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {
#if STATS_ENABLED == ENABLED
    // @Group: STAT
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 1, ParametersG2, AP_Stats),
#endif
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
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 4, ParametersG2, RC_Channels_Rover),

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

    // @Group: WENC
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
    AP_SUBGROUPINFO(wheel_encoder, "WENC", 9, ParametersG2, AP_WheelEncoder),

    // @Group: ATC
    // @Path: ../libraries/APM_Control/AR_AttitudeControl.cpp
    AP_SUBGROUPINFO(attitude_control, "ATC", 10, ParametersG2, AR_AttitudeControl),

    // @Param: TURN_RADIUS
    // @DisplayName: Turn radius of vehicle
    // @Description: Turn radius of vehicle in meters while at low speeds.  Lower values produce tighter turns in steering mode
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TURN_RADIUS", 11, ParametersG2, turn_radius, 0.9),

    // @Param: ACRO_TURN_RATE
    // @DisplayName: Acro mode turn rate maximum
    // @Description: Acro mode turn rate maximum
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_TURN_RATE", 12, ParametersG2, acro_turn_rate, 180.0f),

    // @Group: SRTL_
    // @Path: ../libraries/AP_SmartRTL/AP_SmartRTL.cpp
    AP_SUBGROUPINFO(smart_rtl, "SRTL_", 13, ParametersG2, AP_SmartRTL),

    // 14 was WP_SPEED and should not be re-used

    // @Param: RTL_SPEED
    // @DisplayName: Return-to-Launch speed default
    // @Description: Return-to-Launch speed default.  If zero use WP_SPEED or CRUISE_SPEED.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RTL_SPEED", 15, ParametersG2, rtl_speed, 0.0f),

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Frame Class
    // @Values: 0:Undefined,1:Rover,2:Boat,3:BalanceBot
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 16, ParametersG2, frame_class, 1),

    // @Group: FENCE_
    // @Path: ../libraries/AC_Fence/AC_Fence.cpp
    AP_SUBGROUPINFO(fence, "FENCE_", 17, ParametersG2, AC_Fence),

    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 18, ParametersG2, AP_Proximity),

    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
    AP_SUBGROUPINFO(avoid, "AVOID_", 19, ParametersG2, AC_Avoid),

    // 20 was PIVOT_TURN_RATE and should not be re-used

    // @Param: BAL_PITCH_MAX
    // @DisplayName: BalanceBot Maximum Pitch
    // @Description: Pitch angle in degrees at 100% throttle
    // @Units: deg
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BAL_PITCH_MAX", 21, ParametersG2, bal_pitch_max, 2),

    // @Param: CRASH_ANGLE
    // @DisplayName: Crash Angle
    // @Description: Pitch/Roll angle limit in degrees for crash check. Zero disables check
    // @Units: deg
    // @Range: 0 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRASH_ANGLE", 22, ParametersG2, crash_angle, 0),

    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 23, ParametersG2, AP_Follow),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type
    // @Description: Frame Type
    // @Values: 0:Undefined,1:Omni3,2:OmniX,3:OmniPlus
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_TYPE", 24, ParametersG2, frame_type, 0),

    // @Param: LOIT_TYPE
    // @DisplayName: Loiter type
    // @Description: Loiter behaviour when moving to the target point
    // @Values: 0:Forward or reverse to target point,1:Always face bow towards target point
    // @User: Standard
    AP_GROUPINFO("LOIT_TYPE", 25, ParametersG2, loit_type, 0),

    // @Group: SPRAYER_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    AP_SUBGROUPINFO(sprayer, "SPRAY_", 26, ParametersG2, AC_Sprayer),

    // @Group: WRC
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
    AP_SUBGROUPINFO(wheel_rate_control, "WRC", 27, ParametersG2, AP_WheelRateControl),

#if AP_RALLY == ENABLED
    // @Group: RALLY_
    // @Path: AP_Rally.cpp,../libraries/AP_Rally/AP_Rally.cpp
    AP_SUBGROUPINFO(rally, "RALLY_", 28, ParametersG2, AP_Rally_Rover),
#endif

    // @Param: SIMPLE_TYPE
    // @DisplayName: Simple_Type
    // @Description: Simple mode types
    // @Values: 0:InitialHeading,1:CardinalDirections
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SIMPLE_TYPE", 29, ParametersG2, simple_type, 0),

    // @Param: LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: Vehicle will drift when within this distance of the target position
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 30, ParametersG2, loit_radius, 2),

    // @Group: WNDVN_
    // @Path: ../libraries/AP_WindVane/AP_WindVane.cpp
    AP_SUBGROUPINFO(windvane, "WNDVN_", 31, ParametersG2, AP_WindVane),

    // 32 to 36 were old sailboat params

    // @Group: ARSPD
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    AP_SUBGROUPINFO(airspeed, "ARSPD", 37, ParametersG2, AP_Airspeed),

    // @Param: MIS_DONE_BEHAVE
    // @DisplayName: Mission done behave
    // @Description: Behaviour after mission completes
    // @Values: 0:Hold,1:Loiter,2:Acro
    // @User: Standard
    AP_GROUPINFO("MIS_DONE_BEHAVE", 38, ParametersG2, mis_done_behave, 0),

#if GRIPPER_ENABLED == ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 39, ParametersG2, AP_Gripper),
#endif

    // @Param: BAL_PITCH_TRIM
    // @DisplayName: Balance Bot pitch trim angle
    // @Description: Balance Bot pitch trim for balancing. This offsets the tilt of the center of mass.
    // @Units: deg
    // @Range: -2 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BAL_PITCH_TRIM", 40, ParametersG2, bal_pitch_trim, 0),

#ifdef ENABLE_SCRIPTING
    // @Group: SCR_
    // @Path: ../libraries/AP_Scripting/AP_Scripting.cpp
    AP_SUBGROUPINFO(scripting, "SCR_", 41, ParametersG2, AP_Scripting),
#endif

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds steering user stick input in auto modes, allowing the user to have some degree of control without changing modes.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("STICK_MIXING", 42, ParametersG2, stick_mixing, 0),

    // @Group: WP_
    // @Path: ../libraries/AR_WPNav/AR_WPNav.cpp
    AP_SUBGROUPINFO(wp_nav, "WP_", 43, ParametersG2, AR_WPNav),

    // @Group: SAIL_
    // @Path: sailboat.cpp
    AP_SUBGROUPINFO(sailboat, "SAIL_", 44, ParametersG2, Sailboat),

    // @Group: OA_
    // @Path: ../libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    AP_SUBGROUPINFO(oa, "OA_", 45, ParametersG2, AP_OAPathPlanner),

    // @Param: SPEED_MAX
    // @DisplayName: Speed maximum
    // @Description: Maximum speed vehicle can obtain at full throttle. If 0, it will be estimated based on CRUISE_SPEED and CRUISE_THROTTLE.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPEED_MAX", 46, ParametersG2, speed_max, 0.0f),

    // @Param: LOIT_SPEED_GAIN
    // @DisplayName: Loiter speed gain
    // @Description: Determines how agressively LOITER tries to correct for drift from loiter point. Higher is faster but default should be acceptable.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("LOIT_SPEED_GAIN", 47, ParametersG2, loiter_speed_gain, 0.5f),

    // @Param: FS_OPTIONS
    // @DisplayName: Rover Failsafe Options
    // @Description: Bitmask to enable Rover failsafe options
    // @Values: 0:None,1:Failsafe enabled in Hold mode
    // @Bitmask: 0:Failsafe enabled in Hold mode
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 48, ParametersG2, fs_options, 0),

    AP_GROUPEND
};

// These auxiliary channel param descriptions are here so that users of beta Mission Planner (which uses the master branch as its source of descriptions)
// can get them.  These lines can be removed once Rover-3.6-beta testing begins or we improve the source of descriptions for GCSs.
//
// @Param: CH7_OPTION
// @DisplayName: Channel 7 option
// @Description: What to do use channel 7 for
// @Values: 0:Nothing,1:SaveWaypoint,2:LearnCruiseSpeed,3:ArmDisarm,4:Manual,5:Acro,6:Steering,7:Hold,8:Auto,9:RTL,10:SmartRTL,11:Guided,12:Loiter
// @User: Standard

// @Param: AUX_CH
// @DisplayName: Auxiliary switch channel
// @Description: RC Channel to use for auxiliary functions including saving waypoints
// @User: Advanced

// @Param: PIVOT_TURN_ANGLE
// @DisplayName: Pivot turn angle
// @Description: Navigation angle threshold in degrees to switch to pivot steering when SKID_STEER_OUT is 1. This allows you to setup a skid steering rover to turn on the spot in auto mode when the angle it needs to turn it greater than this angle. An angle of zero means to disable pivot turning. Note that you will probably also want to set a low value for WP_RADIUS to get neat turns.
// @Units: deg
// @Range: 0 360
// @Increment: 1
// @User: Standard

// @Param: PIVOT_TURN_RATE
// @DisplayName: Pivot turn rate
// @Description: Desired pivot turn rate in deg/s.
// @Units: deg/s
// @Range: 0 360
// @Increment: 1
// @User: Standard

ParametersG2::ParametersG2(void)
    :
#if ADVANCED_FAILSAFE == ENABLED
    afs(rover.mode_auto.mission),
#endif
    beacon(rover.serial_manager),
    motors(rover.ServoRelayEvents),
    wheel_rate_control(wheel_encoder),
    attitude_control(rover.ahrs),
    smart_rtl(),
    proximity(),
    avoid(),
    follow(),
    windvane(),
    airspeed(),
    wp_nav(attitude_control, rover.L1_controller),
    sailboat()
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
    { Parameters::k_param_throttle_min_old,   0,      AP_PARAM_INT8,  "MOT_THR_MIN" },
    { Parameters::k_param_throttle_max_old,   0,      AP_PARAM_INT8,  "MOT_THR_MAX" },
    { Parameters::k_param_compass_enabled_deprecated,       0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    { Parameters::k_param_pivot_turn_angle_old,   0,  AP_PARAM_INT16,  "WP_PIVOT_ANGLE" },
    { Parameters::k_param_waypoint_radius_old,    0,  AP_PARAM_FLOAT,  "WP_RADIUS" },
    { Parameters::k_param_waypoint_overshoot_old, 0,  AP_PARAM_FLOAT,  "WP_OVERSHOOT" },
    { Parameters::k_param_g2,                20,      AP_PARAM_INT16,  "WP_PIVOT_RATE" },

    { Parameters::k_param_g2,                32,      AP_PARAM_FLOAT,  "SAIL_ANGLE_MIN" },
    { Parameters::k_param_g2,                33,      AP_PARAM_FLOAT,  "SAIL_ANGLE_MAX" },
    { Parameters::k_param_g2,                34,      AP_PARAM_FLOAT,  "SAIL_ANGLE_IDEAL" },
    { Parameters::k_param_g2,                35,      AP_PARAM_FLOAT,  "SAIL_HEEL_MAX" },
    { Parameters::k_param_g2,                36,      AP_PARAM_FLOAT,  "SAIL_NO_GO_ANGLE" },
    { Parameters::k_param_arming,             2,     AP_PARAM_INT16,  "ARMING_CHECK" },
};

void Rover::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad var table\n");
        AP_HAL::panic("Bad var table");
    }

    if (!g.format_version.load() ||
         g.format_version != Parameters::k_format_version) {
        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
    }

    const uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROVER);

    SRV_Channels::set_default_function(CH_1, SRV_Channel::k_steering);
    SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttle);

    if (is_balancebot()) {
        g2.crash_angle.set_default(30);
    }

    SRV_Channels::upgrade_parameters();
    hal.console->printf("load_all took %uus\n", unsigned(micros() - before));

    // set a more reasonable default NAVL1_PERIOD for rovers
    L1_controller.set_default_period(NAVL1_PERIOD);

    // convert CH7_OPTION to RC7_OPTION for Rover-3.4 to 3.5 upgrade
    const AP_Param::ConversionInfo ch7_option_info = { Parameters::k_param_ch7_option, 0, AP_PARAM_INT8, "RC7_OPTION" };
    AP_Int8 ch7_opt_old;
    if (AP_Param::find_old_parameter(&ch7_option_info, &ch7_opt_old)) {
        const uint8_t ch7_opt_map[] = {0,7,50,41,51,52,53,54,16,4,42,55,56};
        const uint8_t ch7_opt_old_val = (uint8_t)ch7_opt_old.get();
        if (ch7_opt_old_val < ARRAY_SIZE(ch7_opt_map)) {
            AP_Param::set_default_by_name(ch7_option_info.new_name, ch7_opt_map[ch7_opt_old_val]);
        }
    }

    // set AR_WPNav's WP_SPEED to be old WP_SPEED (if set) or CRUISE_SPEED (if set)
    const AP_Param::ConversionInfo wp_speed_old_info = { Parameters::k_param_g2, 14, AP_PARAM_FLOAT, "WP_SPEED" };
    const AP_Param::ConversionInfo cruise_speed_info = { Parameters::k_param_speed_cruise, 0, AP_PARAM_FLOAT, "WP_SPEED" };
    AP_Float wp_speed_old;
    if (AP_Param::find_old_parameter(&wp_speed_old_info, &wp_speed_old)) {
        // old WP_SPEED parameter value was set so copy to new WP_SPEED
        AP_Param::convert_old_parameter(&wp_speed_old_info, 1.0f);
    } else {
        // copy CRUISE_SPEED to new WP_SPEED
        AP_Param::convert_old_parameter(&cruise_speed_info, 1.0f);
    }

    // attitude control FF and FILT parameter changes for Rover-3.6
    const AP_Param::ConversionInfo ff_and_filt_conversion_info[] = {
        { Parameters::k_param_g2, 24650, AP_PARAM_FLOAT, "ATC_STR_RAT_FLTE" },
        { Parameters::k_param_g2, 28746, AP_PARAM_FLOAT, "ATC_STR_RAT_FF" },
        { Parameters::k_param_g2, 24714, AP_PARAM_FLOAT, "ATC_SPEED_FLTE" },
        { Parameters::k_param_g2, 28810, AP_PARAM_FLOAT, "ATC_SPEED_FF" },
        { Parameters::k_param_g2, 25226, AP_PARAM_FLOAT, "ATC_BAL_FLTE" },
        { Parameters::k_param_g2, 29322, AP_PARAM_FLOAT, "ATC_BAL_FF" },
        { Parameters::k_param_g2, 25354, AP_PARAM_FLOAT, "ATC_SAIL_FLTE" },
        { Parameters::k_param_g2, 29450, AP_PARAM_FLOAT, "ATC_SAIL_FF" },
    };
    uint8_t filt_table_size = ARRAY_SIZE(ff_and_filt_conversion_info);
    for (uint8_t i=0; i<filt_table_size; i++) {
        AP_Param::convert_old_parameters(&ff_and_filt_conversion_info[i], 1.0f);
    }

    // configure safety switch to allow stopping the motors while armed
#if HAL_HAVE_SAFETY_SWITCH
    AP_Param::set_default_by_name("BRD_SAFETYOPTION", AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF|
                                                      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON|
                                                      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED);
#endif
}
