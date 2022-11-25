#include "Sub.h"

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  ArduSub parameter definitions
 *
 */

#define GSCALAR(v, name, def) { sub.g.v.vtype, name, Parameters::k_param_ ## v, &sub.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { sub.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&sub.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &sub.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&sub.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&sub.v, {group_info : class::var_info} }

const AP_Param::Info Sub::var_info[] = {

    // @Param: SURFACE_DEPTH
    // @DisplayName: Depth reading at surface
    // @Description: The depth the external pressure sensor will read when the vehicle is considered at the surface (in centimeters)
    // @Units: cm
    // @Range: -100 0
    // @User: Standard
    GSCALAR(surface_depth, "SURFACE_DEPTH", SURFACE_DEPTH_DEFAULT),

    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    // @ReadOnly: True
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: .5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Values: 0:None,1:Roll,2:Pitch,4:Yaw
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

#if RANGEFINDER_ENABLED == ENABLED
    // @Param: RNGFND_GAIN
    // @DisplayName: Rangefinder gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the sub
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(rangefinder_gain,     "RNGFND_GAIN",           RANGEFINDER_GAIN_DEFAULT),
#endif

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls what action to take when GCS heartbeat is lost.
    // @Values: 0:Disabled,1:Warn only,2:Disarm,3:Enter depth hold mode,4:Enter surface mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISARM),

    // @Param: FS_LEAK_ENABLE
    // @DisplayName: Leak Failsafe Enable
    // @Description: Controls what action to take if a leak is detected.
    // @Values: 0:Disabled,1:Warn only,2:Enter surface mode
    // @User: Standard
    GSCALAR(failsafe_leak, "FS_LEAK_ENABLE", FS_LEAK_WARN_ONLY),

    // @Param: FS_PRESS_ENABLE
    // @DisplayName: Internal Pressure Failsafe Enable
    // @Description: Controls what action to take if internal pressure exceeds FS_PRESS_MAX parameter.
    // @Values: 0:Disabled,1:Warn only
    // @User: Standard
    GSCALAR(failsafe_pressure, "FS_PRESS_ENABLE", FS_PRESS_DISABLED),

    // @Param: FS_TEMP_ENABLE
    // @DisplayName: Internal Temperature Failsafe Enable
    // @Description: Controls what action to take if internal temperature exceeds FS_TEMP_MAX parameter.
    // @Values: 0:Disabled,1:Warn only
    // @User: Standard
    GSCALAR(failsafe_temperature, "FS_TEMP_ENABLE", FS_TEMP_DISABLED),

    // @Param: FS_PRESS_MAX
    // @DisplayName: Internal Pressure Failsafe Threshold
    // @Description: The maximum internal pressure allowed before triggering failsafe. Failsafe action is determined by FS_PRESS_ENABLE parameter
    // @Units: Pa
    // @User: Standard
    GSCALAR(failsafe_pressure_max, "FS_PRESS_MAX", FS_PRESS_MAX_DEFAULT),

    // @Param: FS_TEMP_MAX
    // @DisplayName: Internal Temperature Failsafe Threshold
    // @Description: The maximum internal temperature allowed before triggering failsafe. Failsafe action is determined by FS_TEMP_ENABLE parameter.
    // @Units: degC
    // @User: Standard
    GSCALAR(failsafe_temperature_max, "FS_TEMP_MAX", FS_TEMP_MAX_DEFAULT),

    // @Param: FS_TERRAIN_ENAB
    // @DisplayName: Terrain Failsafe Enable
    // @Description: Controls what action to take if terrain information is lost during AUTO mode
    // @Values: 0:Disarm, 1:Hold Position, 2:Surface
    // @User: Standard
    GSCALAR(failsafe_terrain, "FS_TERRAIN_ENAB", FS_TERRAIN_DISARM),

    // @Param: FS_PILOT_INPUT
    // @DisplayName: Pilot input failsafe action
    // @Description: Controls what action to take if no pilot input has been received after the timeout period specified by the FS_PILOT_TIMEOUT parameter
    // @Values: 0:Disabled, 1:Warn Only, 2:Disarm
    // @User: Standard
    GSCALAR(failsafe_pilot_input, "FS_PILOT_INPUT", FS_PILOT_INPUT_DISARM),

    // @Param: FS_PILOT_TIMEOUT
    // @DisplayName: Timeout for activation of pilot input failsafe
    // @Description: Controls the maximum interval between received pilot inputs before the failsafe action is triggered
    // @Units: s
    // @Range: 0.1 3.0
    // @User: Standard
    GSCALAR(failsafe_pilot_input_timeout, "FS_PILOT_TIMEOUT", 3.0f),

    // @Param: XTRACK_ANG_LIM
    // @DisplayName: Crosstrack correction angle limit
    // @Description: Maximum allowed angle (in degrees) between current track and desired heading during waypoint navigation
    // @Range: 10 90
    // @User: Standard
    GSCALAR(xtrack_angle_limit,"XTRACK_ANG_LIM", 45),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course, 4:Correct crosstrack error
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: PILOT_SPEED_UP
    // @DisplayName: Pilot maximum vertical ascending speed
    // @Description: The maximum vertical ascending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_up,     "PILOT_SPEED_UP",   PILOT_VELZ_MAX),

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical descending speed
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_dn,     "PILOT_SPEED_DN",   0),

    // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_accel_z,  "PILOT_ACCEL_Z",    PILOT_ACCEL_Z_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The PWM deadzone in microseconds above and below mid throttle. Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: PWM
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Values: 830:Default,894:Default+RCIN,958:Default+IMU,1854:Default+Motors,-6146:NearlyAll-AC315,45054:NearlyAll,131071:All+FastATT,262142:All+MotBatt,393214:All+FastIMU,397310:All+FastIMU+PID,655358:All+FullIMU,0:Disabled
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,17:MOTBATT,18:IMU_FAST,19:IMU_RAW
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 0:Disabled, 1:Warn only, 2:Disarm
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION",    FS_EKF_ACTION_DEFAULT),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH",    FS_EKF_THRESHOLD_DEFAULT),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check enable
    // @Description: This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
    // @Values: 0:Disabled,1:Warn only,2:Disarm
    // @User: Advanced
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    FS_CRASH_DISABLED),

    // @Param: JS_GAIN_DEFAULT
    // @DisplayName: Default gain at boot
    // @Description: Default gain at boot, must be in range [JS_GAIN_MIN , JS_GAIN_MAX]. Current gain value is accessible via NAMED_VALUE_FLOAT MAVLink message with name 'PilotGain'.
    // @User: Standard
    // @Range: 0.1 1.0
    GSCALAR(gain_default, "JS_GAIN_DEFAULT", 0.5),

    // @Param: JS_GAIN_MAX
    // @DisplayName: Maximum joystick gain
    // @Description: Maximum joystick gain
    // @User: Standard
    // @Range: 0.2 1.0
    GSCALAR(maxGain, "JS_GAIN_MAX", 1.0),

    // @Param: JS_GAIN_MIN
    // @DisplayName: Minimum joystick gain
    // @Description: Minimum joystick gain
    // @User: Standard
    // @Range: 0.1 0.8
    GSCALAR(minGain, "JS_GAIN_MIN", 0.25),

    // @Param: JS_GAIN_STEPS
    // @DisplayName: Gain steps
    // @Description: Controls the number of steps between minimum and maximum joystick gain when the gain is adjusted using buttons. Set to 1 to always use JS_GAIN_DEFAULT.
    // @User: Standard
    // @Range: 1 10
    GSCALAR(numGainSettings, "JS_GAIN_STEPS", 4),

    // @Param: JS_LIGHTS_STEPS
    // @DisplayName: Lights brightness steps
    // @Description: Number of steps in brightness between minimum and maximum brightness
    // @User: Standard
    // @Range: 1 10
    // @Units: PWM
    GSCALAR(lights_steps, "JS_LIGHTS_STEPS", 8),

    // @Param: JS_THR_GAIN
    // @DisplayName: Throttle gain scalar
    // @Description: Scalar for gain on the throttle channel
    // @User: Standard
    // @Range: 0.5 4.0
    GSCALAR(throttle_gain, "JS_THR_GAIN", 1.0f),

    // @Param: FRAME_CONFIG
    // @DisplayName: Frame configuration
    // @Description: Set this parameter according to your vehicle/motor configuration
    // @User: Standard
    // @RebootRequired: True
    // @Values: 0:BlueROV1, 1:Vectored, 2:Vectored_6DOF, 3:Vectored_6DOF_90, 4:SimpleROV-3, 5:SimpleROV-4, 6:SimpleROV-5, 7:Custom
    GSCALAR(frame_configuration, "FRAME_CONFIG", AP_Motors6DOF::SUB_FRAME_VECTORED),

    // @Group: BTN0_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_0,                   "BTN0_", JSButton),

    // @Group: BTN1_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_1,                   "BTN1_", JSButton),

    // @Group: BTN2_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_2,                   "BTN2_", JSButton),

    // @Group: BTN3_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_3,                   "BTN3_", JSButton),

    // @Group: BTN4_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_4,                   "BTN4_", JSButton),

    // @Group: BTN5_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_5,                   "BTN5_", JSButton),

    // @Group: BTN6_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_6,                   "BTN6_", JSButton),

    // @Group: BTN7_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_7,                   "BTN7_", JSButton),

    // @Group: BTN8_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_8,                   "BTN8_", JSButton),

    // @Group: BTN9_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_9,                   "BTN9_", JSButton),

    // @Group: BTN10_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_10,                   "BTN10_", JSButton),

    // @Group: BTN11_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_11,                   "BTN11_", JSButton),

    // @Group: BTN12_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_12,                   "BTN12_", JSButton),

    // @Group: BTN13_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_13,                   "BTN13_", JSButton),

    // @Group: BTN14_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_14,                   "BTN14_", JSButton),

    // @Group: BTN15_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_15,                   "BTN15_", JSButton),

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_SPEED_DEFAULT),

    // @Param: ACRO_RP_P
    // @DisplayName: Acro Roll and Pitch P gain
    // @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_rp_p,                 "ACRO_RP_P",           ACRO_RP_P),

    // @Param: ACRO_YAW_P
    // @DisplayName: Acro Yaw P gain
    // @Description: Converts pilot yaw input into a desired rate of rotation.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_yaw_p,                 "ACRO_YAW_P",           ACRO_YAW_P),

    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),

    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     ACRO_TRAINER_LIMITED),

    // @Param: ACRO_EXPO
    // @DisplayName: Acro Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @User: Advanced
    GSCALAR(acro_expo,  "ACRO_EXPO",    ACRO_EXPO_DEFAULT),

    // variables not in the g class which contain EEPROM saved variables

#if AP_CAMERA_ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS_", AP_InertialSensor),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    GOBJECT(loiter_nav, "LOIT_", AC_Loiter),

#if CIRCLE_NAV_ENABLED == ENABLED
    // @Group: CIRCLE_
    // @Path: ../libraries/AC_WPNav/AC_Circle.cpp
    GOBJECT(circle_nav, "CIRCLE_",  AC_Circle),
#endif

    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp
    GOBJECT(attitude_control, "ATC_", AC_AttitudeControl_Sub),

    // @Group: PSC
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECT(pos_control, "PSC", AC_PosControl),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[0],  gcs0,       "SR0_",     GCS_MAVLINK_Parameters),

#if MAVLINK_COMM_NUM_BUFFERS >= 2
    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[1],  gcs1,       "SR1_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 3
    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[2],  gcs2,       "SR2_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 4
    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[3],  gcs3,       "SR3_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 5
    // @Group: SR4_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[4],  gcs4,       "SR4_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 6
    // @Group: SR5_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[5],  gcs5,       "SR5_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 7
    // @Group: SR6_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[6],  gcs6,       "SR6_",     GCS_MAVLINK_Parameters),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: LOG
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger,           "LOG",  AP_Logger),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // @Group: ARMING_
    // @Path: AP_Arming_Sub.cpp,../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming, "ARMING_", AP_Arming_Sub),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if AP_SIM_ENABLED
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // Leak detector
    // @Group: LEAK
    // @Path: ../libraries/AP_LeakDetector/AP_LeakDetector.cpp
    GOBJECT(leak_detector, "LEAK", AP_LeakDetector),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AVOIDANCE_ENABLED == ENABLED
    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
    GOBJECT(avoid,      "AVOID_",   AC_Avoid),
#endif

#if HAL_RALLY_ENABLED
    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,      "RALLY_",   AP_Rally),
#endif

    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_Motors6DOF.cpp,../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    GOBJECT(motors, "MOT_",         AP_Motors6DOF),

#if RCMAP_ENABLED == ENABLED
    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),
#endif

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

#if RANGEFINDER_ENABLED == ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,   "RNGFND", RangeFinder),
#endif

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

#if AP_OPTICALFLOW_ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", AP_OpticalFlow),
#endif

#if AP_RPM_ENABLED
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    { AP_PARAM_GROUP, "", Parameters::k_param_vehicle, (const void *)&sub, {group_info : AP_Vehicle::var_info} },

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

#if HAL_PROXIMITY_ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 2, ParametersG2, AP_Proximity),
#endif

#if AP_GRIPPER_ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 3, ParametersG2, AP_Gripper),
#endif

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels),

#if AP_SCRIPTING_ENABLED
    // @Group: SCR_
    // @Path: ../libraries/AP_Scripting/AP_Scripting.cpp
    AP_SUBGROUPINFO(scripting, "SCR_", 18, ParametersG2, AP_Scripting),
#endif

    // 19 was airspeed

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2()
{
    AP_Param::setup_object_defaults(this, var_info);
}

const AP_Param::ConversionInfo conversion_table[] = {
    { Parameters::k_param_fs_batt_voltage,   0,      AP_PARAM_FLOAT,  "BATT_LOW_VOLT" },
    { Parameters::k_param_fs_batt_mah,       0,      AP_PARAM_FLOAT,  "BATT_LOW_MAH" },
    { Parameters::k_param_failsafe_battery_enabled,       0,      AP_PARAM_INT8,  "BATT_FS_LOW_ACT" },
    { Parameters::k_param_compass_enabled_deprecated,       0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    { Parameters::k_param_arming,            2,     AP_PARAM_INT16,  "ARMING_CHECK" },
};

void Sub::load_parameters()
{
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad var table\n");
        AP_HAL::panic("Bad var table");
    }

    hal.util->set_soft_armed(false);

    if (!g.format_version.load() ||
            g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->println("done.");
    }
    g.format_version.set_default(Parameters::k_format_version);

    uint32_t before = AP_HAL::micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    hal.console->printf("load_all took %uus\n", (unsigned)(AP_HAL::micros() - before));
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_SUB);

    convert_old_parameters();

    AP_Param::set_default_by_name("BRD_SAFETY_DEFLT", 0);
    AP_Param::set_default_by_name("ARMING_CHECK",
            AP_Arming::ARMING_CHECK_RC |
            AP_Arming::ARMING_CHECK_VOLTAGE |
            AP_Arming::ARMING_CHECK_BATTERY);
    AP_Param::set_default_by_name("CIRCLE_RATE", 2.0f);
    AP_Param::set_default_by_name("ATC_ACCEL_Y_MAX", 110000.0f);
    AP_Param::set_default_by_name("RC3_TRIM", 1100);
    AP_Param::set_default_by_name("COMPASS_OFFS_MAX", 1000);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);
    AP_Param::set_default_by_name("MNT1_TYPE", 1);
    AP_Param::set_default_by_name("MNT1_DEFLT_MODE", MAV_MOUNT_MODE_RC_TARGETING);
    AP_Param::set_default_by_name("MNT1_RC_RATE", 30);
    AP_Param::set_default_by_name("RC7_OPTION", 214);   // MOUNT1_YAW
    AP_Param::set_default_by_name("RC8_OPTION", 213);   // MOUNT1_PITCH
    // We should ignore this parameter since ROVs are neutral buoyancy
    AP_Param::set_by_name("MOT_THST_HOVER", 0.5);

// PARAMETER_CONVERSION - Added: JAN-2022
#if AP_AIRSPEED_ENABLED
    // Find G2's Top Level Key
    AP_Param::ConversionInfo info;
    if (!AP_Param::find_top_level_key_by_pointer(&g2, info.old_key)) {
        return;
    }

    const uint16_t old_index = 19;          // Old parameter index in the tree
    const uint16_t old_top_element = 4051;  // Old group element in the tree for the first subgroup element
    AP_Param::convert_class(info.old_key, &airspeed, airspeed.var_info, old_index, old_top_element, false);
#endif


    // PARAMETER_CONVERSION - Added: Mar-2022
#if AP_FENCE_ENABLED
    AP_Param::convert_class(g.k_param_fence_old, &fence, fence.var_info, 0, 0, true);
#endif
}

void Sub::convert_old_parameters()
{
    // attitude control filter parameter changes from _FILT to FLTE or FLTD
    const AP_Param::ConversionInfo filt_conversion_info[] = {
        // move ATC_RAT_RLL/PIT_FILT to FLTD, move ATC_RAT_YAW_FILT to FLTE
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTE" },
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTE" },
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
    };
    AP_Param::convert_old_parameters(&filt_conversion_info[0], ARRAY_SIZE(filt_conversion_info));

    SRV_Channels::upgrade_parameters();
}
