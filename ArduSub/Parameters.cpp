#include "Sub.h"

#include <AP_Gripper/AP_Gripper.h>

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
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // SYSID_THISMAV was here

    // SYSID_MYGCS was here

    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 0.5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // AP_SerialManager was here

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls what action to take when GCS heartbeat is lost.
    // @Values: 0:Disabled,1:Warn only,2:Disarm,3:Enter depth hold mode,4:Enter surface mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISARM),

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_gcs_timeout, "FS_GCS_TIMEOUT", FS_GCS_TIMEOUT_S),

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
    
#if AP_SUB_RC_ENABLED        
    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software RC failsafe activated by a setting on the throttle input channel. It also enables RC failsafe on absence of RC signals being recieved.
    // @Values:  0:Disabled,1: Force effective control inputs to trim positions and prevent arming,2:Surface and hold on surface on failsafe 
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   0),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level in microseconds on channel 3 below which throttle failsafe triggers
    // @Range: 910 1100
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",  FS_THR_VALUE_DEFAULT),
    
    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,7:Circle,9:Surface,16:PosHold,19:Manual,20:Motor Detect,21:SurfTrak
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               (uint8_t)FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1230, <= 1360
    GSCALAR(flight_mode2, "FLTMODE2",               (uint8_t)FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1360, <= 1490
    GSCALAR(flight_mode3, "FLTMODE3",               (uint8_t)FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1490, <= 1620
    GSCALAR(flight_mode4, "FLTMODE4",               (uint8_t)FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1620, <= 1749
    GSCALAR(flight_mode5, "FLTMODE5",               (uint8_t)FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >=1750
    GSCALAR(flight_mode6, "FLTMODE6",               (uint8_t)FLIGHT_MODE_6),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @Values: 0:Disabled,5:Channel5,6:Channel6,7:Channel7,8:Channel8,9:Channel9,10:Channel 10,11:Channel 11,12:Channel 12,13:Channel 13,14:Channel 14,15:Channel 15
    // @User: Advanced
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         0),

    // @Param: THR_ARM_POS
    // @DisplayName: Throttle arming position
    // @Description: Determines if throttle must be at min or within trim value to arm, if RC checks and RC_OPTION for "0" throttle are enabled.
    // @User: Standard
    // @Bitmask: 0:Throttle at or below min, 1: Throttle within a dead zone of RC trim value
    GSCALAR(thr_arming_position, "THR_ARM_POS", 1),    
#endif
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

    // @Param: SURFACE_MAX_THR
    // @DisplayName: Surface Maximum Throttle
    // @Description: Maximum throttle value when the vehicle is at the surface. This value is used to scale throttle linearly from 1. (full) to min as the vehicle approaches the surface. The attenuation starts at 1 meter from surface.  Only upwards throttle is limited.
    // @Range: 0.0 1.0
    // @User: Standard
    // @Increment: 0.01
    GSCALAR(surface_max_throttle, "SURFACE_MAX_THR", 0.1f),

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
    // @Range: 20 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_up,     "PILOT_SPEED_UP",   PILOT_VELZ_MAX),

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical descending speed
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 20 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_dn,     "PILOT_SPEED_DN",   0),

    // @Param: PILOT_SPEED
    // @DisplayName: Pilot maximum horizontal speed
    // @Description: The maximum horizontal velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed,     "PILOT_SPEED",   PILOT_SPEED_DEFAULT),

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
    // @Range: 0.6 1.0
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
    // @Description: Scalar for gain on the throttle channel. Gets scaled with the current JS gain
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

    // @Group: BTN16_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_16,                   "BTN16_", JSButton),

    // @Group: BTN17_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_17,                   "BTN17_", JSButton),

    // @Group: BTN18_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_18,                   "BTN18_", JSButton),

    // @Group: BTN19_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_19,                   "BTN19_", JSButton),

    // @Group: BTN20_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_20,                   "BTN20_", JSButton),

    // @Group: BTN21_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_21,                   "BTN21_", JSButton),

    // @Group: BTN22_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_22,                   "BTN22_", JSButton),

    // @Group: BTN23_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_23,                   "BTN23_", JSButton),

    // @Group: BTN24_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_24,                   "BTN24_", JSButton),

    // @Group: BTN25_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_25,                   "BTN25_", JSButton),

    // @Group: BTN26_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_26,                   "BTN26_", JSButton),

    // @Group: BTN27_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_27,                   "BTN27_", JSButton),

    // @Group: BTN28_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_28,                   "BTN28_", JSButton),

    // @Group: BTN29_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_29,                   "BTN29_", JSButton),

    // @Group: BTN30_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_30,                   "BTN30_", JSButton),

    // @Group: BTN31_
    // @Path: ../libraries/AP_JSButton/AP_JSButton.cpp
    GGROUP(jbtn_31,                   "BTN31_", JSButton),

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
    // @Range: -0.5 0.95
    // @User: Advanced
    GSCALAR(acro_expo,  "ACRO_EXPO",    ACRO_EXPO_DEFAULT),

    // variables not in the g class which contain EEPROM saved variables

#if AP_CAMERA_ENABLED
    // @Group: CAM
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera, "CAM", AP_Camera),
#endif

#if AP_RELAY_ENABLED
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY", AP_Relay),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS", AP_InertialSensor),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    GOBJECT(loiter_nav, "LOIT_", AC_Loiter),

#if CIRCLE_NAV_ENABLED
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

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

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
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
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

#if AVOIDANCE_ENABLED
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

#if RCMAP_ENABLED
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

#if AP_RANGEFINDER_ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,   "RNGFND", RangeFinder),

    // @Param: RNGFND_SQ_MIN
    // @DisplayName: Rangefinder signal quality minimum
    // @Description: Minimum signal quality for good rangefinder readings
    // @Range: 0 100
    // @User: Advanced
    GSCALAR(rangefinder_signal_min, "RNGFND_SQ_MIN", RANGEFINDER_SIGNAL_MIN_DEFAULT),

    // @Param: SURFTRAK_DEPTH
    // @DisplayName: SURFTRAK minimum depth
    // @Description: Minimum depth to engage SURFTRAK mode
    // @Units: cm
    // @User: Standard
    GSCALAR(surftrak_depth, "SURFTRAK_DEPTH", SURFTRAK_DEPTH_DEFAULT),
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

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

#if HAL_GCS_ENABLED
    // @Group: MAV
    // @Path: ../libraries/GCS_MAVLink/GCS.cpp
    GOBJECT(_gcs,           "MAV",  GCS),
#endif

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // 1 was AP_Stats

#if HAL_PROXIMITY_ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 2, ParametersG2, AP_Proximity),
#endif

    // 3 was AP_Gripper

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels),

    // 18 was scripting

    // @Param: ORIGIN_LAT
    // @DisplayName: Backup latitude for EKF origin
    // @Description:  Backup EKF origin latitude used when not using a positioning system.
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LAT", 19, ParametersG2, backup_origin_lat, 0),

    // @Param: ORIGIN_LON
    // @DisplayName: Backup longitude for EKF origin
    // @Description:  Backup EKF origin longitude used when not using a positioning system.
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LON", 20, ParametersG2, backup_origin_lon, 0),

    // @Param: ORIGIN_ALT
    // @DisplayName: Backup altitude (MSL) for EKF origin
    // @Description:  Backup EKF origin altitude (MSL) used when not using a positioning system.
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ORIGIN_ALT", 21, ParametersG2, backup_origin_alt, 0),

    // @Param: SFC_NOBARO_THST
    // @DisplayName: Surface mode throttle output when no barometer is available
    // @Description: Surface mode throttle output when no borometer is available. 100% is full throttle. -100% is maximum throttle downwards
    // @Units: %
    // @User: Standard
    // @Range: -100 100
    AP_GROUPINFO("SFC_NOBARO_THST", 22, ParametersG2, surface_nobaro_thrust, 10),

    // @Group: ACTUATOR
    // @Path: ../ArduSub/actuators.cpp
    AP_SUBGROUPINFO(actuators, "ACTUATOR", 23, ParametersG2, Actuators),

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
};

void Sub::load_parameters()
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_SUB);

    convert_old_parameters();
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));
    // We should ignore this parameter since ROVs are neutral buoyancy
    AP_Param::set_by_name("MOT_THST_HOVER", 0.5);

    // PARAMETER_CONVERSION - Added: Mar-2022
#if AP_FENCE_ENABLED
    AP_Param::convert_class(g.k_param_fence_old, &fence, fence.var_info, 0, true);
#endif

    // PARAMETER_CONVERSION - Added: July-2025 for ArduPilot-4.7
#if AP_RPM_ENABLED
    AP_Param::convert_class(g.k_param_rpm_sensor_old, &rpm_sensor, rpm_sensor.var_info, 0, true, true);
#endif

    static const AP_Param::G2ObjectConversion g2_conversions[] {
#if AP_AIRSPEED_ENABLED
    // PARAMETER_CONVERSION - Added: JAN-2022
        { &airspeed, airspeed.var_info, 19 },
#endif
#if AP_STATS_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024
        { &stats, stats.var_info, 1 },
#endif
#if AP_SCRIPTING_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024
        { &scripting, scripting.var_info, 18 },
#endif
#if AP_GRIPPER_ENABLED
    // PARAMETER_CONVERSION - Added: Feb-2024
        { &gripper, gripper.var_info, 3 },
#endif
    };

    AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

    // PARAMETER_CONVERSION - Added: Feb-2024
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

    static const AP_Param::TopLevelObjectConversion toplevel_conversions[] {
#if AP_SERIALMANAGER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024
        { &serial_manager, serial_manager.var_info, Parameters::k_param_serial_manager_old },
#endif
    };

    AP_Param::convert_toplevel_objects(toplevel_conversions, ARRAY_SIZE(toplevel_conversions));

#if HAL_GCS_ENABLED
    // Move parameters into new MAV_ parameter namespace
    // PARAMETER_CONVERSION - Added: Mar-2025
    {
        static const AP_Param::ConversionInfo gcs_conversion_info[] {
            { Parameters::k_param_sysid_this_mav_old, 0, AP_PARAM_INT16,  "MAV_SYSID" },
            { Parameters::k_param_sysid_my_gcs_old, 0, AP_PARAM_INT16, "MAV_GCS_SYSID" },
        };
        AP_Param::convert_old_parameters(&gcs_conversion_info[0], ARRAY_SIZE(gcs_conversion_info));
    }
#endif  // HAL_GCS_ENABLED
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

#if LEAKDETECTOR_MAX_INSTANCES > 0
// PARAMETER_CONVERSION - Added: Dec-2025
// Deals with leak detector getting misconfigured when updating from Sub 4.1
void Sub::update_leak_pins()
{
    for (uint8_t instance = 0; instance < LEAKDETECTOR_MAX_INSTANCES; instance++) {
        if (leak_detector.get_pin(instance) <= 0) {
            // leak detector does not use pin
            continue;
        }
        uint8_t servo_channel;
        if (!hal.gpio->pin_to_servo_channel(leak_detector.get_pin(instance), servo_channel)) {
            // leak detector pin does not map to a servo channel
            continue;
        }
        if (SRV_Channels::is_GPIO(servo_channel)) {
            // servo channel is already set to GPIO
            continue;
        }
        if (SRV_Channels::channel_function(servo_channel) != SRV_Channel::Function::k_none) {
            // servo channel is already set to a function
            gcs().send_text(MAV_SEVERITY_WARNING, "Leak detector %u error. Please set SERVO%u_FUNCTION to GPIO", instance + 1, servo_channel + 1);
            continue;
        }
        // servo channel is disabled, let's set it to GPIO for the user
        gcs().send_text(MAV_SEVERITY_INFO, "Leak detector %u pin (servo %u) auto-set to GPIO", instance + 1, servo_channel + 1);
        char param_name[20];
        snprintf(param_name, sizeof(param_name), "SERVO%u_FUNCTION", servo_channel + 1);
        AP_Param::set_and_save_by_name(param_name, static_cast<int>(SRV_Channel::Function::k_GPIO));
    }
}
#endif

#if AP_RELAY_ENABLED
// PARAMETER_CONVERSION - Added: Dec-2025
// Deals with relay getting misconfigured when updating from Sub 4.1
void Sub::update_relay_pins()
{
    for (uint8_t instance = 0; instance < AP_RELAY_NUM_RELAYS; instance++) {
        uint8_t servo_channel;
        uint8_t pin;
        if (!relay.get_pin_by_instance(instance, pin) || !hal.gpio->pin_to_servo_channel(pin, servo_channel)) {
            // instance does not use pin or pin does not map to a servo channel
            continue;
        }
        if (!relay.enabled(instance) || SRV_Channels::is_GPIO(servo_channel)) {
            // instance is not enabled or servo channel is already set to GPIO
            continue;
        }
        if (SRV_Channels::channel_function(servo_channel) != SRV_Channel::Function::k_none) {
            // servo channel is already set to a function
            gcs().send_text(MAV_SEVERITY_WARNING, "Relay %u error. Please set SERVO%u_FUNCTION to GPIO", instance + 1, servo_channel + 1);
            continue;
        }
        // servo channel is disabled, let's set it to GPIO for the user
        gcs().send_text(MAV_SEVERITY_INFO, "Relay %u pin (servo %u) auto-set to GPIO", instance + 1, servo_channel + 1);
        char param_name[20];
        snprintf(param_name, sizeof(param_name), "SERVO%u_FUNCTION", servo_channel + 1);
        AP_Param::set_and_save_by_name(param_name, static_cast<int>(SRV_Channel::Function::k_GPIO));
    }
}
#endif
