#include "Copter.h"

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
 *  ArduCopter parameter definitions
 *
 */

#define GSCALAR(v, name, def) { copter.g.v.vtype, name, Parameters::k_param_ ## v, &copter.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { copter.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&copter.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &copter.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&copter.v, {group_info : class::var_info} }
#define GOBJECTPTR(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&copter.v, {group_info : class::var_info}, AP_PARAM_FLAG_POINTER }
#define GOBJECTVARPTR(v, name, var_info_ptr) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&copter.v, {group_info_ptr : var_info_ptr}, AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&copter.v, {group_info : class::var_info} }

#if FRAME_CONFIG == HELI_FRAME
// 6 here is AP_Motors::MOTOR_FRAME_HELI
#define DEFAULT_FRAME_CLASS 6
#else
#define DEFAULT_FRAME_CLASS 0
#endif

const AP_Param::Info Copter::var_info[] = {
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
    // @Values: 255:Mission Planner and DroidPlanner, 252: AP Planner 2
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

    // @Param: PILOT_TKOFF_ALT
    // @DisplayName: Pilot takeoff altitude
    // @Description: Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
    // @User: Standard
    // @Units: cm
    // @Range: 0.0 1000.0
    // @Increment: 10
    GSCALAR(pilot_takeoff_alt,  "PILOT_TKOFF_ALT",  PILOT_TKOFF_ALT_DEFAULT),

    // @Param: PILOT_THR_BHV
    // @DisplayName: Throttle stick behavior
    // @Description: Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.
    // @User: Standard
    // @Values: 0:None,1:Feedback from mid stick,2:High throttle cancels landing,4:Disarm on land detection
    // @Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection
    GSCALAR(throttle_behavior, "PILOT_THR_BHV", 0),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Advanced
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Values: 0:None,1:Roll,2:Pitch,4:Yaw,8:AccelZ
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw,3:AccelZ
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

#if MODE_RTL_ENABLED == ENABLED
    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum alt above home the vehicle will climb to before returning.  If the vehicle is flying higher than this value it will return at its current altitude.
    // @Units: cm
    // @Range: 200 8000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: RTL_CONE_SLOPE
    // @DisplayName: RTL cone slope
    // @Description: Defines a cone above home which determines maximum climb
    // @Range: 0.5 10.0
    // @Increment: .1
    // @Values: 0:Disabled,1:Shallow,3:Steep
    // @User: Standard
    GSCALAR(rtl_cone_slope,   "RTL_CONE_SLOPE",     RTL_CONE_SLOPE_DEFAULT),

    // @Param: RTL_SPEED
    // @DisplayName: RTL speed
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    GSCALAR(rtl_speed_cms,   "RTL_SPEED",     0),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: cm
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

    // @Param: RTL_CLIMB_MIN
    // @DisplayName: RTL minimum climb
    // @Description: The vehicle will climb this many cm during the initial climb portion of the RTL
    // @Units: cm
    // @Range: 0 3000
    // @Increment: 10
    // @User: Standard
    GSCALAR(rtl_climb_min,  "RTL_CLIMB_MIN",    RTL_CLIMB_MIN_DEFAULT),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before beginning final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),
#endif

#if RANGEFINDER_ENABLED == ENABLED
    // @Param: RNGFND_GAIN
    // @DisplayName: Rangefinder gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(rangefinder_gain, "RNGFND_GAIN", RANGEFINDER_GAIN_DEFAULT),
#endif

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode (Deprecated in 4.0+),3:Enabled always SmartRTL or RTL,4:Enabled always SmartRTL or Land,5:Enabled always land (4.0+ Only)
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Super Simple Mode
    // @Description: Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Mode1,2:Mode2,3:Mode1+2,4:Mode3,5:Mode1+3,6:Mode2+3,7:Mode1+2+3,8:Mode4,9:Mode1+4,10:Mode2+4,11:Mode1+2+4,12:Mode3+4,13:Mode1+3+4,14:Mode2+3+4,15:Mode1+2+3+4,16:Mode5,17:Mode1+5,18:Mode2+5,19:Mode1+2+5,20:Mode3+5,21:Mode1+3+5,22:Mode2+3+5,23:Mode1+2+3+5,24:Mode4+5,25:Mode1+4+5,26:Mode2+4+5,27:Mode1+2+4+5,28:Mode3+4+5,29:Mode1+3+4+5,30:Mode2+3+4+5,31:Mode1+2+3+4+5,32:Mode6,33:Mode1+6,34:Mode2+6,35:Mode1+2+6,36:Mode3+6,37:Mode1+3+6,38:Mode2+3+6,39:Mode1+2+3+6,40:Mode4+6,41:Mode1+4+6,42:Mode2+4+6,43:Mode1+2+4+6,44:Mode3+4+6,45:Mode1+3+4+6,46:Mode2+3+4+6,47:Mode1+2+3+4+6,48:Mode5+6,49:Mode1+5+6,50:Mode2+5+6,51:Mode1+2+5+6,52:Mode3+5+6,53:Mode1+3+5+6,54:Mode2+3+5+6,55:Mode1+2+3+5+6,56:Mode4+5+6,57:Mode1+4+5+6,58:Mode2+4+5+6,59:Mode1+2+4+5+6,60:Mode3+4+5+6,61:Mode1+3+4+5+6,62:Mode2+3+4+5+6,63:Mode1+2+3+4+5+6
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     0),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: LAND_SPEED_HIGH
    // @DisplayName: Land speed high
    // @Description: The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed_high,        "LAND_SPEED_HIGH",   0),
    
    // @Param: PILOT_SPEED_UP
    // @DisplayName: Pilot maximum vertical speed ascending
    // @Description: The maximum vertical ascending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_up,     "PILOT_SPEED_UP",   PILOT_VELZ_MAX),

    // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_accel_z,  "PILOT_ACCEL_Z",    PILOT_ACCEL_Z_DEFAULT),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values:  0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode (Deprecated in 4.0+),3:Enabled always Land,4:Enabled always SmartRTL or RTL,5:Enabled always SmartRTL or Land
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_ENABLED_ALWAYS_RTL),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level in microseconds on channel 3 below which throttle failsafe triggers
    // @Range: 925 1100
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: PWM
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               (uint8_t)FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               (uint8_t)FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               (uint8_t)FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               (uint8_t)FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               (uint8_t)FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               (uint8_t)FLIGHT_MODE_6),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @Values: 0:Disabled,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    // @User: Advanced
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         CH_MODE_DEFAULT),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Values: 830:Default,894:Default+RCIN,958:Default+IMU,1854:Default+Motors,-6146:NearlyAll-AC315,45054:NearlyAll,131071:All+FastATT,262142:All+MotBatt,393214:All+FastIMU,397310:All+FastIMU+PID,655358:All+FullIMU,0:Disabled
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,17:MOTBATT,18:IMU_FAST,19:IMU_RAW
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: ESC_CALIBRATION
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up, 1:Start-up in ESC Calibration mode if throttle high, 2:Start-up in ESC Calibration mode regardless of throttle, 3:Start-up and automatically calibrate ESCs, 9:Disabled
    GSCALAR(esc_calibrate, "ESC_CALIBRATION",       0),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,56:Rate Yaw Filter,55:Motor Yaw Headroom,14:AltHold kP,7:Throttle Rate kP,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,12:Loiter Pos kP,22:Velocity XY kP,28:Velocity XY kI,10:WP Speed,25:Acro RollPitch kP,40:Acro Yaw kP,45:RC Feel,13:Heli Ext Gyro,38:Declination,39:Circle Rate,41:RangeFinder Gain,46:Rate Pitch kP,47:Rate Pitch kI,48:Rate Pitch kD,49:Rate Roll kP,50:Rate Roll kI,51:Rate Roll kD,52:Rate Pitch FF,53:Rate Roll FF,54:Rate Yaw FF,58:SysID Magnitude
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X, V, etc)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15: I, 18: BetaFlightXReversed
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(frame_type, "FRAME_TYPE", HAL_FRAME_TYPE_DEFAULT),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Copter),

    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds. A value of zero disables auto disarm.
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),
    
    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: cdeg
    // @Range: 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

    // @Param: PHLD_BRAKE_RATE
    // @DisplayName: PosHold braking rate
    // @Description: PosHold flight mode's rotation rate during braking in deg/sec
    // @Units: deg/s
    // @Range: 4 12
    // @User: Advanced
    GSCALAR(poshold_brake_rate, "PHLD_BRAKE_RATE",  POSHOLD_BRAKE_RATE_DEFAULT),

    // @Param: PHLD_BRAKE_ANGLE
    // @DisplayName: PosHold braking angle max
    // @Description: PosHold flight mode's max lean angle during braking in centi-degrees
    // @Units: cdeg
    // @Range: 2000 4500
    // @User: Advanced
    GSCALAR(poshold_brake_angle_max, "PHLD_BRAKE_ANGLE",  POSHOLD_BRAKE_ANGLE_DEFAULT),

    // @Param: LAND_REPOSITION
    // @DisplayName: Land repositioning
    // @Description: Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
    // @Values: 0:No repositioning, 1:Repositioning
    // @User: Advanced
    GSCALAR(land_repositioning, "LAND_REPOSITION",     LAND_REPOSITION_DEFAULT),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 1:Land, 2:AltHold, 3:Land even in Stabilize
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
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    1),

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

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

#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the roll axis. A higher value causes faster decay of desired to actual attitude.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the pitch axis. A higher value causes faster decay of desired to actual attitude.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),
#endif

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     (uint8_t)ModeAcro::Trainer::LIMITED),

    // @Param: ACRO_RP_EXPO
    // @DisplayName: Acro Roll/Pitch Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced
    GSCALAR(acro_rp_expo,  "ACRO_RP_EXPO",    ACRO_RP_EXPO_DEFAULT),
#endif

    // variables not in the g class which contain EEPROM saved variables

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

#if PARACHUTE == ENABLED
    // @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute, "CHUTE_", AP_Parachute),
#endif

    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    GOBJECT(landinggear,    "LGR_", AP_LandingGear),

#if FRAME_CONFIG == HELI_FRAME
    // @Group: IM_
    // @Path: ../libraries/AC_InputManager/AC_InputManager_Heli.cpp
    GOBJECT(input_manager, "IM_", AC_InputManager_Heli),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS_", AP_InertialSensor),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECTPTR(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    GOBJECTPTR(loiter_nav, "LOIT_", AC_Loiter),

#if MODE_CIRCLE_ENABLED == ENABLED
    // @Group: CIRCLE_
    // @Path: ../libraries/AC_WPNav/AC_Circle.cpp
    GOBJECTPTR(circle_nav, "CIRCLE_",  AC_Circle),
#endif

    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
#if FRAME_CONFIG == HELI_FRAME
    GOBJECTPTR(attitude_control, "ATC_", AC_AttitudeControl_Heli),
#else
    GOBJECTPTR(attitude_control, "ATC_", AC_AttitudeControl_Multi),
#endif

    // @Group: PSC
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECTPTR(pos_control, "PSC", AC_PosControl),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[0],  gcs0,       "SR0_",     GCS_MAVLINK_Parameters),

    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[1],  gcs1,       "SR1_",     GCS_MAVLINK_Parameters),

    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[2],  gcs2,       "SR2_",     GCS_MAVLINK_Parameters),

    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[3],  gcs3,       "SR3_",     GCS_MAVLINK_Parameters),

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

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if SPRAYER_ENABLED == ENABLED
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    GOBJECT(sprayer,                "SPRAY_",       AC_Sprayer),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    GOBJECT(sitl, "SIM_", SITL::SITL),
#endif

    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AC_FENCE == ENABLED
    // @Group: FENCE_
    // @Path: ../libraries/AC_Fence/AC_Fence.cpp
    GOBJECT(fence,      "FENCE_",   AC_Fence),
#endif

    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
#if AC_AVOID_ENABLED == ENABLED
    GOBJECT(avoid,      "AVOID_",   AC_Avoid),
#endif

#if AC_RALLY == ENABLED
    // @Group: RALLY_
    // @Path: AP_Rally.cpp,../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,      "RALLY_",   AP_Rally_Copter),
#endif

#if FRAME_CONFIG == HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli_Single.cpp,../libraries/AP_Motors/AP_MotorsHeli_Dual.cpp,../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECTVARPTR(motors, "H_",        &copter.motors_var_info),
#else
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    GOBJECTVARPTR(motors, "MOT_",      &copter.motors_var_info),
#endif

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

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

#if MODE_AUTO_ENABLED == ENABLED
    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECTN(mode_auto.mission, mission, "MIS_", AP_Mission),
#endif

    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
    
#if RANGEFINDER_ENABLED == ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,   "RNGFND", RangeFinder),
#endif

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

#if OPTFLOW == ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", OpticalFlow),
#endif

#if PRECISION_LANDING == ENABLED
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    GOBJECT(precland, "PLND_", AC_PrecLand),
#endif

#if RPM_ENABLED == ENABLED
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

#if HAL_ADSB_ENABLED
    // @Group: ADSB_
    // @Path: ../libraries/AP_ADSB/AP_ADSB.cpp
    GOBJECT(adsb,                "ADSB_", AP_ADSB),

    // @Group: AVD_
    // @Path: ../libraries/AP_Avoidance/AP_Avoidance.cpp
    GOBJECT(avoidance_adsb, "AVD_", AP_Avoidance_Copter),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

#if MODE_THROW_ENABLED == ENABLED
    // @Param: THROW_MOT_START
    // @DisplayName: Start motors before throwing is detected
    // @Description: Used by Throw mode. Controls whether motors will run at the speed set by MOT_SPIN_MIN or will be stopped when armed and waiting for the throw.
    // @Values: 0:Stopped,1:Running
    // @User: Standard
    GSCALAR(throw_motor_start, "THROW_MOT_START", 0),
#endif

    // @Param: RTL_ALT_TYPE
    // @DisplayName: RTL mode altitude type
    // @Description: RTL altitude type.  Set to 1 for Terrain following during RTL and then set WPNAV_RFND_USE=1 to use rangefinder or WPNAV_RFND_USE=0 to use Terrain database
    // @Values: 0:Relative to Home, 1:Terrain
    // @User: Standard
    GSCALAR(rtl_alt_type, "RTL_ALT_TYPE", 0),

#if OSD_ENABLED == ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    { AP_PARAM_GROUP, "", Parameters::k_param_vehicle, (const void *)&copter, {group_info : AP_Vehicle::var_info} },

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // @Param: WP_NAVALT_MIN
    // @DisplayName: Minimum navigation altitude
    // @Description: This is the altitude in meters above which for navigation can begin. This applies in auto takeoff and auto landing.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("WP_NAVALT_MIN", 1, ParametersG2, wp_navalt_min, 0),

#if BUTTON_ENABLED == ENABLED
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    AP_SUBGROUPPTR(button_ptr, "BTN_", 2, ParametersG2, AP_Button),
#endif

#if MODE_THROW_ENABLED == ENABLED
    // @Param: THROW_NEXTMODE
    // @DisplayName: Throw mode's follow up mode
    // @Description: Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)
    // @Values: 3:Auto,4:Guided,5:LOITER,6:RTL,9:Land,17:Brake,18:Throw
    // @User: Standard
    AP_GROUPINFO("THROW_NEXTMODE", 3, ParametersG2, throw_nextmode, 18),

    // @Param: THROW_TYPE
    // @DisplayName: Type of Type
    // @Description: Used by Throw mode. Specifies whether Copter is thrown upward or dropped.
    // @Values: 0:Upward Throw,1:Drop
    // @User: Standard
    AP_GROUPINFO("THROW_TYPE", 4, ParametersG2, throw_type, ModeThrow::ThrowType_Upward),
#endif

    // @Param: GND_EFFECT_COMP
    // @DisplayName: Ground Effect Compensation Enable/Disable
    // @Description: Ground Effect Compensation Enable/Disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GND_EFFECT_COMP", 5, ParametersG2, gndeffect_comp_enabled, 1),

#if ADVANCED_FAILSAFE == ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    AP_SUBGROUPINFO(afs, "AFS_", 6, ParametersG2, AP_AdvancedFailsafe),
#endif

    // @Param: DEV_OPTIONS
    // @DisplayName: Development options
    // @Description: Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning
    // @Bitmask: 0:ADSBMavlinkProcessing,1:DevOptionVFR_HUDRelativeAlt,2:SetAttitudeTarget_ThrustAsThrust
    // @User: Advanced
    AP_GROUPINFO("DEV_OPTIONS", 7, ParametersG2, dev_options, 0),

#if BEACON_ENABLED == ENABLED
    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 14, ParametersG2, AP_Beacon),
#endif

#if PROXIMITY_ENABLED == ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 8, ParametersG2, AP_Proximity),
#endif

    // @Param: ACRO_Y_EXPO
    // @DisplayName: Acro Yaw Expo
    // @Description: Acro yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced
    AP_GROUPINFO("ACRO_Y_EXPO", 9, ParametersG2, acro_y_expo, ACRO_Y_EXPO_DEFAULT),

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_THR_MID
    // @DisplayName: Acro Thr Mid
    // @Description: Acro Throttle Mid
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACRO_THR_MID", 10, ParametersG2, acro_thr_mid, ACRO_THR_MID_DEFAULT),
#endif

    // @Param: SYSID_ENFORCE
    // @DisplayName: GCS sysid enforcement
    // @Description: This controls whether packets from other than the expected GCS system ID will be accepted
    // @Values: 0:NotEnforced,1:Enforced
    // @User: Advanced
    AP_GROUPINFO("SYSID_ENFORCE", 11, ParametersG2, sysid_enforce, 0),

#if STATS_ENABLED == ENABLED
    // @Group: STAT
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 12, ParametersG2, AP_Stats),
#endif

#if GRIPPER_ENABLED == ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 13, ParametersG2, AP_Gripper),
#endif

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 6:Heli, 7:Tri, 8:SingleCopter, 9:CoaxCopter, 10:BiCopter, 11:Heli_Dual, 12:DodecaHexa, 13:HeliQuad
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_CLASS", 15, ParametersG2, frame_class, DEFAULT_FRAME_CLASS),

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels_Copter),

    // 18 was used by AP_VisualOdom

    // @Group: TCAL
    // @Path: ../libraries/AP_TempCalibration/AP_TempCalibration.cpp
    AP_SUBGROUPINFO(temp_calibration, "TCAL", 19, ParametersG2, AP_TempCalibration),

#if TOY_MODE_ENABLED == ENABLED
    // @Group: TMODE
    // @Path: toy_mode.cpp
    AP_SUBGROUPINFO(toy_mode, "TMODE", 20, ParametersG2, ToyMode),
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
    // @Group: SRTL_
    // @Path: ../libraries/AP_SmartRTL/AP_SmartRTL.cpp
    AP_SUBGROUPINFO(smart_rtl, "SRTL_", 21, ParametersG2, AP_SmartRTL),
#endif

#if WINCH_ENABLED == ENABLED
    // 22 was AP_WheelEncoder

    // @Group: WINCH
    // @Path: ../libraries/AP_Winch/AP_Winch.cpp
    AP_SUBGROUPINFO(winch, "WINCH", 23, ParametersG2, AP_Winch),
#endif

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical speed descending
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PILOT_SPEED_DN", 24, ParametersG2, pilot_speed_dn, 0),

    // @Param: LAND_ALT_LOW
    // @DisplayName: Land alt low
    // @Description: Altitude during Landing at which vehicle slows to LAND_SPEED
    // @Units: cm
    // @Range: 100 10000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("LAND_ALT_LOW", 25, ParametersG2, land_alt_low, 1000),

#if !HAL_MINIMIZE_FEATURES && OPTFLOW == ENABLED
    // @Group: FHLD
    // @Path: mode_flowhold.cpp
    AP_SUBGROUPPTR(mode_flowhold_ptr, "FHLD", 26, ParametersG2, ModeFlowHold),
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 27, ParametersG2, AP_Follow),
#endif

#ifdef USER_PARAMS_ENABLED
    AP_SUBGROUPINFO(user_parameters, "USR", 28, ParametersG2, UserParameters),
#endif

#if AUTOTUNE_ENABLED == ENABLED
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune.cpp
    AP_SUBGROUPPTR(autotune_ptr, "AUTOTUNE_",  29, ParametersG2, AutoTune),
#endif

#ifdef ENABLE_SCRIPTING
    // @Group: SCR_
    // @Path: ../libraries/AP_Scripting/AP_Scripting.cpp
    AP_SUBGROUPINFO(scripting, "SCR_", 30, ParametersG2, AP_Scripting),
#endif

    // @Param: TUNE_MIN
    // @DisplayName: Tuning minimum
    // @Description: Minimum value that the parameter currently being tuned with the transmitter's channel 6 knob will be set to
    // @User: Standard
    AP_GROUPINFO("TUNE_MIN", 31, ParametersG2, tuning_min, 0),

    // @Param: TUNE_MAX
    // @DisplayName: Tuning maximum
    // @Description: Maximum value that the parameter currently being tuned with the transmitter's channel 6 knob will be set to
    // @User: Standard
    AP_GROUPINFO("TUNE_MAX", 32, ParametersG2, tuning_max, 0),

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    // @Group: OA_
    // @Path: ../libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    AP_SUBGROUPINFO(oa, "OA_", 33, ParametersG2, AP_OAPathPlanner),
#endif

#if MODE_SYSTEMID_ENABLED == ENABLED
    // @Group: SID
    // @Path: mode_systemid.cpp
    AP_SUBGROUPPTR(mode_systemid_ptr, "SID", 34, ParametersG2, ModeSystemId),
#endif

    // @Param: FS_VIBE_ENABLE
    // @DisplayName: Vibration Failsafe enable
    // @Description: This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("FS_VIBE_ENABLE", 35, ParametersG2, fs_vibe_enabled, 1),

    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe options bitmask
    // @Description: Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.
    // @Values: 0:Disabled, 1:Continue if in Auto on RC failsafe only, 2:Continue if in Auto on GCS failsafe only, 3:Continue if in Auto on RC and/or GCS failsafe, 4:Continue if in Guided on RC failsafe only, 8:Continue if landing on any failsafe, 16:Continue if in pilot controlled modes on GCS failsafe, 19:Continue if in Auto on RC and/or GCS failsafe and continue if in pilot controlled modes on GCS failsafe
    // @Bitmask: 0:Continue if in Auto on RC failsafe, 1:Continue if in Auto on GCS failsafe, 2:Continue if in Guided on RC failsafe, 3:Continue if landing on any failsafe, 4:Continue if in pilot controlled modes on GCS failsafe, 5:Release Gripper
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 36, ParametersG2, fs_options, 0),

#if MODE_AUTOROTATE_ENABLED == ENABLED
    // @Group: AROT_
    // @Path: ../libraries/AC_Autorotation/AC_Autorotation.cpp
    AP_SUBGROUPINFO(arot, "AROT_", 37, ParametersG2, AC_Autorotation),
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
    // @Group: ZIGZ_
    // @Path: mode_zigzag.cpp
    AP_SUBGROUPPTR(mode_zigzag_ptr, "ZIGZ_", 38, ParametersG2, ModeZigZag),
#endif

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_OPTIONS
    // @DisplayName: Acro mode options
    // @Description: A range of options that can be applied to change acro mode behaviour. Air-mode enables ATC_THR_MIX_MAN at all times (air-mode has no effect on helicopters). Rate Loop Only disables the use of angle stabilization and uses angular rate stabilization only.
    // @Bitmask: 0:Air-mode,1:Rate Loop Only
    // @User: Advanced
    AP_GROUPINFO("ACRO_OPTIONS", 39, ParametersG2, acro_options, 0),
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // @Param: AUTO_OPTIONS
    // @DisplayName: Auto mode options
    // @Description: A range of options that can be applied to change auto mode behaviour. Allow Arming allows the copter to be armed in Auto. Allow Takeoff Without Raising Throttle allows takeoff without the pilot having to raise the throttle
    // @Bitmask: 0:Allow Arming,1:Allow Takeoff Without Raising Throttle
    // @User: Advanced
    AP_GROUPINFO("AUTO_OPTIONS", 40, ParametersG2, auto_options, 0),
#endif

#if MODE_GUIDED_ENABLED == ENABLED
    // @Param: GUID_OPTIONS
    // @DisplayName: Guided mode options
    // @Description: Options that can be applied to change guided mode behaviour
    // @Bitmask: 0:Allow Arming from Transmitter
    // @User: Advanced
    AP_GROUPINFO("GUID_OPTIONS", 41, ParametersG2, guided_options, 0),
#endif

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2(void)
    : temp_calibration() // this doesn't actually need constructing, but removing it here is problematic syntax-wise
#if BEACON_ENABLED == ENABLED
    , beacon(copter.serial_manager)
#endif
#if PROXIMITY_ENABLED == ENABLED
    , proximity()
#endif
#if ADVANCED_FAILSAFE == ENABLED
    ,afs(copter.mode_auto.mission)
#endif
#if MODE_SMARTRTL_ENABLED == ENABLED
    ,smart_rtl()
#endif
#if !HAL_MINIMIZE_FEATURES && OPTFLOW == ENABLED
    ,mode_flowhold_ptr(&copter.mode_flowhold)
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    ,follow()
#endif
#ifdef USER_PARAMS_ENABLED
    ,user_parameters()
#endif
#if AUTOTUNE_ENABLED == ENABLED
    ,autotune_ptr(&copter.autotune)
#endif
#if MODE_SYSTEMID_ENABLED == ENABLED
    ,mode_systemid_ptr(&copter.mode_systemid)
#endif
#if MODE_AUTOROTATE_ENABLED == ENABLED
    ,arot()
#endif
    ,button_ptr(&copter.button)
#if MODE_ZIGZAG_ENABLED == ENABLED
    ,mode_zigzag_ptr(&copter.mode_zigzag)
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
    { Parameters::k_param_log_bitmask_old,    0,      AP_PARAM_INT16, "LOG_BITMASK" },
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },
    { Parameters::k_param_arming_check_old,   0,      AP_PARAM_INT8,  "ARMING_CHECK" },
    // battery
    { Parameters::k_param_fs_batt_voltage,    0,      AP_PARAM_FLOAT,  "BATT_LOW_VOLT" },
    { Parameters::k_param_fs_batt_mah,        0,      AP_PARAM_FLOAT,  "BATT_LOW_MAH" },
    { Parameters::k_param_failsafe_battery_enabled,0, AP_PARAM_INT8,   "BATT_FS_LOW_ACT" },

    { Parameters::Parameters::k_param_ch7_option_old,   0,      AP_PARAM_INT8,  "RC7_OPTION" },
    { Parameters::Parameters::k_param_ch8_option_old,   0,      AP_PARAM_INT8,  "RC8_OPTION" },
    { Parameters::Parameters::k_param_ch9_option_old,   0,      AP_PARAM_INT8,  "RC9_OPTION" },
    { Parameters::Parameters::k_param_ch10_option_old,   0,      AP_PARAM_INT8,  "RC10_OPTION" },
    { Parameters::Parameters::k_param_ch11_option_old,   0,      AP_PARAM_INT8,  "RC11_OPTION" },
    { Parameters::Parameters::k_param_ch12_option_old,   0,      AP_PARAM_INT8,  "RC12_OPTION" },
    { Parameters::k_param_compass_enabled_deprecated,    0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    { Parameters::k_param_arming,             2,     AP_PARAM_INT16,  "ARMING_CHECK" },
};

void Copter::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad var table\n");
        AP_HAL::panic("Bad var table");
    }

    // disable centrifugal force correction, it will be enabled as part of the arming process
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

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

    uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    // convert landing gear parameters
    convert_lgr_parameters();

    // convert fs_options parameters
    convert_fs_options_params();

    hal.console->printf("load_all took %uus\n", (unsigned)(micros() - before));

    // setup AP_Param frame type flags
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_COPTER);

}

// handle conversion of PID gains
void Copter::convert_pid_parameters(void)
{
    // conversion info
    const AP_Param::ConversionInfo pid_conversion_info[] = {
        { Parameters::k_param_pid_rate_roll, 0, AP_PARAM_FLOAT, "ATC_RAT_RLL_P" },
        { Parameters::k_param_pid_rate_roll, 1, AP_PARAM_FLOAT, "ATC_RAT_RLL_I" },
        { Parameters::k_param_pid_rate_roll, 2, AP_PARAM_FLOAT, "ATC_RAT_RLL_D" },
        { Parameters::k_param_pid_rate_pitch, 0, AP_PARAM_FLOAT, "ATC_RAT_PIT_P" },
        { Parameters::k_param_pid_rate_pitch, 1, AP_PARAM_FLOAT, "ATC_RAT_PIT_I" },
        { Parameters::k_param_pid_rate_pitch, 2, AP_PARAM_FLOAT, "ATC_RAT_PIT_D" },
        { Parameters::k_param_pid_rate_yaw, 0, AP_PARAM_FLOAT, "ATC_RAT_YAW_P" },
        { Parameters::k_param_pid_rate_yaw, 1, AP_PARAM_FLOAT, "ATC_RAT_YAW_I" },
        { Parameters::k_param_pid_rate_yaw, 2, AP_PARAM_FLOAT, "ATC_RAT_YAW_D" },
#if FRAME_CONFIG == HELI_FRAME
        { Parameters::k_param_pid_rate_roll,  4, AP_PARAM_FLOAT, "ATC_RAT_RLL_VFF" },
        { Parameters::k_param_pid_rate_pitch, 4, AP_PARAM_FLOAT, "ATC_RAT_PIT_VFF" },
        { Parameters::k_param_pid_rate_yaw  , 4, AP_PARAM_FLOAT, "ATC_RAT_YAW_VFF" },
#endif
    };
    const AP_Param::ConversionInfo imax_conversion_info[] = {
        { Parameters::k_param_pid_rate_roll,  5, AP_PARAM_FLOAT, "ATC_RAT_RLL_IMAX" },
        { Parameters::k_param_pid_rate_pitch, 5, AP_PARAM_FLOAT, "ATC_RAT_PIT_IMAX" },
        { Parameters::k_param_pid_rate_yaw,   5, AP_PARAM_FLOAT, "ATC_RAT_YAW_IMAX" },
#if FRAME_CONFIG == HELI_FRAME
        { Parameters::k_param_pid_rate_roll,  7, AP_PARAM_FLOAT, "ATC_RAT_RLL_ILMI" },
        { Parameters::k_param_pid_rate_pitch, 7, AP_PARAM_FLOAT, "ATC_RAT_PIT_ILMI" },
        { Parameters::k_param_pid_rate_yaw,   7, AP_PARAM_FLOAT, "ATC_RAT_YAW_ILMI" },
#endif
    };
    // conversion from Copter-3.3 to Copter-3.4
    const AP_Param::ConversionInfo angle_and_filt_conversion_info[] = {
        { Parameters::k_param_p_stabilize_roll, 0, AP_PARAM_FLOAT, "ATC_ANG_RLL_P" },
        { Parameters::k_param_p_stabilize_pitch, 0, AP_PARAM_FLOAT, "ATC_ANG_PIT_P" },
        { Parameters::k_param_p_stabilize_yaw, 0, AP_PARAM_FLOAT, "ATC_ANG_YAW_P" },
        { Parameters::k_param_pid_rate_roll, 6, AP_PARAM_FLOAT, "ATC_RAT_RLL_FILT" },
        { Parameters::k_param_pid_rate_pitch, 6, AP_PARAM_FLOAT, "ATC_RAT_PIT_FILT" },
        { Parameters::k_param_pid_rate_yaw, 6, AP_PARAM_FLOAT, "ATC_RAT_YAW_FILT" },
        { Parameters::k_param_pi_vel_xy, 0, AP_PARAM_FLOAT, "PSC_VELXY_P" },
        { Parameters::k_param_pi_vel_xy, 1, AP_PARAM_FLOAT, "PSC_VELXY_I" },
        { Parameters::k_param_pi_vel_xy, 2, AP_PARAM_FLOAT, "PSC_VELXY_IMAX" },
        { Parameters::k_param_pi_vel_xy, 3, AP_PARAM_FLOAT, "PSC_VELXY_FILT" },
        { Parameters::k_param_p_vel_z, 0, AP_PARAM_FLOAT, "PSC_VELZ_P" },
        { Parameters::k_param_pid_accel_z, 0, AP_PARAM_FLOAT, "PSC_ACCZ_P" },
        { Parameters::k_param_pid_accel_z, 1, AP_PARAM_FLOAT, "PSC_ACCZ_I" },
        { Parameters::k_param_pid_accel_z, 2, AP_PARAM_FLOAT, "PSC_ACCZ_D" },
        { Parameters::k_param_pid_accel_z, 5, AP_PARAM_FLOAT, "PSC_ACCZ_IMAX" },
        { Parameters::k_param_pid_accel_z, 6, AP_PARAM_FLOAT, "PSC_ACCZ_FLTE" },
        { Parameters::k_param_p_alt_hold, 0, AP_PARAM_FLOAT, "PSC_POSZ_P" },
        { Parameters::k_param_p_pos_xy, 0, AP_PARAM_FLOAT, "PSC_POSXY_P" },
    };
    const AP_Param::ConversionInfo throttle_conversion_info[] = {
        { Parameters::k_param_throttle_min, 0, AP_PARAM_FLOAT, "MOT_SPIN_MIN" },
        { Parameters::k_param_throttle_mid, 0, AP_PARAM_FLOAT, "MOT_THST_HOVER" }
    };
    const AP_Param::ConversionInfo loiter_conversion_info[] = {
        { Parameters::k_param_wp_nav, 4, AP_PARAM_FLOAT, "LOIT_SPEED" },
        { Parameters::k_param_wp_nav, 7, AP_PARAM_FLOAT, "LOIT_BRK_JERK" },
        { Parameters::k_param_wp_nav, 8, AP_PARAM_FLOAT, "LOIT_ACC_MAX" },
        { Parameters::k_param_wp_nav, 9, AP_PARAM_FLOAT, "LOIT_BRK_ACCEL" }
    };

    // gains increase by 27% due to attitude controller's switch to use radians instead of centi-degrees
    // and motor libraries switch to accept inputs in -1 to +1 range instead of -4500 ~ +4500
    float pid_scaler = 1.27f;

#if FRAME_CONFIG != HELI_FRAME
    // Multicopter x-frame gains are 40% lower because -1 or +1 input to motors now results in maximum rotation
    if (g.frame_type == AP_Motors::MOTOR_FRAME_TYPE_X || g.frame_type == AP_Motors::MOTOR_FRAME_TYPE_V || g.frame_type == AP_Motors::MOTOR_FRAME_TYPE_H) {
        pid_scaler = 0.9f;
    }
#endif

    // scale PID gains
    uint8_t table_size = ARRAY_SIZE(pid_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&pid_conversion_info[i], pid_scaler);
    }
    // reduce IMAX into -1 ~ +1 range
    table_size = ARRAY_SIZE(imax_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&imax_conversion_info[i], 1.0f/4500.0f);
    }
    // convert angle controller gain and filter without scaling
    table_size = ARRAY_SIZE(angle_and_filt_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&angle_and_filt_conversion_info[i], 1.0f);
    }
    // convert throttle parameters (multicopter only)
    table_size = ARRAY_SIZE(throttle_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&throttle_conversion_info[i], 0.001f);
    }
    // convert RC_FEEL_RP to ATC_INPUT_TC
    const AP_Param::ConversionInfo rc_feel_rp_conversion_info = { Parameters::k_param_rc_feel_rp, 0, AP_PARAM_INT8, "ATC_INPUT_TC" };
    AP_Int8 rc_feel_rp_old;
    if (AP_Param::find_old_parameter(&rc_feel_rp_conversion_info, &rc_feel_rp_old)) {
        AP_Param::set_default_by_name(rc_feel_rp_conversion_info.new_name, (1.0f / (2.0f + rc_feel_rp_old.get() * 0.1f)));
    }
    // convert loiter parameters
    table_size = ARRAY_SIZE(loiter_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&loiter_conversion_info[i], 1.0f);
    }

    // TradHeli default parameters
#if FRAME_CONFIG == HELI_FRAME
    static const struct AP_Param::defaults_table_struct heli_defaults_table[] = {
        { "LOIT_ACC_MAX", 500.0f },
        { "LOIT_BRK_ACCEL", 125.0f },
        { "LOIT_BRK_DELAY", 1.0f },
        { "LOIT_BRK_JERK", 250.0f },
        { "LOIT_SPEED", 3000.0f },
        { "PHLD_BRAKE_ANGLE", 800.0f },
        { "PHLD_BRAKE_RATE", 4.0f },
        { "PSC_ACCZ_P", 0.28f },
        { "PSC_VELXY_D", 0.0f },
        { "PSC_VELXY_I", 0.5f },
        { "PSC_VELXY_P", 1.0f },
        { "RC8_OPTION", 32 },
        { "RC_OPTIONS", 0 },
    };
    AP_Param::set_defaults_from_table(heli_defaults_table, ARRAY_SIZE(heli_defaults_table));
#endif

    // attitude and position control filter parameter changes (from _FILT to FLTD, FLTE, FLTT) for Copter-4.0
    // magic numbers shown below are discovered by setting AP_PARAM_KEY_DUMP = 1
    const AP_Param::ConversionInfo ff_and_filt_conversion_info[] = {
#if FRAME_CONFIG == HELI_FRAME
        // tradheli moves ATC_RAT_RLL/PIT_FILT to FLTE, ATC_RAT_YAW_FILT to FLTE
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTE" },
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTE" },
        { Parameters::k_param_attitude_control, 388, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
#else
        // multicopters move ATC_RAT_RLL/PIT_FILT to FLTD & FLTT, ATC_RAT_YAW_FILT to FLTE
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTD" },
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTT" },
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTD" },
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTT" },
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
        { Parameters::k_param_attitude_control, 449, AP_PARAM_FLOAT, "ATC_RAT_RLL_FF" },
        { Parameters::k_param_attitude_control, 450, AP_PARAM_FLOAT, "ATC_RAT_PIT_FF" },
        { Parameters::k_param_attitude_control, 451, AP_PARAM_FLOAT, "ATC_RAT_YAW_FF" },
#endif
        { Parameters::k_param_pos_control, 388, AP_PARAM_FLOAT, "PSC_ACCZ_FLTE" },
    };
    uint8_t filt_table_size = ARRAY_SIZE(ff_and_filt_conversion_info);
    for (uint8_t i=0; i<filt_table_size; i++) {
        AP_Param::convert_old_parameters(&ff_and_filt_conversion_info[i], 1.0f);
    }

    // notch filter parameter conversions (changed position and type) for Copter-3.7
    const AP_Param::ConversionInfo notchfilt_conversion_info[] = {
        { Parameters::k_param_ins, 165, AP_PARAM_INT16, "INS_NOTCH_FREQ" },
        { Parameters::k_param_ins, 229, AP_PARAM_INT16, "INS_NOTCH_BW" },
    };
    uint8_t notchfilt_table_size = ARRAY_SIZE(notchfilt_conversion_info);
    for (uint8_t i=0; i<notchfilt_table_size; i++) {
        AP_Param::convert_old_parameters(&notchfilt_conversion_info[i], 1.0f);
    }

    // make any SRV_Channel upgrades needed
    SRV_Channels::upgrade_parameters();
}

/*
  convert landing gear parameters
 */
void Copter::convert_lgr_parameters(void)
{
    // convert landing gear PWM values
    uint8_t chan;
    if (!SRV_Channels::find_channel(SRV_Channel::k_landing_gear_control, chan)) {
        return;
    }
    // parameter names are 1 based
    chan += 1;

    char pname[17];
    AP_Int16 *servo_min, *servo_max, *servo_trim;
    AP_Int16 *servo_reversed;

    enum ap_var_type ptype;
    // get pointers to the servo min, max and trim parameters
    snprintf(pname, sizeof(pname), "SERVO%u_MIN", chan);
    servo_min = (AP_Int16 *)AP_Param::find(pname, &ptype);

    snprintf(pname, sizeof(pname), "SERVO%u_MAX", chan);
    servo_max = (AP_Int16 *)AP_Param::find(pname, &ptype);

    snprintf(pname, sizeof(pname), "SERVO%u_TRIM", chan);
    servo_trim = (AP_Int16 *)AP_Param::find(pname, &ptype);

    snprintf(pname, sizeof(pname), "SERVO%u_REVERSED", chan & 0x3F); // Only use the 6 LSBs, avoids a cpp warning
    servo_reversed = (AP_Int16 *)AP_Param::find(pname, &ptype);

    if (!servo_min || !servo_max || !servo_trim || !servo_reversed) {
        // this shouldn't happen
        return;
    }
    if (servo_min->configured_in_storage() ||
        servo_max->configured_in_storage() ||
        servo_trim->configured_in_storage() ||
        servo_reversed->configured_in_storage()) {
        // has been previously saved, don't upgrade
        return;
    }

    // get the old PWM values
    AP_Int16 old_pwm;
    uint16_t old_retract=0, old_deploy=0;
    const AP_Param::ConversionInfo cinfo_ret { Parameters::k_param_landinggear, 0, AP_PARAM_INT16, nullptr };
    const AP_Param::ConversionInfo cinfo_dep { Parameters::k_param_landinggear, 1, AP_PARAM_INT16, nullptr };
    if (AP_Param::find_old_parameter(&cinfo_ret, &old_pwm)) {
        old_retract = (uint16_t)old_pwm.get();
    }
    if (AP_Param::find_old_parameter(&cinfo_dep, &old_pwm)) {
        old_deploy = (uint16_t)old_pwm.get();
    }

    if (old_retract == 0 && old_deploy == 0) {
        // old parameters were never set
        return;
    }

    // use old defaults
    if (old_retract == 0) {
        old_retract = 1250;
    }
    if (old_deploy == 0) {
        old_deploy = 1750;
    }

    // set and save correct values on the servo
    if (old_retract <= old_deploy) {
        servo_max->set_and_save(old_deploy);
        servo_min->set_and_save(old_retract);
        servo_trim->set_and_save(old_retract);
        servo_reversed->set_and_save_ifchanged(0);
    } else {
        servo_max->set_and_save(old_retract);
        servo_min->set_and_save(old_deploy);
        servo_trim->set_and_save(old_deploy);
        servo_reversed->set_and_save_ifchanged(1);
    }
}

#if FRAME_CONFIG == HELI_FRAME
// handle conversion of tradheli parameters from Copter-3.6 to Copter-3.7
void Copter::convert_tradheli_parameters(void)
{
    if (g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI) {
        // single heli conversion info
        const AP_Param::ConversionInfo singleheli_conversion_info[] = {
            { Parameters::k_param_motors, 1, AP_PARAM_INT16, "H_SW_H3_SV1_POS" },
            { Parameters::k_param_motors, 2, AP_PARAM_INT16, "H_SW_H3_SV2_POS" },
            { Parameters::k_param_motors, 3, AP_PARAM_INT16, "H_SW_H3_SV3_POS" },
            { Parameters::k_param_motors, 7, AP_PARAM_INT16, "H_SW_H3_PHANG" },
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW_COL_DIR" },
        };

        // convert single heli parameters without scaling
        uint8_t table_size = ARRAY_SIZE(singleheli_conversion_info);
        for (uint8_t i=0; i<table_size; i++) {
            AP_Param::convert_old_parameter(&singleheli_conversion_info[i], 1.0f);
        }

        // convert to known swash type for setups that match
        AP_Int16 swash_pos_1, swash_pos_2, swash_pos_3, swash_phang; 
        AP_Int8  swash_type;
        bool swash_pos1_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[0], &swash_pos_1);
        bool swash_pos2_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[1], &swash_pos_2);
        bool swash_pos3_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[2], &swash_pos_3);
        bool swash_phang_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[3], &swash_phang);
        const AP_Param::ConversionInfo swash_type_info { Parameters::k_param_motors, 5, AP_PARAM_INT8, "H_SW_TYPE" };
        bool swash_type_exists = AP_Param::find_old_parameter(&swash_type_info, &swash_type);

        if (swash_type_exists) {
            // convert swash type to new parameter
            AP_Param::convert_old_parameter(&swash_type_info, 1.0f);
        } else {
        // old swash type is not in eeprom and thus type is default value of generic swash
            if (swash_pos1_exist || swash_pos2_exist || swash_pos3_exist || swash_phang_exist) {
                // if any params exist with the generic swash then the upgraded swash type must be generic
                // find the new variable in the variable structures
                enum ap_var_type ptype;
                AP_Param *ap2;
                ap2 = AP_Param::find("H_SW_TYPE", &ptype);
                // make sure the pointer is valid
                if (ap2 != nullptr) {
                    // see if we can load it from EEPROM
                    if (!ap2->configured_in_storage()) {
                        // the new parameter is not in storage so set generic swash
                        AP_Param::set_and_save_by_name("H_SW_TYPE", SwashPlateType::SWASHPLATE_TYPE_H3);            
                    }
                }
            }
        }
    } else if (g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI_DUAL) {
        // dual heli conversion info
        const AP_Param::ConversionInfo dualheli_conversion_info[] = {
            { Parameters::k_param_motors, 1, AP_PARAM_INT16, "H_SW_H3_SV1_POS" },
            { Parameters::k_param_motors, 2, AP_PARAM_INT16, "H_SW_H3_SV2_POS" },
            { Parameters::k_param_motors, 3, AP_PARAM_INT16, "H_SW_H3_SV3_POS" },
            { Parameters::k_param_motors, 4, AP_PARAM_INT16, "H_SW2_H3_SV1_POS" },
            { Parameters::k_param_motors, 5, AP_PARAM_INT16, "H_SW2_H3_SV2_POS" },
            { Parameters::k_param_motors, 6, AP_PARAM_INT16, "H_SW2_H3_SV3_POS" },
            { Parameters::k_param_motors, 7, AP_PARAM_INT16, "H_SW_H3_PHANG" },
            { Parameters::k_param_motors, 8, AP_PARAM_INT16, "H_SW2_H3_PHANG" },
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW_COL_DIR" },
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW2_COL_DIR" },
        };

        // convert dual heli parameters without scaling
        uint8_t table_size = ARRAY_SIZE(dualheli_conversion_info);
        for (uint8_t i=0; i<table_size; i++) {
            AP_Param::convert_old_parameter(&dualheli_conversion_info[i], 1.0f);
        }


        // convert to known swash type for setups that match
        AP_Int16 swash1_pos_1, swash1_pos_2, swash1_pos_3, swash1_phang, swash2_pos_1, swash2_pos_2, swash2_pos_3, swash2_phang; 
        bool swash1_pos1_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[0], &swash1_pos_1);
        bool swash1_pos2_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[1], &swash1_pos_2);
        bool swash1_pos3_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[2], &swash1_pos_3);
        bool swash1_phang_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[6], &swash1_phang);
        bool swash2_pos1_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[3], &swash2_pos_1);
        bool swash2_pos2_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[4], &swash2_pos_2);
        bool swash2_pos3_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[5], &swash2_pos_3);
        bool swash2_phang_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[7], &swash2_phang);

        // SWASH 1
        // old swash type is not in eeprom and thus type is default value of generic swash
        if (swash1_pos1_exist || swash1_pos2_exist || swash1_pos3_exist || swash1_phang_exist) {
            // if any params exist with the generic swash then the upgraded swash type must be generic
            // find the new variable in the variable structures
            enum ap_var_type ptype;
            AP_Param *ap2;
            ap2 = AP_Param::find("H_SW_TYPE", &ptype);
            // make sure the pointer is valid
            if (ap2 != nullptr) {
                // see if we can load it from EEPROM
                if (!ap2->configured_in_storage()) {
                    // the new parameter is not in storage so set generic swash
                    AP_Param::set_and_save_by_name("H_SW_TYPE", SwashPlateType::SWASHPLATE_TYPE_H3);            
                }
            }
        }
        //SWASH 2
        // old swash type is not in eeprom and thus type is default value of generic swash
        if (swash2_pos1_exist || swash2_pos2_exist || swash2_pos3_exist || swash2_phang_exist) {
            // if any params exist with the generic swash then the upgraded swash type must be generic
            // find the new variable in the variable structures
            enum ap_var_type ptype;
            AP_Param *ap2;
            ap2 = AP_Param::find("H_SW2_TYPE", &ptype);
            // make sure the pointer is valid
            if (ap2 != nullptr) {
                // see if we can load it from EEPROM
                if (!ap2->configured_in_storage()) {
                    // the new parameter is not in storage so set generic swash
                    AP_Param::set_and_save_by_name("H_SW2_TYPE", SwashPlateType::SWASHPLATE_TYPE_H3);            
                }
            }
        }
    }

    // table of rsc parameters to be converted with scaling
    const AP_Param::ConversionInfo rschelipct_conversion_info[] = {
        { Parameters::k_param_motors, 1280, AP_PARAM_INT16, "H_RSC_THRCRV_0" },
        { Parameters::k_param_motors, 1344, AP_PARAM_INT16, "H_RSC_THRCRV_25" },
        { Parameters::k_param_motors, 1408, AP_PARAM_INT16, "H_RSC_THRCRV_50" },
        { Parameters::k_param_motors, 1472, AP_PARAM_INT16, "H_RSC_THRCRV_75" },
        { Parameters::k_param_motors, 1536, AP_PARAM_INT16, "H_RSC_THRCRV_100" },
        { Parameters::k_param_motors, 448, AP_PARAM_INT16, "H_RSC_SETPOINT" },
        { Parameters::k_param_motors, 768, AP_PARAM_INT16, "H_RSC_CRITICAL" },
        { Parameters::k_param_motors, 832, AP_PARAM_INT16, "H_RSC_IDLE" },
    };
    // convert heli rsc parameters with scaling
    uint8_t table_size = ARRAY_SIZE(rschelipct_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&rschelipct_conversion_info[i], 0.1f);
    }

    // table of rsc parameters to be converted without scaling
    const AP_Param::ConversionInfo rscheli_conversion_info[] = {
        { Parameters::k_param_motors, 512, AP_PARAM_INT8,  "H_RSC_MODE" },
        { Parameters::k_param_motors, 640, AP_PARAM_INT8,  "H_RSC_RAMP_TIME" },
        { Parameters::k_param_motors, 704, AP_PARAM_INT8,  "H_RSC_RUNUP_TIME" },
        { Parameters::k_param_motors, 1216, AP_PARAM_INT16,"H_RSC_SLEWRATE" },
    };
    // convert heli rsc parameters without scaling
    table_size = ARRAY_SIZE(rscheli_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&rscheli_conversion_info[i], 1.0f);
    }

    // update tail speed parameter with scaling
    AP_Int16 *tailspeed;
    enum ap_var_type ptype;
    tailspeed = (AP_Int16 *)AP_Param::find("H_TAIL_SPEED", &ptype);
    if (tailspeed != nullptr && tailspeed->get() > 100 ) {
        uint16_t tailspeed_pct = (uint16_t)(0.1f * tailspeed->get());
        AP_Param::set_and_save_by_name("H_TAIL_SPEED", tailspeed_pct );
    }

    // table of stabilize collective parameters to be converted with scaling
    const AP_Param::ConversionInfo collhelipct_conversion_info[] = {
        { Parameters::k_param_input_manager, 1, AP_PARAM_INT16,  "IM_STB_COL_1" },
        { Parameters::k_param_input_manager, 2, AP_PARAM_INT16,  "IM_STB_COL_2" },
        { Parameters::k_param_input_manager, 3, AP_PARAM_INT16,  "IM_STB_COL_3" },
        { Parameters::k_param_input_manager, 4, AP_PARAM_INT16,  "IM_STB_COL_4" },
    };

    // convert stabilize collective parameters with scaling
    table_size = ARRAY_SIZE(collhelipct_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&collhelipct_conversion_info[i], 0.1f);
    }

}
#endif

void Copter::convert_fs_options_params(void)
{
    // If FS_OPTIONS has already been configured and we don't change it.
    enum ap_var_type ptype;
    AP_Int32 *fs_opt = (AP_Int32 *)AP_Param::find("FS_OPTIONS", &ptype);

    if (fs_opt == nullptr || fs_opt->configured_in_storage() || ptype != AP_PARAM_INT32) {
        return;
    }

    // Variable to capture the new FS_OPTIONS setting
    int32_t fs_options_converted = 0;

    // If FS_THR_ENABLED is 2 (continue mission), change to RTL and add continue mission to the new FS_OPTIONS parameter
    if (g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
        fs_options_converted |= int32_t(FailsafeOption::RC_CONTINUE_IF_AUTO);
        AP_Param::set_and_save_by_name("FS_THR_ENABLE", FS_THR_ENABLED_ALWAYS_RTL);
    }

    // If FS_GCS_ENABLED is 2 (continue mission), change to RTL and add continue mission to the new FS_OPTIONS parameter
    if (g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
        fs_options_converted |= int32_t(FailsafeOption::GCS_CONTINUE_IF_AUTO);
        AP_Param::set_and_save_by_name("FS_GCS_ENABLE", FS_GCS_ENABLED_ALWAYS_RTL);
    }

    // Write the new value to FS_OPTIONS
    // AP_Param::set_and_save_by_name("FS_OPTIONS", fs_options_converted);
    fs_opt->set_and_save(fs_options_converted);
}
