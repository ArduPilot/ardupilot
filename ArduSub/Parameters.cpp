/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
 *  ArduCopter parameter definitions
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
	// @Range: -100 0
    // @User: Standard
	GSCALAR(surface_depth, "SURFACE_DEPTH", SURFACE_DEPTH_DEFAULT),

    // @Param: SYSID_SW_MREV
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
	// @ReadOnly: True
    GSCALAR(format_version, "SYSID_SW_MREV",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @Values: 0:ArduPlane,4:AntennaTracker,10:Copter,20:Rover
    // @User: Advanced
	// @ReadOnly: True
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

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

#if CLI_ENABLED == ENABLED
    // @Param: CLI_ENABLED
    // @DisplayName: CLI Enable
    // @Description: This enables/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(cli_enabled,    "CLI_ENABLED",    0),
#endif

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

    // @Param: RNGFND_GAIN
    // @DisplayName: Rangefinder gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(rangefinder_gain,     "RNGFND_GAIN",           RANGEFINDER_GAIN_DEFAULT),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Land,2:RTL
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATT_DISABLED),

    // @Param: FS_BATT_VOLTAGE
    // @DisplayName: Failsafe battery voltage
    // @Description: Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
    // @Units: Volts
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(fs_batt_voltage,        "FS_BATT_VOLTAGE", FS_BATT_VOLTAGE_DEFAULT),

    // @Param: FS_BATT_MAH
    // @DisplayName: Failsafe battery milliAmpHours
    // @Description: Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    GSCALAR(fs_batt_mah,            "FS_BATT_MAH", FS_BATT_MAH_DEFAULT),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls what action to take when GCS heartbeat is lost.
    // @Values: 0:Disabled,1:Warn only,2:Disarm,3:Enter depth hold mode,4:Enter surface mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: FS_LEAK_ENABLE
    // @DisplayName: Leak Failsafe Enable
    // @Description: Controls what action to take if a leak is detected.
    // @Values: 0:Disabled,1:Warn only,2:Enter surface mode
    // @User: Standard
    GSCALAR(failsafe_leak, "FS_LEAK_ENABLE", FS_LEAK_DISABLED),

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
    // @Units: Pascal
    // @User: Standard
    GSCALAR(failsafe_pressure_max, "FS_PRESS_MAX", FS_PRESS_MAX_DEFAULT),

    // @Param: FS_TEMP_MAX
    // @DisplayName: Internal Temperature Failsafe Threshold
    // @Description: The maximum internal temperature allowed before triggering failsafe. Failsafe action is determined by FS_TEMP_ENABLE parameter.
    // @Units: Degrees Centigrade
    // @User: Standard
    GSCALAR(failsafe_temperature_max, "FS_TEMP_MAX", FS_TEMP_MAX_DEFAULT),

    // @Param: FS_TERRAIN_ENAB
    // @DisplayName: Terrain Failsafe Enable
    // @Description: Controls what action to take if terrain information is lost during AUTO mode
    // @Values: 0:Disarm, 1:Hold Position, 2:Surface
    // @User: Standard
    GSCALAR(failsafe_terrain, "FS_TERRAIN_ENAB", FS_TERRAIN_DISARM),

    // @Param: XTRACK_ANG_LIM
    // @DisplayName: Crosstrack correction angle limit
    // @Description: Maximum allowed angle (in degrees) between current track and desired heading during waypoint navigation
    // @Range: 10 90
    // @User: Standard
	GSCALAR(xtrack_angle_limit,"XTRACK_ANG_LIM", 45),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: MAG_ENABLE
    // @DisplayName: Compass enable/disable
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: PILOT_VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_velocity_z_max,     "PILOT_VELZ_MAX",   PILOT_VELZ_MAX),

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
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode,3:Enabled always LAND
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_DISABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @Range: 925 1100
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle.  Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: pwm
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
	// @Values: 0:Stabilize,2:DepthHold,19:Manual
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
	// @Values: 0:Stabilize,2:DepthHold,19:Manual
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
	// @Values: 0:Stabilize,2:DepthHold,19:Manual
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
	// @Values: 0:Stabilize,2:DepthHold,19:Manual
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
	// @Values: 0:Stabilize,2:DepthHold,19:Manual
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
	// @Values: 0:Stabilize,2:DepthHold,19:Manual
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Values: 830:Default,894:Default+RCIN,958:Default+IMU,1854:Default+Motors,-6146:NearlyAll-AC315,45054:NearlyAll,131070:All+DisarmedLogging,131071:All+FastATT,262142:All+MotBatt,393214:All+FastIMU,397310:All+FastIMU+PID,655358:All+FullIMU,0:Disabled
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,16:WHEN_DISARMED,17:MOTBATT,18:IMU_FAST,19:IMU_RAW
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: ESC_CALIBRATION
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up, 1:Start-up in ESC Calibration mode if throttle high, 2:Start-up in ESC Calibration mode regardless of throttle, 9:Disabled
    GSCALAR(esc_calibrate, "ESC_CALIBRATION",       0),

#if CH6_TUNE_ENABLED == ENABLED
    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,14:Altitude Hold kP,7:Throttle Rate kP,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,42:Loiter Speed,12:Loiter Pos kP,22:Velocity XY kP,28:Velocity XY kI,10:WP Speed,25:Acro RollPitch kP,40:Acro Yaw kP,13:Heli Ext Gyro,17:OF Loiter kP,18:OF Loiter kI,19:OF Loiter kD,38:Declination,39:Circle Rate,41:RangeFinder Gain,46:Rate Pitch kP,47:Rate Pitch kI,48:Rate Pitch kD,49:Rate Roll kP,50:Rate Roll kI,51:Rate Roll kD,52:Rate Pitch FF,53:Rate Roll FF,54:Rate Yaw FF
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
#endif

#if AUXSW_ENABLED == ENABLED
    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
	// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  AUXSW_DO_NOTHING),

    // @Param: CH8_OPT
    // @DisplayName: Channel 8 option
    // @Description: Select which function if performed when CH8 is above 1800 pwm
	// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw
    // @User: Standard
    GSCALAR(ch8_option, "CH8_OPT",                  AUXSW_DO_NOTHING),

    // @Param: CH9_OPT
    // @DisplayName: Channel 9 option
    // @Description: Select which function if performed when CH9 is above 1800 pwm
	// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw
    // @User: Standard
    GSCALAR(ch9_option, "CH9_OPT",                  AUXSW_DO_NOTHING),

    // @Param: CH10_OPT
    // @DisplayName: Channel 10 option
    // @Description: Select which function if performed when CH10 is above 1800 pwm
	// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw
    // @User: Standard
    GSCALAR(ch10_option, "CH10_OPT",                AUXSW_DO_NOTHING),

    // @Param: CH11_OPT
    // @DisplayName: Channel 11 option
    // @Description: Select which function if performed when CH11 is above 1800 pwm
	// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw
    // @User: Standard
    GSCALAR(ch11_option, "CH11_OPT",                AUXSW_DO_NOTHING),

    // @Param: CH12_OPT
    // @DisplayName: Channel 12 option
    // @Description: Select which function if performed when CH12 is above 1800 pwm
	// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:Gripper, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off, 29:Landing Gear, 30:Lost Copter Sound, 31:Motor Emergency Stop, 32:Motor Interlock, 33:Brake, 37:Throw
    // @User: Standard
    GSCALAR(ch12_option, "CH12_OPT",                AUXSW_DO_NOTHING),
#endif

    // @Param: ARMING_CHECK
    // @DisplayName: Arming check
    // @Description: Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
    // @Values: 0:Disabled, 1:Enabled, -3:Skip Baro, -5:Skip Compass, -9:Skip GPS, -17:Skip INS, -33:Skip Params/Rangefinder, -65:Skip RC, 127:Skip Voltage
    // @Bitmask: 0:All,1:Baro,2:Compass,3:GPS,4:INS,5:Parameters+Rangefinder,6:RC,7:Voltage
    // @User: Standard
    GSCALAR(arming_check, "ARMING_CHECK",           ARMING_CHECK_NONE),

    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds. A value of zero disables auto disarm.
    // @Units: Seconds
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),
    
    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: Centi-degrees
    // @Range: 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

    // @Param: RC_FEEL_RP
    // @DisplayName: RC Feel Roll/Pitch
    // @Description: RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
    // @Range: 0 100
    // @Increment: 10
    // @User: Standard
    // @Values: 0:Very Soft, 25:Soft, 50:Medium, 75:Crisp, 100:Very Crisp
    GSCALAR(rc_feel_rp, "RC_FEEL_RP",  RC_FEEL_RP_MEDIUM),

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
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    0),

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
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),

    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),
    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),

    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                   "RC12_", RC_Channel_aux),

    // @Group: RC13_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_13,                   "RC13_", RC_Channel_aux),

    // @Group: RC14_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_14,                   "RC14_", RC_Channel_aux),

	// @Param: JS_GAIN_DEFAULT
	// @DisplayName: Default gain at boot
	// @Description: Default gain at boot, must be in range [JS_GAIN_MIN , JS_GAIN_MAX]
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

	// @Param: JS_CAM_TILT_STEP
	// @DisplayName: Camera tilt step size
	// @Description: Size of PWM increment on camera tilt servo
	// @User: Standard
	// @Range: 30 400
	GSCALAR(cam_tilt_step, "JS_CAM_TILT_STEP", 50),

	// @Param: JS_LIGHTS_STEP
	// @DisplayName: Lights step size
	// @Description: Size of PWM increment on lights servo
	// @User: Standard
	// @Range: 30 400
	GSCALAR(lights_step, "JS_LIGHTS_STEP", 100),

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
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: ACRO_RP_P
    // @DisplayName: Acro Roll and Pitch P gain
    // @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_rp_p,                 "ACRO_RP_P",           ACRO_RP_P),

    // @Param: ACRO_YAW_P
    // @DisplayName: Acro Yaw P gain
    // @Description: Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
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

    // @Param: VEL_XY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VEL_XY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VEL_XY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VEL_XY_FILT_HZ
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced
    GGROUP(pi_vel_xy,   "VEL_XY_",  AC_PI_2D),

    // @Param: VEL_Z_P
    // @DisplayName: Velocity (vertical) P gain
    // @Description: Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    GGROUP(p_vel_z,     "VEL_Z_", AC_P),

    // @Param: ACCEL_Z_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
	// @Increment: 0.05
    // @User: Standard

    // @Param: ACCEL_Z_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: ACCEL_Z_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Standard

    // @Param: ACCEL_Z_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: ACCEL_Z_FILT
    // @DisplayName: Throttle acceleration filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    GGROUP(pid_accel_z, "ACCEL_Z_", AC_PID),

    // @Param: POS_Z_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    GGROUP(p_alt_hold,              "POS_Z_", AC_P),

    // @Param: POS_XY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    GGROUP(p_pos_xy,                "POS_XY_", AC_P),

#if TRANSECT_ENABLED == ENABLED
	GGROUP(pid_crosstrack_control, "XTRACK_", AC_PID),

	GGROUP(pid_heading_control, "HEAD_", AC_PID),
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

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS_", AP_InertialSensor),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AC_WPNav),

#if CIRCLE_NAV_ENABLED == ENABLED
    // @Group: CIRCLE_
    // @Path: ../libraries/AC_WPNav/AC_Circle.cpp
    GOBJECT(circle_nav, "CIRCLE_",  AC_Circle),
#endif

    // @Group: ATC_
	// @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    GOBJECT(attitude_control, "ATC_", AC_AttitudeControl_Multi),

    // @Group: PSC
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECT(pos_control, "PSC", AC_PosControl),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs[0],  gcs0,       "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs[1],  gcs1,       "SR1_",     GCS_MAVLINK),

    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs[2],  gcs2,       "SR2_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs[3],  gcs3,       "SR3_",     GCS_MAVLINK),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: LOG
    // @Path: ../libraries/DataFlash/DataFlash.cpp
    GOBJECT(DataFlash,           "LOG",  DataFlash_Class),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

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

    // Leak detector
    // @Group: LEAK
    // @Path: ../libraries/AP_LeakDetector/AP_LeakDetector.cpp
	GOBJECT(leak_detector, "LEAK", AP_LeakDetector),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AC_FENCE == ENABLED
    // @Group: FENCE_
    // @Path: ../libraries/AC_Fence/AC_Fence.cpp
    GOBJECT(fence,      "FENCE_",   AC_Fence),
#endif

#if AVOIDANCE_ENABLED == ENABLED
	// @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
    GOBJECT(avoid,      "AVOID_",   AC_Avoid),
#endif

#if AC_RALLY == ENABLED
    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,      "RALLY_",   AP_Rally),
#endif

//#if (FRAME_CONFIG == VECTORED_FRAME || FRAME_CONFIG == BLUEROV_FRAME || FRAME_CONFIG == VECTORED6DOF_FRAME || FRAME_CONFIG == SIMPLEROV || FRAME_CONFIG == VECTORED90_FRAME)
	// @Group: MOT_
	// @Path: ../libraries/AP_Motors/AP_Motors6DOF.cpp,../libraries/AP_Motors/AP_MotorsMulticopter.cpp
	GOBJECT(motors, "MOT_",         AP_Motors6DOF),

//#else
//    // @Group: MOT_
//    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
//    GOBJECT(motors, "MOT_",         AP_MotorsMulticopter),
//#endif

#if RCMAP_ENABLED == ENABLED
    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),
#endif

    // @Group: EKF_
    // @Path: ../libraries/AP_NavEKF/AP_NavEKF.cpp
    GOBJECTN(EKF, NavEKF, "EKF_", NavEKF),

    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(EKF2, NavEKF2, "EK2_", NavEKF2),
    
    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),
    
#if RANGEFINDER_ENABLED == ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/RangeFinder.cpp
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

#if RPM_ENABLED == ENABLED
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

#if AUTOTUNE_ENABLED == ENABLED
    // @Param: AUTOTUNE_AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Values: 7:All,1:Roll Only,2:Pitch Only,4:Yaw Only,3:Roll and Pitch,5:Roll and Yaw,6:Pitch and Yaw
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    GSCALAR(autotune_axis_bitmask, "AUTOTUNE_AXES", 7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

    // @Param: AUTOTUNE_AGGR
    // @DisplayName: Autotune aggressiveness
    // @Description: Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
    // @Range: 0.05 0.10
    // @User: Standard
    GSCALAR(autotune_aggressiveness, "AUTOTUNE_AGGR", 0.1f),

    // @Param: AUTOTUNE_MIN_D
    // @DisplayName: AutoTune minimum D
    // @Description: Defines the minimum D gain
    // @Range: 0.001 0.006
    // @User: Standard
    GSCALAR(autotune_min_d, "AUTOTUNE_MIN_D", 0.001f),
#endif

	// @Group: NTF_
	// @Path: ../libraries/AP_Notify/AP_Notify.cpp
	GOBJECT(notify, "NTF_",  AP_Notify),

	// @Param: TERRAIN_FOLLOW
	// @DisplayName: Terrain Following use control
	// @Description: This enables terrain following for RTL and LAND flight modes. To use this option TERRAIN_ENABLE must be 1 and the GCS must  support sending terrain data to the aircraft.  In RTL the RTL_ALT will be considered a height above the terrain.  In LAND mode the vehicle will slow to LAND_SPEED 10m above terrain (instead of 10m above home).  This parameter does not affect AUTO and Guided which use a per-command flag to determine if the height is above-home, absolute or above-terrain.
	// @Values: 0:Do Not Use in RTL and Land,1:Use in RTL and Land
	// @User: Standard
	GSCALAR(terrain_follow, "TERRAIN_FOLLOW", 0),

	// @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

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

#if PROXIMITY_ENABLED == ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 2, ParametersG2, AP_Proximity),
#endif

#if GRIPPER_ENABLED == ENABLED
	// @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 3, ParametersG2, AP_Gripper),
#endif

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2(void)
#if PROXIMITY_ENABLED == ENABLED
   : proximity(sub.serial_manager)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Sub::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        cliSerial->printf("Bad var table\n");
        AP_HAL::panic("Bad var table");
    }

    // disable centrifugal force correction, it will be enabled as part of the arming process
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf("Firmware change: erasing EEPROM...\n");
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        cliSerial->println("done.");
    }

    uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    cliSerial->printf("load_all took %uus\n", (unsigned)(micros() - before));
}
