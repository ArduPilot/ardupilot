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
    // @Range: 1 255
    // @Increment: 1
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
    // @Range: 200 300000
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

    // @Param: RTL_ALT_TYPE
    // @DisplayName: RTL mode altitude type
    // @Description: RTL altitude type.  Set to 1 for Terrain following during RTL and then set WPNAV_RFND_USE=1 to use rangefinder or WPNAV_RFND_USE=0 to use Terrain database
    // @Values: 0:Relative to Home, 1:Terrain
    // @User: Standard
    GSCALAR(rtl_alt_type, "RTL_ALT_TYPE", 0),
#endif

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.
    // @Values: 0:Disabled/NoAction,1:RTL,2:RTL or Continue with Mission in Auto Mode (Removed in 4.0+-see FS_OPTIONS),3:SmartRTL or RTL,4:SmartRTL or Land,5:Land,6:Auto DO_LAND_START or RTL,7:Brake or Land
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
    // @Description: Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode. The bitmask is for flight mode switch positions
    // @Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6
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
    // @Values:  0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode (Removed in 4.0+),3:Enabled always Land,4:Enabled always SmartRTL or RTL,5:Enabled always SmartRTL or Land,6:Enabled Auto DO_LAND_START or RTL,7:Enabled always Brake or Land
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_ENABLED_ALWAYS_RTL),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level in microseconds on channel 3 below which throttle failsafe triggers
    // @Range: 910 1100
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
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate,27:Auto RTL
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
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         CH_MODE_DEFAULT),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     (uint8_t)Mode::Number::STABILIZE),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode). The bitmask is for flightmode switch positions.
    // @Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what on-board log types to enable. This value is made up of the sum of each of the log types you want to be saved. It is usually best just to enable all basiclog types by setting this to 65535. 
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Control Tuning,5:Navigation Tuning,6:RC input,7:IMU,8:Mission Commands,9:Battery Monitor,10:RC output,11:Optical Flow,12:PID,13:Compass,15:Camera,17:Motors,18:Fast IMU,19:Raw IMU,20:Video Stabilization,21:Fast harmonic notch logging
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
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,56:Rate Yaw Filter,55:Motor Yaw Headroom,14:AltHold kP,7:Throttle Rate kP,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,12:Loiter Pos kP,22:Velocity XY kP,28:Velocity XY kI,10:WP Speed,25:Acro Roll/Pitch deg/s,40:Acro Yaw deg/s,45:RC Feel,13:Heli Ext Gyro,38:Declination,39:Circle Rate,46:Rate Pitch kP,47:Rate Pitch kI,48:Rate Pitch kD,49:Rate Roll kP,50:Rate Roll kI,51:Rate Roll kD,52:Rate Pitch FF,53:Rate Roll FF,54:Rate Yaw FF,58:SysID Magnitude,59:PSC Angle Max
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X, V, etc)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15: I, 18: BetaFlightXReversed, 19:Y4
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(frame_type, "FRAME_TYPE", HAL_FRAME_TYPE_DEFAULT),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Copter),

    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds after landing touchdown detection. A value of zero disables auto disarm. If Emergency Motor stop active, delay time is half this value.
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),
    
    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

#if MODE_POSHOLD_ENABLED == ENABLED
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
    // @Increment: 10
    // @Range: 2000 4500
    // @User: Advanced
    GSCALAR(poshold_brake_angle_max, "PHLD_BRAKE_ANGLE",  POSHOLD_BRAKE_ANGLE_DEFAULT),
#endif

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
    // @Description: Allows setting the maximum acceptable compass, velocity, position and height variances. Used in arming check and EKF failsafe.
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

    // ACRO_RP_EXPO moved to Command Model class

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     (uint8_t)ModeAcro::Trainer::LIMITED),
#endif

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

#if PARACHUTE == ENABLED
    // @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute, "CHUTE_", AP_Parachute),
#endif

#if AP_LANDINGGEAR_ENABLED
    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    GOBJECT(landinggear,    "LGR_", AP_LandingGear),
#endif

#if FRAME_CONFIG == HELI_FRAME
    // @Group: IM_
    // @Path: ../libraries/AC_InputManager/AC_InputManager_Heli.cpp
    GOBJECT(input_manager, "IM_", AC_InputManager_Heli),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS", AP_InertialSensor),

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
    GOBJECTVARPTR(attitude_control, "ATC_", &copter.attitude_control_var_info),

    // @Group: PSC
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECTPTR(pos_control, "PSC", AC_PosControl),

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

#if HAL_SPRAYER_ENABLED
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    GOBJECT(sprayer,                "SPRAY_",       AC_Sprayer),
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

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
#if AC_AVOID_ENABLED == ENABLED
    GOBJECT(avoid,      "AVOID_",   AC_Avoid),
#endif

#if HAL_RALLY_ENABLED
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

#if AC_PRECLAND_ENABLED
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    GOBJECT(precland, "PLND_", AC_PrecLand),
#endif

#if AP_RPM_ENABLED
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
    GSCALAR(throw_motor_start, "THROW_MOT_START", (float)ModeThrow::PreThrowMotorState::STOPPED),

    // @Param: THROW_ALT_MIN
    // @DisplayName: Throw mode minimum altitude
    // @Description: Minimum altitude above which Throw mode will detect a throw or a drop - 0 to disable the check
    // @Units: m
    // @User: Advanced
    GSCALAR(throw_altitude_min, "THROW_ALT_MIN", 0),

    // @Param: THROW_ALT_MAX
    // @DisplayName: Throw mode maximum altitude
    // @Description: Maximum altitude under which Throw mode will detect a throw or a drop - 0 to disable the check
    // @Units: m
    // @User: Advanced
    GSCALAR(throw_altitude_max, "THROW_ALT_MAX", 0),
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    // @Group: CC
    // @Path: ../libraries/AC_CustomControl/AC_CustomControl.cpp
    GOBJECT(custom_control, "CC", AC_CustomControl),
#endif

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

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

#if HAL_BUTTON_ENABLED
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
    AP_GROUPINFO("THROW_TYPE", 4, ParametersG2, throw_type, (float)ModeThrow::ThrowType::Upward),
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
    // @Bitmask: 0:ADSBMavlinkProcessing,1:DevOptionVFR_HUDRelativeAlt
    // @User: Advanced
    AP_GROUPINFO("DEV_OPTIONS", 7, ParametersG2, dev_options, 0),

#if AP_BEACON_ENABLED
    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 14, ParametersG2, AP_Beacon),
#endif

#if HAL_PROXIMITY_ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 8, ParametersG2, AP_Proximity),
#endif

    // ACRO_Y_EXPO (9) moved to Command Model Class

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

    // 12 was AP_Stats

#if AP_GRIPPER_ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 13, ParametersG2, AP_Gripper),
#endif

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 6:Heli, 7:Tri, 8:SingleCopter, 9:CoaxCopter, 10:BiCopter, 11:Heli_Dual, 12:DodecaHexa, 13:HeliQuad, 14:Deca, 15:Scripting Matrix, 16:6DoF Scripting, 17:Dynamic Scripting Matrix
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

#if AP_TEMPCALIBRATION_ENABLED
    // @Group: TCAL
    // @Path: ../libraries/AP_TempCalibration/AP_TempCalibration.cpp
    AP_SUBGROUPINFO(temp_calibration, "TCAL", 19, ParametersG2, AP_TempCalibration),
#endif

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

#if AP_WINCH_ENABLED
    // 22 was AP_WheelEncoder

    // @Group: WINCH
    // @Path: ../libraries/AP_Winch/AP_Winch.cpp
    AP_SUBGROUPINFO(winch, "WINCH", 23, ParametersG2, AP_Winch),
#endif

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical speed descending
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s.  If 0 PILOT_SPEED_UP value is used.
    // @Units: cm/s
    // @Range: 0 500
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

#if MODE_FLOWHOLD_ENABLED == ENABLED
    // @Group: FHLD
    // @Path: mode_flowhold.cpp
    AP_SUBGROUPPTR(mode_flowhold_ptr, "FHLD", 26, ParametersG2, ModeFlowHold),
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 27, ParametersG2, AP_Follow),
#endif

#if USER_PARAMS_ENABLED == ENABLED
    AP_SUBGROUPINFO(user_parameters, "USR", 28, ParametersG2, UserParameters),
#endif

#if AUTOTUNE_ENABLED == ENABLED
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp,../libraries/AC_AutoTune/AC_AutoTune_Heli.cpp
    AP_SUBGROUPPTR(autotune_ptr, "AUTOTUNE_",  29, ParametersG2, AutoTune),
#endif

    // 30 was AP_Scripting

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
    AP_GROUPINFO("FS_OPTIONS", 36, ParametersG2, fs_options, (float)Copter::FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL),

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
    // @Description: A range of options that can be applied to change auto mode behaviour. Allow Arming allows the copter to be armed in Auto. Allow Takeoff Without Raising Throttle allows takeoff without the pilot having to raise the throttle. Ignore pilot yaw overrides the pilot's yaw stick being used while in auto.
    // @Bitmask: 0:Allow Arming,1:Allow Takeoff Without Raising Throttle,2:Ignore pilot yaw,7:Allow weathervaning
    // @User: Advanced
    AP_GROUPINFO("AUTO_OPTIONS", 40, ParametersG2, auto_options, 0),
#endif

#if MODE_GUIDED_ENABLED == ENABLED
    // @Param: GUID_OPTIONS
    // @DisplayName: Guided mode options
    // @Description: Options that can be applied to change guided mode behaviour
    // @Bitmask: 0:Allow Arming from Transmitter,2:Ignore pilot yaw,3:SetAttitudeTarget interprets Thrust As Thrust,4:Do not stabilize PositionXY,5:Do not stabilize VelocityXY,6:Waypoint navigation used for position targets,7:Allow weathervaning
    // @User: Advanced
    AP_GROUPINFO("GUID_OPTIONS", 41, ParametersG2, guided_options, 0),
#endif

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 42, ParametersG2, fs_gcs_timeout, 5),

#if MODE_RTL_ENABLED == ENABLED
    // @Param: RTL_OPTIONS
    // @DisplayName: RTL mode options
    // @Description: Options that can be applied to change RTL mode behaviour
    // @Bitmask: 2:Ignore pilot yaw
    // @User: Advanced
    AP_GROUPINFO("RTL_OPTIONS", 43, ParametersG2, rtl_options, 0),
#endif

    // @Param: FLIGHT_OPTIONS
    // @DisplayName: Flight mode options
    // @Description: Flight mode specific options
    // @Bitmask: 0:Disable thrust loss check, 1:Disable yaw imbalance warning, 2:Release gripper on thrust loss, 3:Use a separate rate loop thread
    // @User: Advanced
    AP_GROUPINFO("FLIGHT_OPTIONS", 44, ParametersG2, flight_options, 0),

#if RANGEFINDER_ENABLED == ENABLED
    // @Param: RNGFND_FILT
    // @DisplayName: Rangefinder filter
    // @Description: Rangefinder filter to smooth distance.  Set to zero to disable filtering
    // @Units: Hz
    // @Range: 0 5
    // @Increment: 0.05
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RNGFND_FILT", 45, ParametersG2, rangefinder_filt, RANGEFINDER_FILT_DEFAULT),
#endif

#if MODE_GUIDED_ENABLED == ENABLED
    // @Param: GUID_TIMEOUT
    // @DisplayName: Guided mode timeout
    // @Description: Guided mode timeout after which vehicle will stop or return to level if no updates are received from caller. Only applicable during any combination of velocity, acceleration, angle control, and/or angular rate control
    // @Units: s
    // @Range: 0.1 5
    // @User: Advanced
    AP_GROUPINFO("GUID_TIMEOUT", 46, ParametersG2, guided_timeout, 3.0),
#endif

    // ACRO_PR_RATE (47), ACRO_Y_RATE (48), PILOT_Y_RATE (49) and PILOT_Y_EXPO (50) moved to command model class

    // @Param: SURFTRAK_MODE
    // @DisplayName: Surface Tracking Mode
    // @Description: set which surface to track in surface tracking
    // @Values: 0:Do not track, 1:Ground, 2:Ceiling
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SURFTRAK_MODE", 51, ParametersG2, surftrak_mode, (uint8_t)Copter::SurfaceTracking::Surface::GROUND),

    // @Param: FS_DR_ENABLE
    // @DisplayName: DeadReckon Failsafe Action
    // @Description: Failsafe action taken immediately as deadreckoning starts. Deadreckoning starts when EKF loses position and velocity source and relies only on wind estimates
    // @Values: 0:Disabled/NoAction,1:Land, 2:RTL, 3:SmartRTL or RTL, 4:SmartRTL or Land, 6:Auto DO_LAND_START or RTL
    // @User: Standard
    AP_GROUPINFO("FS_DR_ENABLE", 52, ParametersG2, failsafe_dr_enable, (uint8_t)Copter::FailsafeAction::RTL),

    // @Param: FS_DR_TIMEOUT
    // @DisplayName: DeadReckon Failsafe Timeout
    // @Description: DeadReckoning is available for this many seconds after losing position and/or velocity source.  After this timeout elapses the EKF failsafe will trigger in modes requiring a position estimate
    // @Range: 0 120
    // @User: Standard
    AP_GROUPINFO("FS_DR_TIMEOUT", 53, ParametersG2, failsafe_dr_timeout, 30),

#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    // @Param: ACRO_RP_RATE
    // @DisplayName: Acro Roll and Pitch Rate
    // @Description: Acro mode maximum roll and pitch rate.  Higher values mean faster rate of rotation
    // @Units: deg/s
    // @Range: 1 1080
    // @User: Standard

    // @Param: ACRO_RP_EXPO
    // @DisplayName: Acro Roll/Pitch Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 0.95
    // @User: Advanced

    // @Param: ACRO_RP_RATE_TC
    // @DisplayName: Acro roll/pitch rate control input time constant
    // @Description: Acro roll and pitch rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_acro_rp, "ACRO_RP_", 54, ParametersG2, AC_CommandModel),
#endif

#if MODE_ACRO_ENABLED == ENABLED || MODE_DRIFT_ENABLED == ENABLED
    // @Param: ACRO_Y_RATE
    // @DisplayName: Acro Yaw Rate
    // @Description: Acro mode maximum yaw rate.  Higher value means faster rate of rotation
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard

    // @Param: ACRO_Y_EXPO
    // @DisplayName: Acro Yaw Expo
    // @Description: Acro yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -1.0 0.95
    // @User: Advanced

    // @Param: ACRO_Y_RATE_TC
    // @DisplayName: Acro yaw rate control input time constant
    // @Description: Acro yaw rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_acro_y, "ACRO_Y_", 55, ParametersG2, AC_CommandModel),
#endif

    // @Param: PILOT_Y_RATE
    // @DisplayName: Pilot controlled yaw rate
    // @Description: Pilot controlled yaw rate max.  Used in all pilot controlled modes except Acro
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard

    // @Param: PILOT_Y_EXPO
    // @DisplayName: Pilot controlled yaw expo
    // @Description: Pilot controlled yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced

    // @Param: PILOT_Y_RATE_TC
    // @DisplayName: Pilot yaw rate control input time constant
    // @Description: Pilot yaw rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_pilot, "PILOT_Y_", 56, ParametersG2, AC_CommandModel),

    // @Param: TKOFF_SLEW_TIME
    // @DisplayName: Slew time of throttle during take-off
    // @Description: Time to slew the throttle from minimum to maximum while checking for a succsessful takeoff.
    // @Units: s
    // @Range: 0.25 5.0
    // @User: Standard
    AP_GROUPINFO("TKOFF_SLEW_TIME", 57, ParametersG2, takeoff_throttle_slew_time, 2.0),

#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // @Param: TKOFF_RPM_MIN
    // @DisplayName: Takeoff Check RPM minimum
    // @Description: Takeoff is not permitted until motors report at least this RPM.  Set to zero to disable check
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("TKOFF_RPM_MIN", 58, ParametersG2, takeoff_rpm_min, 0),
#endif

#if WEATHERVANE_ENABLED == ENABLED
    // @Group: WVANE_
    // @Path: ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp
    AP_SUBGROUPINFO(weathervane, "WVANE_", 59, ParametersG2, AC_WeatherVane),
#endif

    // ID 60 is reserved for the SHIP_OPS

    // extend to a new group
    AP_SUBGROUPEXTENSION("", 61, ParametersG2, var_info2),

    // ID 62 is reserved for the SHOW_... parameters from the Skybrush fork at
    // https://github.com/skybrush-io/ardupilot

    AP_GROUPEND
};

/*
  extension to g2 parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info2[] = {

    // @Param: PLDP_THRESH
    // @DisplayName: Payload Place thrust ratio threshold
    // @Description: Ratio of vertical thrust during decent below which payload touchdown will trigger.
    // @Range: 0.5 0.9
    // @User: Standard
    AP_GROUPINFO("PLDP_THRESH", 1, ParametersG2, pldp_thrust_placed_fraction, 0.9),

    // @Param: PLDP_RNG_MIN
    // @DisplayName: Payload Place minimum range finder altitude
    // @Description: Minimum range finder altitude in m to trigger payload touchdown, set to zero to disable.
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PLDP_RNG_MIN", 2, ParametersG2, pldp_range_finder_minimum_m, 0.0),

    // @Param: PLDP_DELAY
    // @DisplayName: Payload Place climb delay
    // @Description: Delay after release, in seconds, before aircraft starts to climb back to starting altitude.
    // @Units: s
    // @Range: 0 120
    // @User: Standard
    AP_GROUPINFO("PLDP_DELAY", 3, ParametersG2, pldp_delay_s, 0.0),

    // @Param: PLDP_SPEED_DN
    // @DisplayName: Payload Place decent speed
    // @Description: The maximum vertical decent velocity in m/s. If 0 LAND_SPEED value is used.
    // @Units: m/s
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PLDP_SPEED_DN", 4, ParametersG2, pldp_descent_speed_ms, 0.0),

    // @Param: SURFTRAK_TC
    // @DisplayName: Surface Tracking Filter Time Constant
    // @Description: Time to achieve 63.2% of the surface altitude measurement change.  If 0 filtering is disabled
    // @Units: s
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("SURFTRAK_TC", 5, ParametersG2, surftrak_tc, 1.0),

    // @Param: TKOFF_THR_MAX
    // @DisplayName: Takeoff maximum throttle during take-off ramp up
    // @Description: Takeoff maximum throttle allowed before controllers assume the aircraft is airborne during the takeoff process.
    // @Range: 0.0 0.9
    // @User: Advanced
    AP_GROUPINFO("TKOFF_THR_MAX", 6, ParametersG2, takeoff_throttle_max, 0.9),

#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // @Param: TKOFF_RPM_MAX
    // @DisplayName: Takeoff Check RPM maximum
    // @Description: Takeoff is not permitted until motors report no more than this RPM.  Set to zero to disable check
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("TKOFF_RPM_MAX", 7, ParametersG2, takeoff_rpm_max, 0),
#endif

    // @Param: FS_EKF_FILT
    // @DisplayName: EKF Failsafe filter cutoff
    // @Description: EKF Failsafe filter cutoff frequency. EKF variances are filtered using this value to avoid spurious failsafes from transient high variances. A higher value means the failsafe is more likely to trigger.
    // @Range: 0 10
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FS_EKF_FILT", 8, ParametersG2, fs_ekf_filt_hz, FS_EKF_FILT_DEFAULT),

    // ID 62 is reserved for the AP_SUBGROUPEXTENSION

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2(void)
    : command_model_pilot(PILOT_Y_RATE_DEFAULT, PILOT_Y_EXPO_DEFAULT, 0.0f)
#if AP_TEMPCALIBRATION_ENABLED
    , temp_calibration()
#endif
#if AP_BEACON_ENABLED
    , beacon()
#endif
#if HAL_PROXIMITY_ENABLED
    , proximity()
#endif
#if ADVANCED_FAILSAFE == ENABLED
    ,afs()
#endif
#if MODE_SMARTRTL_ENABLED == ENABLED
    ,smart_rtl()
#endif
#if MODE_FLOWHOLD_ENABLED == ENABLED
    ,mode_flowhold_ptr(&copter.mode_flowhold)
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    ,follow()
#endif
#if USER_PARAMS_ENABLED == ENABLED
    ,user_parameters()
#endif
#if AUTOTUNE_ENABLED == ENABLED
    ,autotune_ptr(&copter.mode_autotune.autotune)
#endif
#if MODE_SYSTEMID_ENABLED == ENABLED
    ,mode_systemid_ptr(&copter.mode_systemid)
#endif
#if MODE_AUTOROTATE_ENABLED == ENABLED
    ,arot()
#endif
#if HAL_BUTTON_ENABLED
    ,button_ptr(&copter.button)
#endif
#if MODE_ZIGZAG_ENABLED == ENABLED
    ,mode_zigzag_ptr(&copter.mode_zigzag)
#endif

#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    ,command_model_acro_rp(ACRO_RP_RATE_DEFAULT, ACRO_RP_EXPO_DEFAULT, 0.0f)
#endif

#if MODE_ACRO_ENABLED == ENABLED || MODE_DRIFT_ENABLED == ENABLED
    ,command_model_acro_y(ACRO_Y_RATE_DEFAULT, ACRO_Y_EXPO_DEFAULT, 0.0f)
#endif

#if WEATHERVANE_ENABLED == ENABLED
    ,weathervane()
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
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
    // PARAMETER_CONVERSION - Added: Jan-2017
    { Parameters::k_param_arming_check_old,   0,      AP_PARAM_INT8,  "ARMING_CHECK" },
    // battery
    // PARAMETER_CONVERSION - Added: Mar-2018
    { Parameters::k_param_fs_batt_voltage,    0,      AP_PARAM_FLOAT,  "BATT_LOW_VOLT" },
    { Parameters::k_param_fs_batt_mah,        0,      AP_PARAM_FLOAT,  "BATT_LOW_MAH" },
    { Parameters::k_param_failsafe_battery_enabled,0, AP_PARAM_INT8,   "BATT_FS_LOW_ACT" },

    // PARAMETER_CONVERSION - Added: Aug-2018
    { Parameters::Parameters::k_param_ch7_option_old,   0,      AP_PARAM_INT8,  "RC7_OPTION" },
    { Parameters::Parameters::k_param_ch8_option_old,   0,      AP_PARAM_INT8,  "RC8_OPTION" },
    { Parameters::Parameters::k_param_ch9_option_old,   0,      AP_PARAM_INT8,  "RC9_OPTION" },
    { Parameters::Parameters::k_param_ch10_option_old,   0,      AP_PARAM_INT8,  "RC10_OPTION" },
    { Parameters::Parameters::k_param_ch11_option_old,   0,      AP_PARAM_INT8,  "RC11_OPTION" },
    { Parameters::Parameters::k_param_ch12_option_old,   0,      AP_PARAM_INT8,  "RC12_OPTION" },
    // PARAMETER_CONVERSION - Added: Apr-2019
    { Parameters::k_param_compass_enabled_deprecated,    0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    // PARAMETER_CONVERSION - Added: Jul-2019
    { Parameters::k_param_arming,             2,     AP_PARAM_INT16,  "ARMING_CHECK" },
};

void Copter::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

#if AP_LANDINGGEAR_ENABLED
    // convert landing gear parameters
    // PARAMETER_CONVERSION - Added: Nov-2018
    convert_lgr_parameters();
#endif

#if MODE_RTL_ENABLED == ENABLED
    // PARAMETER_CONVERSION - Added: Sep-2021
    g.rtl_altitude.convert_parameter_width(AP_PARAM_INT16);
#endif

    // PARAMETER_CONVERSION - Added: Mar-2022
#if AP_FENCE_ENABLED
    AP_Param::convert_class(g.k_param_fence_old, &fence, fence.var_info, 0, 0, true);
#endif

    // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
#if AP_STATS_ENABLED
    {
        // Find G2's Top Level Key
        AP_Param::ConversionInfo info;
        if (!AP_Param::find_top_level_key_by_pointer(&g2, info.old_key)) {
            return;
        }

        const uint16_t old_index = 12;       // Old parameter index in g2
        const uint16_t old_top_element = 4044; // Old group element in the tree for the first subgroup element (see AP_PARAM_KEY_DUMP)
        AP_Param::convert_class(info.old_key, &stats, stats.var_info, old_index, old_top_element, false);
    }
#endif
    // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
#if AP_SCRIPTING_ENABLED
    {
        // Find G2's Top Level Key
        AP_Param::ConversionInfo info;
        if (!AP_Param::find_top_level_key_by_pointer(&g2, info.old_key)) {
            return;
        }

        const uint16_t old_index = 30;       // Old parameter index in g2
        const uint16_t old_top_element = 94; // Old group element in the tree for the first subgroup element (see AP_PARAM_KEY_DUMP)
        AP_Param::convert_class(info.old_key, &scripting, scripting.var_info, old_index, old_top_element, false);
    }
#endif

    // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, 0, true);
#endif

    // setup AP_Param frame type flags
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_COPTER);
}

// handle conversion of PID gains
void Copter::convert_pid_parameters(void)
{
    const AP_Param::ConversionInfo angle_and_filt_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_pid_rate_yaw, 6, AP_PARAM_FLOAT, "ATC_RAT_YAW_FILT" },
        { Parameters::k_param_pi_vel_xy, 0, AP_PARAM_FLOAT, "PSC_VELXY_P" },
        { Parameters::k_param_pi_vel_xy, 1, AP_PARAM_FLOAT, "PSC_VELXY_I" },
        { Parameters::k_param_pi_vel_xy, 2, AP_PARAM_FLOAT, "PSC_VELXY_IMAX" },
        // PARAMETER_CONVERSION - Added: Aug-2021
        { Parameters::k_param_pi_vel_xy, 3, AP_PARAM_FLOAT, "PSC_VELXY_FLTE" },
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_p_vel_z, 0, AP_PARAM_FLOAT, "PSC_VELZ_P" },
        { Parameters::k_param_pid_accel_z, 0, AP_PARAM_FLOAT, "PSC_ACCZ_P" },
        { Parameters::k_param_pid_accel_z, 1, AP_PARAM_FLOAT, "PSC_ACCZ_I" },
        { Parameters::k_param_pid_accel_z, 2, AP_PARAM_FLOAT, "PSC_ACCZ_D" },
        { Parameters::k_param_pid_accel_z, 5, AP_PARAM_FLOAT, "PSC_ACCZ_IMAX" },
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_pid_accel_z, 6, AP_PARAM_FLOAT, "PSC_ACCZ_FLTE" },
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_p_alt_hold, 0, AP_PARAM_FLOAT, "PSC_POSZ_P" },
        { Parameters::k_param_p_pos_xy, 0, AP_PARAM_FLOAT, "PSC_POSXY_P" },
    };
    const AP_Param::ConversionInfo loiter_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Apr-2018
        { Parameters::k_param_wp_nav, 4, AP_PARAM_FLOAT, "LOIT_SPEED" },
        { Parameters::k_param_wp_nav, 7, AP_PARAM_FLOAT, "LOIT_BRK_JERK" },
        { Parameters::k_param_wp_nav, 8, AP_PARAM_FLOAT, "LOIT_ACC_MAX" },
        { Parameters::k_param_wp_nav, 9, AP_PARAM_FLOAT, "LOIT_BRK_ACCEL" }
    };

    // convert angle controller gain and filter without scaling
    for (const auto &info : angle_and_filt_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0f);
    }
    // convert RC_FEEL_RP to ATC_INPUT_TC
    // PARAMETER_CONVERSION - Added: Mar-2018
    const AP_Param::ConversionInfo rc_feel_rp_conversion_info = { Parameters::k_param_rc_feel_rp, 0, AP_PARAM_INT8, "ATC_INPUT_TC" };
    AP_Int8 rc_feel_rp_old;
    if (AP_Param::find_old_parameter(&rc_feel_rp_conversion_info, &rc_feel_rp_old)) {
        AP_Param::set_default_by_name(rc_feel_rp_conversion_info.new_name, (1.0f / (2.0f + rc_feel_rp_old.get() * 0.1f)));
    }
    // convert loiter parameters
    for (const auto &info : loiter_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0f);
    }

    // TradHeli default parameters
#if FRAME_CONFIG == HELI_FRAME
    static const struct AP_Param::defaults_table_struct heli_defaults_table[] = {
        // PARAMETER_CONVERSION - Added: Nov-2018
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
        // PARAMETER_CONVERSION - Added: Jan-2019
        { "RC8_OPTION", 32 },
        // PARAMETER_CONVERSION - Added: Aug-2018
        { "RC_OPTIONS", 0 },
        // PARAMETER_CONVERSION - Added: Feb-2022
        { "ATC_RAT_RLL_ILMI", 0.05},
        { "ATC_RAT_PIT_ILMI", 0.05},
    };
    AP_Param::set_defaults_from_table(heli_defaults_table, ARRAY_SIZE(heli_defaults_table));
#endif

    // attitude and position control filter parameter changes (from _FILT to FLTD, FLTE, FLTT) for Copter-4.0
    // magic numbers shown below are discovered by setting AP_PARAM_KEY_DUMP = 1
    const AP_Param::ConversionInfo ff_and_filt_conversion_info[] = {
#if FRAME_CONFIG == HELI_FRAME
        // tradheli moves ATC_RAT_RLL/PIT_FILT to FLTE, ATC_RAT_YAW_FILT to FLTE
        // PARAMETER_CONVERSION - Added: Jul-2019
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTE" },
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTE" },
        { Parameters::k_param_attitude_control, 388, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
#else
        // multicopters move ATC_RAT_RLL/PIT_FILT to FLTD & FLTT, ATC_RAT_YAW_FILT to FLTE
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTD" },
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTT" },
        // PARAMETER_CONVERSION - Added: Jul-2019
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTD" },
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTT" },
        // PARAMETER_CONVERSION - Added: Jul-2019
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
        { Parameters::k_param_attitude_control, 449, AP_PARAM_FLOAT, "ATC_RAT_RLL_FF" },
        { Parameters::k_param_attitude_control, 450, AP_PARAM_FLOAT, "ATC_RAT_PIT_FF" },
        { Parameters::k_param_attitude_control, 451, AP_PARAM_FLOAT, "ATC_RAT_YAW_FF" },
#endif
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_pos_control, 388, AP_PARAM_FLOAT, "PSC_ACCZ_FLTE" },
    };
    AP_Param::convert_old_parameters(&ff_and_filt_conversion_info[0], ARRAY_SIZE(ff_and_filt_conversion_info));

#if HAL_INS_NUM_HARMONIC_NOTCH_FILTERS > 1
    if (!ins.harmonic_notches[1].params.enabled()) {
        // notch filter parameter conversions (moved to INS_HNTC2) for 4.2.x, converted from fixed notch
        const AP_Param::ConversionInfo notchfilt_conversion_info[] {
            // PARAMETER_CONVERSION - Added: Apr 2022
            { Parameters::k_param_ins, 101, AP_PARAM_INT8,  "INS_HNTC2_ENABLE" },
            { Parameters::k_param_ins, 293, AP_PARAM_FLOAT, "INS_HNTC2_ATT" },
            { Parameters::k_param_ins, 357, AP_PARAM_FLOAT, "INS_HNTC2_FREQ" },
            { Parameters::k_param_ins, 421, AP_PARAM_FLOAT, "INS_HNTC2_BW" },
        };
        AP_Param::convert_old_parameters(&notchfilt_conversion_info[0], ARRAY_SIZE(notchfilt_conversion_info));
        AP_Param::set_default_by_name("INS_HNTC2_MODE", 0);
        AP_Param::set_default_by_name("INS_HNTC2_HMNCS", 1);
    }
#endif

    // ACRO_RP_P and ACRO_Y_P replaced with ACRO_RP_RATE and ACRO_Y_RATE for Copter-4.2
    // PARAMETER_CONVERSION - Added: Sep-2021
    const AP_Param::ConversionInfo acro_rpy_conversion_info[] = {
        { Parameters::k_param_acro_rp_p, 0, AP_PARAM_FLOAT, "ACRO_RP_RATE" },
        { Parameters::k_param_acro_yaw_p,  0, AP_PARAM_FLOAT, "ACRO_Y_RATE" }
    };
    for (const auto &info : acro_rpy_conversion_info) {
        AP_Param::convert_old_parameter(&info, 45.0);
    }

    // convert rate and expo command model parameters for Copter-4.3
    // PARAMETER_CONVERSION - Added: June-2022
    const AP_Param::ConversionInfo cmd_mdl_conversion_info[] = {
        { Parameters::k_param_g2, 47, AP_PARAM_FLOAT, "ACRO_RP_RATE" },
        { Parameters::k_param_acro_rp_expo,  0, AP_PARAM_FLOAT, "ACRO_RP_EXPO" },
        { Parameters::k_param_g2,  48, AP_PARAM_FLOAT, "ACRO_Y_RATE" },
        { Parameters::k_param_g2,  9, AP_PARAM_FLOAT, "ACRO_Y_EXPO" },
        { Parameters::k_param_g2,  49, AP_PARAM_FLOAT, "PILOT_Y_RATE" },
        { Parameters::k_param_g2,  50, AP_PARAM_FLOAT, "PILOT_Y_EXPO" },
    };
    for (const auto &info : cmd_mdl_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0);
    }

    // make any SRV_Channel upgrades needed
    SRV_Channels::upgrade_parameters();
}

#if HAL_PROXIMITY_ENABLED
void Copter::convert_prx_parameters()
{
    // convert PRX to PRX1_ parameters for Copter-4.3
    // PARAMETER_CONVERSION - Added: Aug-2022
    const AP_Param::ConversionInfo prx_conversion_info[] = {
        { Parameters::k_param_g2, 72, AP_PARAM_INT8, "PRX1_TYPE" },
        { Parameters::k_param_g2, 136, AP_PARAM_INT8, "PRX1_ORIENT" },
        { Parameters::k_param_g2, 200, AP_PARAM_INT16, "PRX1_YAW_CORR" },
        { Parameters::k_param_g2, 264, AP_PARAM_INT16, "PRX1_IGN_ANG1" },
        { Parameters::k_param_g2, 328, AP_PARAM_INT8, "PRX1_IGN_WID1" },
        { Parameters::k_param_g2, 392, AP_PARAM_INT16, "PRX1_IGN_ANG2" },
        { Parameters::k_param_g2, 456, AP_PARAM_INT8, "PRX1_IGN_WID2" },
        { Parameters::k_param_g2, 520, AP_PARAM_INT16, "PRX1_IGN_ANG3" },
        { Parameters::k_param_g2, 584, AP_PARAM_INT8, "PRX1_IGN_WID3" },
        { Parameters::k_param_g2, 648, AP_PARAM_INT16, "PRX1_IGN_ANG4" },
        { Parameters::k_param_g2, 712, AP_PARAM_INT8, "PRX1_IGN_WID4" },
        { Parameters::k_param_g2, 1224, AP_PARAM_FLOAT, "PRX1_MIN" },
        { Parameters::k_param_g2, 1288, AP_PARAM_FLOAT, "PRX1_MAX" },
    };
    for (const auto &info : prx_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0);
    }
}
#endif

#if AP_LANDINGGEAR_ENABLED
/*
  convert landing gear parameters
 */
void Copter::convert_lgr_parameters(void)
{
    // PARAMETER_CONVERSION - Added: Nov-2018

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
    if (servo_min->configured() ||
        servo_max->configured() ||
        servo_trim->configured() ||
        servo_reversed->configured()) {
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
#endif

#if FRAME_CONFIG == HELI_FRAME
// handle conversion of tradheli parameters from Copter-3.6 to Copter-3.7
void Copter::convert_tradheli_parameters(void) const
{
        // PARAMETER_CONVERSION - Added: Mar-2019
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
        // PARAMETER_CONVERSION - Added: Sep-2019
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
                    if (!ap2->configured()) {
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
        // PARAMETER_CONVERSION - Added: Mar-2019
            { Parameters::k_param_motors, 4, AP_PARAM_INT16, "H_SW2_H3_SV1_POS" },
            { Parameters::k_param_motors, 5, AP_PARAM_INT16, "H_SW2_H3_SV2_POS" },
            { Parameters::k_param_motors, 6, AP_PARAM_INT16, "H_SW2_H3_SV3_POS" },
        // PARAMETER_CONVERSION - Added: Sep-2019
            { Parameters::k_param_motors, 7, AP_PARAM_INT16, "H_SW_H3_PHANG" },
        // PARAMETER_CONVERSION - Added: Mar-2019
            { Parameters::k_param_motors, 8, AP_PARAM_INT16, "H_SW2_H3_PHANG" },
        // PARAMETER_CONVERSION - Added: Sep-2019
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW_COL_DIR" },
        // PARAMETER_CONVERSION - Added: Mar-2019
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW2_COL_DIR" },
        };

        // convert dual heli parameters without scaling
        uint8_t table_size = ARRAY_SIZE(dualheli_conversion_info);
        for (uint8_t i=0; i<table_size; i++) {
            AP_Param::convert_old_parameter(&dualheli_conversion_info[i], 1.0f);
        }


        // PARAMETER_CONVERSION - Added: Sep-2019

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
                if (!ap2->configured()) {
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
                if (!ap2->configured()) {
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

    // PARAMETER_CONVERSION - Added: Dec-2019
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
