//
// Example and reference ArduPilot Mega configuration file
// =======================================================
//
// This file contains documentation and examples for configuration options
// supported by the ArduPilot Mega software.
//
// Most of these options are just that - optional.  You should create
// the APM_Config.h file and use this file as a reference for options
// that you want to change.  Don't copy this file directly; the options
// described here and their default values may change over time.
//
// Each item is marked with a keyword describing when you should set it:
//
// REQUIRED
//          You must configure this in your APM_Config.h file.  The
//          software will not compile if the option is not set.
//
// OPTIONAL
//          The option has a sensible default (which will be described
//          here), but you may wish to override it.
//
// EXPERIMENTAL
//          You should configure this option unless you are prepared
//          to deal with potential problems.  It may relate to a feature
//          still in development, or which is not yet adequately tested.
//
// DEBUG
//          The option should only be set if you are debugging the
//          software, or if you are gathering information for a bug
//          report.
//
// NOTE:
//   Many of these settings are considered 'factory defaults', and the
//   live value is stored and managed in the ArduPilot Mega EEPROM.
//   Use the setup 'factoryreset' command after changing options in
//   your APM_Config.h file.
//
// Units
// -----
//
// Unless indicated otherwise, numeric quantities use the following units:
//
// Measurement | Unit
// ------------+-------------------------------------
// angle       | degrees
// distance    | metres
// speed       | metres per second
// servo angle | microseconds
// voltage     | volts
// times       | seconds
// throttle    | percent
//

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// GPS_PROTOCOL                             REQUIRED
//
// GPS configuration, must be one of:
//
// GPS_PROTOCOL_NONE        No GPS attached
// GPS_PROTOCOL_IMU         X-Plane interface or ArduPilot IMU.
// GPS_PROTOCOL_MTK         MediaTek-based GPS.
// GPS_PROTOCOL_UBLOX       UBLOX GPS
// GPS_PROTOCOL_SIRF        SiRF-based GPS in Binary mode.  NOT TESTED
// GPS_PROTOCOL_NMEA        Standard NMEA GPS.      NOT SUPPORTED (yet?)
//
#define GPS_PROTOCOL  GPS_PROTOCOL_UBLOX
//

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_SENSOR                          OPTIONAL
// AIRSPEED_RATIO                           OPTIONAL
//
// Set AIRSPEED_SENSOR to ENABLED if you have an airspeed sensor attached.
// Adjust AIRSPEED_RATIO in small increments to calibrate the airspeed
// sensor relative to your GPS.  The calculation and default value are optimized for speeds around 12 m/s
//
// The default assumes that an airspeed sensor is connected.
//
#define AIRSPEED_SENSOR     ENABLED
#define AIRSPEED_RATIO      1.9936
//
#define MAGNETOMETER		1
#define USE_MAGNETOMETER	ENABLED

//////////////////////////////////////////////////////////////////////////////
// GCS_PROTOCOL                             OPTIONAL
// GCS_PORT                                 OPTIONAL
//
// The GCS_PROTOCOL option determines which (if any) ground control station
// protocol will be used.  Must be one of:
//
// GCS_PROTOCOL_NONE        No GCS output
// GCS_PROTOCOL_STANDARD    standard APM protocol
// GCS_PROTOCOL_SPECIAL     special test protocol (?)
// GCS_PROTOCOL_LEGACY      legacy ArduPilot protocol
// GCS_PROTOCOL_XPLANE      HIL simulation ground station
// GCS_PROTOCOL_IMU         ArdiPilot IMU output
// GCS_PROTOCOL_JASON       Jason's special secret GCS protocol
// GCS_PROTOCOL_DEBUGTERMINAL  In-flight debug console (experimental)
//
// The GCS_PORT option determines which serial port will be used by the
// GCS protocol.   The usual values are 0 for the console/USB port,
// or 3 for the telemetry port on the oilpan.  Note that some protocols
// will ignore this value and always use the console port.
//
// The default GCS protocol is the standard ArduPilot Mega protocol.
//
// The default serial port is the telemetry port for GCS_PROTOCOL_STANDARD
// and GCS_PROTOCOL_LEGACY.  For all other protocols, the default serial
// port is the FTDI/console port.  GCS_PORT normally should not be set
// in your configuration.
//
#define GCS_PROTOCOL        GCS_PROTOCOL_XDIY
#define GCS_PORT            0
//

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
// SERIAL0_BAUD                             OPTIONAL
//
// Baudrate for the console port.  Default is 38400bps.
//
// SERIAL3_BAUD                             OPTIONAL
//
// Baudrate for the telemetry port.  Default is 115200bps.
//
//#define SERIAL0_BAUD        38400
//#define SERIAL3_BAUD        115200
//
                                
//////////////////////////////////////////////////////////////////////////////
// Battery monitoring                       OPTIONAL
//
// See the manual for details on selecting divider resistors for battery
// monitoring via the oilpan.
//
// BATTERY_EVENT                            OPTIONAL
//
// Set BATTERY_EVENT to ENABLED to enable battery monitoring.  The default is 
// DISABLED.
//
// BATTERY_TYPE                             OPTIONAL if BATTERY_EVENT is set
//
// Set to 0 for 3s LiPo, 1 for 4s LiPo.  The default is 0, selecting a 3s
// battery.
//
// LOW_VOLTAGE                              OPTIONAL if BATTERY_EVENT is set.
//
// Normally derived from BATTERY_TYPE, the automatic value can be overridden
// here.  Value in volts at which ArduPilot Mega should consider the
// battery to be "low".
//
// VOLT_DIV_RATIO                           OPTIONAL
//
// See the manual for details.  The default value corresponds to the resistors
// recommended by the manual.
//
//#define BATTERY_EVENT       DISABLED
//#define BATTERY_TYPE        0
//#define LOW_VOLTAGE         11.4
//#define VOLT_DIV_RATIO      3.0
//

//////////////////////////////////////////////////////////////////////////////
// INPUT_VOLTAGE                            OPTIONAL
//
// In order to have accurate pressure and battery voltage readings, this
// value should be set to the voltage measured on the 5V rail on the oilpan.
//
// See the manual for more details.  The default value should be close.
//
#define INPUT_VOLTAGE 5.0
//


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE								OPTIONAL
// FLIGHT_MODE_CHANNEL			    		OPTIONAL
//
// Flight modes assigned to the control channel, and the input channel that
// is read for the control mode.
//
// Use a servo tester, or the ArduPilotMega_demo test program to check your
// switch settings.
//
// ATTENTION: Some ArduPilot Mega boards have radio channels marked 0-7, and
// others have them marked the standard 1-8.  The FLIGHT_MODE_CHANNEL option
// uses channel numbers 1-8 (and defaults to 8).
//
// If you only have a three-position switch or just want three modes, set your 
// switch to produce 1165, 1425, and 1815 microseconds and configure 
// FLIGHT_MODE 1 & 2, 3 & 4 and 5 & 6 to be the same.  This is the default.
//
// If you have FLIGHT_MODE_CHANNEL set to 8 (the default) and your control
// channel connected to input channel 8, the hardware failsafe mode will
// activate for any control input over 1750ms.
//
// For more modes (up to six), set your switch(es) to produce any of 1165, 
// 1295, 1425, 1555, 1685, and 1815 microseconds. 
//
// Flight mode |  Switch Setting (ms)
// ------------+---------------------
//      1      |     1165
//      2      |     1295
//      3      |     1425
//      4      |     1555
//      5      |     1685
//      6      |     1815	(FAILSAFE if using channel 8)
//
// The following standard flight modes are available:
//
//  Name            | Description
// -----------------+--------------------------------------------
//                  |
//  MANUAL          | Full manual control via the hardware multiplexer.
//                  |
//  STABILIZE       | Tries to maintain level flight, but can be overridden with radio control inputs.
//                  |
//  FLY_BY_WIRE_A   | Autopilot style control via user input, with manual throttle.
//                  |
//  FLY_BY_WIRE_B   | Autopilot style control via user input, aispeed controlled with throttle.
//                  |
//  RTL             | Returns to the Home location and then LOITERs at a safe altitude.
//                  |
//  AUTO            | Autonomous flight based on programmed waypoints.  Use the WaypointWriter
//                  | application or your Ground Control System to edit and upload
//                  | waypoints and other commands.
//                  |
//
//
// The following non-standard modes are EXPERIMENTAL:
//
//  Name            | Description
// -----------------+--------------------------------------------
//                  |
//  LOITER          | Flies in a circle around the current location.
//                  |
//  CIRCLE          | Flies in a stabilized 'dumb' circle.
//                  |
//
//
// If you are using channel 8 for mode switching then FLIGHT_MODE_5 and
// FLIGHT_MODE_6 should be MANUAL.
//
//
#define FLIGHT_MODE_CHANNEL	8
//
//#define FLIGHT_MODE_1         RTL
//#define FLIGHT_MODE_2         RTL
//#define FLIGHT_MODE_3         FLY_BY_WIRE_A
//#define FLIGHT_MODE_4         FLY_BY_WIRE_A
//#define FLIGHT_MODE_5         MANUAL
//#define FLIGHT_MODE_6         MANUAL

// Define the pulse width when to switch to next higher FLIGHT_MODE
#define FLIGHT_MODE_1_BOUNDARY	1125
#define FLIGHT_MODE_2_BOUNDARY	1335
#define FLIGHT_MODE_3_BOUNDARY	1550
#define FLIGHT_MODE_4_BOUNDARY	1690
#define FLIGHT_MODE_5_BOUNDARY	1880




//////////////////////////////////////////////////////////////////////////////
// THROTTLE_FAILSAFE                        OPTIONAL
// THROTTLE_FS_VALUE                        OPTIONAL
// THROTTLE_FAILSAFE_ACTION                 OPTIONAL
//
// The throttle failsafe allows you to configure a software failsafe activated
// by a setting on the throttle input channel (channel 3).
//
// This can be used to achieve a failsafe override on loss of radio control
// without having to sacrifice one of your FLIGHT_MODE settings, as the 
// throttle failsafe overrides the switch-selected mode.
//
// Throttle failsafe is enabled by setting THROTTLE_FAILSAFE to 1.  The default
// is for it to be disabled.
//
// If the throttle failsafe is enabled, THROTTLE_FS_VALUE sets the channel value
// below which the failsafe engages.  The default is 975ms, which is a very low
// throttle setting.  Most transmitters will let you trim the manual throttle
// position up so that you cannot engage the failsafe with a regular stick movement.
//
// Configure your receiver's failsafe setting for the throttle channel to the
// absolute minimum, and use the ArduPilotMega_demo program to check that
// you cannot reach that value with the throttle control.  Leave a margin of
// at least 50 microseconds between the lowest throttle setting and 
// THROTTLE_FS_VALUE.
//
// The FAILSAFE_ACTION setting determines what APM will do when throttle failsafe
// mode is entered while flying in AUTO mode.  This is important in order to avoid
// accidental failsafe behaviour when flying waypoints that take the aircraft 
// temporarily out of radio range.
//
// If FAILSAFE_ACTION is 1, when failsafe is entered in AUTO or LOITER modes, 
// the aircraft will head for home in RTL mode.  If the throttle channel moves
// back up, it will return to AUTO or LOITER mode.
//
// The default behaviour is to ignore throttle failsafe in AUTO and LOITER modes.
//
//#define THROTTLE_FAILSAFE   DISABLED
//#define THROTTLE_FS_VALUE   975
//#define THROTTLE_FAILSAFE_ACTION 2
//

//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM                                OPTIONAL
//
// ArduPilot Mega can update its trim settings by looking at the
// radio inputs when switching out of MANUAL mode.  This allows you to
// manually trim your aircraft before switching to an assisted mode, but it
// also means that you should avoid switching out of MANUAL while you have
// any control stick deflection.
//
// The default is to enable AUTO_TRIM.
//
#define AUTO_TRIM           ENABLED
//

//////////////////////////////////////////////////////////////////////////////
// THROTTLE_REVERSE                         OPTIONAL
//
// A few speed controls require the throttle servo signal be reversed.  Setting
// this to ENABLED will reverse the throttle output signal.  Ensure that your 
// throttle needs to be reversed by using the hardware failsafe and the 
// ArduPilotMega_demo program before setting this option.
//
// The default is to not reverse the signal.
//
//#define THROTTLE_REVERSE    DISABLED
//

//////////////////////////////////////////////////////////////////////////////
// ENABLE_STICK_MIXING                     OPTIONAL
//
// If this option is set to ENABLED, manual control inputs are are respected
// while in the autopilot modes (AUTO, RTL, LOITER, CIRCLE etc.)
//
// The default is to enable stick mixing, allowing the pilot to take
// emergency action without switching modes.
//
#define ENABLE_STICK_MIXING ENABLED
//

//////////////////////////////////////////////////////////////////////////////
// THROTTLE_OUT                             DEBUG
//
// When debugging, it can be useful to disable the throttle output.  Set
// this option to DISABLED to disable throttle output signals.
//
// The default is to not disable throttle output.
//
#define THROTTLE_OUT        DISABLED
//


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY                       OPTIONAL
//
// If configured, inserts a delay between power-up and the beginning of IMU
// calibration during a ground start.
//
// Use this setting to give you time to position the aircraft horizontally
// for the IMU calibration.
//
// The default is to begin IMU calibration immediately at startup.
// 
//#define GROUND_START_DELAY  0
//

//////////////////////////////////////////////////////////////////////////////
// ENABLE_AIR_START                         OPTIONAL
//
// If air start is disabled then you will get a ground start (including IMU 
// calibration) every time the AP is powered up. This means that if you get 
// a power glitch or reboot for some reason in the air, you will probably 
// crash, but it prevents a lot of problems on the ground like unintentional
// motor start-ups, etc.
//
// If air start is enabled then you will get an air start at power up and a
// ground start will be performed if the speed is near zero when we get gps
// lock.
//
// The default is to disable air start.
//
#define ENABLE_AIR_START    0
//


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Altitude measurement and control.
//
// AOA                                      OPTIONAL
//
// The angle in 100ths of a degree that the nose of the aircraft should be
// raised from horizontal in level flight.  The default is 1 degree.
//
//#define AOA                 100 // note, 100ths of a degree
//
// ALT_EST_GAIN                             OPTIONAL
//
// The gain of the altitude estimation function; a lower number results
// in slower error correction and smoother output.  The default is a 
// reasonable starting point.
//
//#define ALT_EST_GAIN        0.01
//
// ALTITUDE_MIX                             OPTIONAL
//
// Configures the blend between GPS and pressure altitude.
// 0 = GPS altitude, 1 = Press alt, 0.5 = half and half, etc.
//
// The default is to use only GPS altitude.
//
//#define ALTITUDE_MIX        0
//


//////////////////////////////////////////////////////////////////////////////
// ENABLE_HIL                      	    OPTIONAL
//
// This will output a binary control string to for use in HIL sims
// such as Xplane 9 or FlightGear.
//
//#define ENABLE_HIL		ENABLED
//


//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE                          OPTIONAL
//
// The speed in metres per second to maintain during cruise.  The default
// is 10m/s, which is a conservative value suitable for relatively small,
// light aircraft.
//
#define AIRSPEED_CRUISE     15
//

//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control
//
// AIRSPEED_FBW_MIN                         OPTIONAL
// AIRSPEED_FBW_MAX                         OPTIONAL
//
// Airspeed corresponding to minimum and maximum throttle in Fly By Wire B mode.
// The defaults are 6 and 30 metres per second.
//
// THROTTLE_ALT_P                           OPTIONAL
// THROTTLE_ALT_I                           OPTIONAL
// THROTTLE_ALT_D                           OPTIONAL
//
// P, I and D terms for the throttle control loop.  Defaults are 0.5, 0, 0.
//
// THROTTLE_ALT_INT_MAX                     OPTIONAL
//
// Maximum throttle input due to the integral.  Limits the throttle input
// due to persistent inability to maintain the commanded speed.  Helps
// prevent the throttle from staying wide open when the control is reduced
// after a period at maxium speed.
// Default is 20 (20%).
//
//#define AIRSPEED_FBW_MIN    6
//#define AIRSPEED_FBW_MAX    30
//#define THROTTLE_ALT_P      0.32
//#define THROTTLE_ALT_I      0.04
//#define THROTTLE_ALT_D      0.0
//#define THROTTLE_ALT_INT_MAX 20 
//

//////////////////////////////////////////////////////////////////////////////
// Throttle control
//
// THROTTLE_MIN                             OPTIONAL
//
// The minimum throttle setting to which the autopilot will reduce the 
// throttle while descending.  The default is zero, which is
// suitable for aircraft with a steady power-off glide.  Increase this
// value if your aircraft needs throttle to maintain a stable descent in
// level flight.
//
// THROTTLE_CRUISE                          OPTIONAL
//
// The approximate throttle setting to achieve AIRSPEED_CRUISE in level flight.
// The default is 45%, which is reasonable for a modestly powered aircraft.
//
// THROTTLE_MAX                             OPTIONAL
//
// The maximum throttle setting the autopilot will apply.  The default is 75%.
// Reduce this value if your aicraft is overpowered, or has complex flight 
// characteristics at high throttle settings.
//
//#define THROTTLE_MIN        0
//#define THROTTLE_CRUISE     45
//#define THROTTLE_MAX        75
//

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
// HEAD_MAX                                 OPTIONAL
//
// The maximum commanded bank angle in either direction.
// The default is 45 degrees.  Decrease this value if your aircraft is not
// stable or has difficulty maintaining altitude in a steep bank.
//
// PITCH_MAX                                OPTIONAL
//
// The maximum commanded pitch up angle.
// The default is 15 degrees.  Care should be taken not to set this value too
// large, as the aircraft may stall.
//
// PITCH_MIN
//
// The maximum commanded pitch down angle.  Note that this value must be 
// negative.  The default is -25 degrees.  Care should be taken not to set 
// this value too large as it may result in overspeeding the aircraft.
//
//#define HEAD_MAX            45
//#define PITCH_MAX           15
//#define PITCH_MIN           -25

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//
// Tuning values for the attitude control PID loops.
//
// The P term is the primary tuning value.  This determines how the control
// deflection varies in proportion to the required correction.
//
// The I term is used to help control surfaces settle.  This value should
// normally be kept low.
//
// The D term is used to control overshoot.  Avoid using or adjusting this
// term if you are not familiar with tuning PID loops.  It should normally
// be zero for most aircraft.
//
// Note: When tuning these values, start with changes of no more than 25% at 
// a time.
//
// SERVO_ROLL_P                             OPTIONAL
// SERVO_ROLL_I                             OPTIONAL
// SERVO_ROLL_D                             OPTIONAL
//
// P, I and D terms for roll control.  Defaults are 0.4, 0, 0.
//
// SERVO_ROLL_INT_MAX                       OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// ROLL_SLEW_LIMIT                          EXPERIMENTAL
//
// Limits the slew rate of the roll control in degrees per second.  If zero,
// slew rate is not limited.  Default is to not limit the roll control slew rate.
// (This feature is currently not implemented.)
//
// SERVO_PITCH_P                            OPTIONAL
// SERVO_PITCH_I                            OPTIONAL
// SERVO_PITCH_D                            OPTIONAL
//
// P, I and D terms for the pitch control.  Defaults are 0.6, 0, 0.
//
// SERVO_PITCH_INT_MAX                      OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. native flight
// AoA).  If you find this value is insufficient, consider adjusting the AOA
// parameter.
// Default is 5 degrees.
//
// PITCH_COMP                               OPTIONAL
//
// Adds pitch input to compensate for the loss of lift due to roll control.
// Default is 0.20 (20% of roll control also applied to pitch control).
//
// SERVO_YAW_P                              OPTIONAL
// SERVO_YAW_I                              OPTIONAL
// SERVO_YAW_D                              OPTIONAL
//
// P, I and D terms for the YAW control.  Defaults are 0.5, 0, 0.
//
// SERVO_YAW_INT_MAX                        OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// RUDDER_MIX                               OPTIONAL
//
// Roll to yaw mixing.  This allows for co-ordinated turns. 
// Default is 0.50 (50% of roll control also applied to yaw control.)
// 
//#define SERVO_ROLL_P        0.4
//#define SERVO_ROLL_I        0.0
//#define SERVO_ROLL_D        0.0
//#define SERVO_ROLL_INT_MAX  5
//#define ROLL_SLEW_LIMIT     0
//#define SERVO_PITCH_P       0.6
//#define SERVO_PITCH_I       0.0
//#define SERVO_PITCH_D       0.0
//#define SERVO_PITCH_INT_MAX 5
//#define PITCH_COMP          0.2
//#define SERVO_YAW_P         0.5
//#define SERVO_YAW_I         0.0
//#define SERVO_YAW_D         0.0
//#define SERVO_YAW_INT_MAX   5
//#define RUDDER_MIX          0.5
//

//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
// 
// Tuning values for the navigation control PID loops.
//
// The P term is the primary tuning value.  This determines how the control
// deflection varies in proportion to the required correction.
//
// The I term is used to control drift.
//
// The D term is used to control overshoot.  Avoid adjusting this term if 
// you are not familiar with tuning PID loops.
//
// Note: When tuning these values, start with changes of no more than 25% at 
// a time.
//
// NAV_ROLL_P                               OPTIONAL
// NAV_ROLL_I                               OPTIONAL
// NAV_ROLL_D                               OPTIONAL
//
// P, I and D terms for navigation control over roll, normally used for 
// controlling the aircraft's course.  The P term controls how aggressively
// the aircraft will bank to change or hold course.
// Defaults are 0.7, 0.01, 0.02.
//
// NAV_ROLL_INT_MAX                         OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// NAV_PITCH_ASP_P                          OPTIONAL
// NAV_PITCH_ASP_I                          OPTIONAL
// NAV_PITCH_ASP_D                          OPTIONAL
//
// P, I and D terms for pitch adjustments made to maintain airspeed.
// Defaults are 0.65, 0, 0.
//
// NAV_PITCH_ASP_INT_MAX                    OPTIONAL
//
// Maximum pitch offset due to the integral.  This limits the control
// output from being overdriven due to a persistent offset (eg. inability
// to maintain the programmed airspeed).
// Default is 5 degrees.
//
// NAV_PITCH_ALT_P                          OPTIONAL
// NAV_PITCH_ALT_I                          OPTIONAL
// NAV_PITCH_ALT_D                          OPTIONAL
//
// P, I and D terms for pitch adjustments made to maintain altitude.
// Defaults are 0.65, 0, 0.
// 
// NAV_PITCH_ALT_INT_MAX                    OPTIONAL
//
// Maximum pitch offset due to the integral.  This limits the control
// output from being overdriven due to a persistent offset (eg. inability
// to maintain the programmed altitude).
// Default is 5 degrees.
//
//#define NAV_ROLL_P          0.7
//#define NAV_ROLL_I          0.01
//#define NAV_ROLL_D          0.02
//#define NAV_ROLL_INT_MAX    5
//#define NAV_PITCH_ASP_P     0.65
//#define NAV_PITCH_ASP_I     0.0
//#define NAV_PITCH_ASP_D     0.0
//#define NAV_PITCH_ASP_INT_MAX 5
//#define NAV_PITCH_ALT_P     0.65
//#define NAV_PITCH_ALT_I     0.0
//#define NAV_PITCH_ALT_D     0.0
//#define NAV_PITCH_ALT_INT_MAX 5
//

//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
// The Energy/altitude control system uses throttle input to control aircraft
// altitude.
//
// The P term is the primary tuning value.  This determines how the throttle
// setting varies in proportion to the required correction.
//
// The I term is used to compensate for small offsets.
//
// The D term is used to control overshoot.  Avoid adjusting this term if 
// you are not familiar with tuning PID loops.
//
// THROTTLE_TE_P                            OPTIONAL
// THROTTLE_TE_I                            OPTIONAL
// THROTTLE_TE_D                            OPTIONAL
//
// P, I and D terms for throttle adjustments made to control altitude.
// Defaults are 0.5, 0, 0.
//
// THROTTLE_TE_INT_MAX                      OPTIONAL
//
// Maximum throttle input due to the integral term.  This limits the
// throttle from being overdriven due to a persistent offset (e.g.
// inability to maintain the programmed altitude).
// Default is 20%.
//
// THROTTLE_SLEW_LIMIT                      OPTIONAL
//
// Limits the slew rate of the throttle, in percent per second.  Helps
// avoid sudden throttle changes, which can destabilise the aircraft.
// A setting of zero disables the feature.
// Default is zero (disabled).
//
// P_TO_T                                   OPTIONAL
//
// Pitch to throttle feed-forward gain.  Used when AIRSPEED_SENSOR
// is DISABLED.  Default is 2.5.
//
//#define THROTTLE_TE_P       0.50
//#define THROTTLE_TE_I       0.0
//#define THROTTLE_TE_D       0.0
//#define THROTTLE_TE_INT_MAX 20 
//#define THROTTLE_SLEW_LIMIT 0
//#define P_TO_T              2.5
//

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
// XTRACK_GAIN                              OPTIONAL
//
// Crosstrack compensation in degrees per metre off track.
// Default value is 0.01 degrees per metre.  Values lower than 0.001 will
// disable crosstrack compensation.
//
// XTRACK_ENTRY_ANGLE                       OPTIONAL
//
// Maximum angle used to correct for track following.
// Default value is 30 degrees.
//
//#define XTRACK_GAIN         0.01
//#define XTRACK_ENTRY_ANGLE  30
//

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Subsystem test and debug.
//
// DEBUG_SUBSYSTEM                          DEBUG
//
// Selects a subsystem debug mode.  Default is 0.
//
// 0 = no debug
// 1 = Debug the Radio input
// 2 = Radio Setup / Servo output
// 4 = Debug the GPS input
// 5 = Debug the GPS input - RAW OUTPUT
// 6 = Debug the IMU
// 7 = Debug the Control Switch
// 8 = Debug the Servo DIP switches
// 9 = Debug the Relay out
// 10 = Debug the Magnetometer
// 11 = Debug the ABS pressure sensor
// 12 = Debug the stored waypoints
// 13 = Debug the Throttle
// 14 = Debug the Radio Min Max
// 15 = Debug the EEPROM - Hex Dump
// 16 = XBee X-CTU Range and RSSI Test
// 17 = Debug IMU - raw gyro and accel outputs
// 20 = Debug Analog Sensors
// 
//
//#define DEBUG_SUBSYSTEM     0
//                                

//////////////////////////////////////////////////////////////////////////////
// DEBUG_LEVEL                              DEBUG
//
// Selects the lowest level of debug messages passed to the telemetry system.
// Default is SEVERITY_LOW.  May be one of:
//
// SEVERITY_LOW
// SEVERITY_MEDIUM
// SEVERITY_HIGH
// SEVERITY_CRITICAL
//
//#define DEBUG_LEVEL         SEVERITY_LOW
//

//////////////////////////////////////////////////////////////////////////////
// Dataflash logging control
//
// Each of these logging options may be set to ENABLED to enable, or DISABLED
// to disable the logging of the respective data.
//
// LOG_ATTITUDE_FAST                        DEBUG
//
// Logs basic attitude info to the dataflash at 50Hz (uses more space). 
// Defaults to DISABLED.
//
// LOG_ATTITUDE_MED                         OPTIONAL
//
// Logs basic attitude info to the dataflash at 10Hz (uses less space than 
// LOG_ATTITUDE_FAST).  Defaults to ENABLED.
//
// LOG_GPS                                  OPTIONAL
//
// Logs GPS info to the dataflash at 10Hz.  Defaults to ENABLED.
//
// LOG_PM                                   OPTIONAL
//
// Logs IMU performance monitoring info every 20 seconds.  
// Defaults to DISABLED.
//
// LOG_CTUN                                 OPTIONAL
//
// Logs control loop tuning info at 10 Hz.  This information is useful for tuning
// servo control loop gain values.  Defaults to DISABLED.
//
// LOG_NTUN                                 OPTIONAL
//
// Logs navigation tuning info at 10 Hz.  This information is useful for tuning
// navigation control loop gain values.  Defaults to DISABLED.
//
// LOG_ MODE                                OPTIONAL
//
// Logs changes to the flight mode upon occurrence.  Defaults to ENABLED.
//
// LOG_RAW                                  DEBUG
//
// Logs raw accelerometer and gyro data at 50 Hz (uses more space).
// Defaults to DISABLED.
//
// LOG_CMD                                  OPTIONAL
//
// Logs new commands when they process. 
// Defaults to ENABLED.
//
//#define LOG_ATTITUDE_FAST   DISABLED
//#define LOG_ATTITUDE_MED    ENABLED
//#define LOG_GPS             ENABLED
//#define LOG_PM              ENABLED
//#define LOG_CTUN            DISABLED
//#define LOG_NTUN            DISABLED
//#define LOG_MODE            ENABLED
//#define LOG_RAW             DISABLED
//#define LOG_CMD             ENABLED
//

//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
// WP_RADIUS_DEFAULT                        OPTIONAL
//
// When the user performs a factory reset on the APM, set the waypoint radius
// (the radius from a target waypoint within which the APM will consider
// itself to have arrived at the waypoint) to this value in meters.  This is
// mainly intended to allow users to start using the APM without running the
// WaypointWriter first.
//
// LOITER_RADIUS_DEFAULT                    OPTIONAL
//
// When the user performs a factory reset on the APM, set the loiter radius
// (the distance the APM will attempt to maintain from a waypoint while
// loitering) to this value in meters.  This is mainly intended to allow
// users to start using the APM without running the WaypointWriter first.
//
//#define WP_RADIUS_DEFAULT		20
//#define LOITER_RADIUS_DEFAULT	30
//

//////////////////////////////////////////////////////////////////////////////
// Debugging interface
//
// DEBUG_PORT                               OPTIONAL
//
// The APM will periodically send messages reporting what it is doing; this
// variable determines to which serial port they will be sent.  Port 0 is the
// USB serial port on the shield, port 3 is the telemetry port.
//
//#define DEBUG_PORT			0
//

//
// Do not remove - this is to discourage users from copying this file 
// and using it as-is rather than editing their own.
//

