/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V3.3-dev"
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
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.com/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS.h>
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_SerialManager.h>   // Serial manager library
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>         // Mission command library
#include <AP_Rally.h>           // Rally point library
#include <AC_PID.h>             // PID library
#include <AC_PI_2D.h>           // PID library (2-axis)
#include <AC_HELI_PID.h>        // Heli specific Rate PID library
#include <AC_P.h>               // P library
#include <AC_AttitudeControl.h> // Attitude control library
#include <AC_AttitudeControl_Heli.h> // Attitude control library for traditional helicopter
#include <AC_PosControl.h>      // Position control library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AC_Circle.h>          // circle navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#include <AP_Frsky_Telem.h>
#if SPRAYER == ENABLED
#include <AC_Sprayer.h>         // crop sprayer library
#endif
#if EPM_ENABLED == ENABLED
#include <AP_EPM.h>				// EPM cargo gripper stuff
#endif
#if PARACHUTE == ENABLED
#include <AP_Parachute.h>		// Parachute release library
#endif
#include <AP_LandingGear.h>     // Landing Gear library
#include <AP_Terrain.h>

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::MultiCopter aparm;

// Local modules
#include "Parameters.h"

// Heli modules
#include "heli.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

// AP_Notify instance
static AP_Notify notify;

// used to detect MAVLink acks from GCS to stop compassmot
static uint8_t command_ack_counter;

// has a log download started?
static bool in_log_download;

////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);
static void gcs_send_text_fmt(const prog_char_t *fmt, ...);


////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if defined(HAL_BOARD_LOG_DIRECTORY)
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);
#else
static DataFlash_Empty DataFlash;
#endif


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
#if MAIN_LOOP_RATE == 400
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_400HZ;
#else
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;
#endif

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

static AP_GPS  gps;

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

static AP_Baro barometer;

static Compass compass;

AP_InertialSensor ins;

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, barometer, gps);
#else
AP_AHRS_DCM ahrs(ins, barometer, gps);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

// Mission library
// forward declaration to keep compiler happy
static bool start_command(const AP_Mission::Mission_Command& cmd);
static bool verify_command(const AP_Mission::Mission_Command& cmd);
static void exit_mission();
AP_Mission mission(ahrs, &start_command, &verify_command, &exit_mission);

////////////////////////////////////////////////////////////////////////////////
// Optical flow sensor
////////////////////////////////////////////////////////////////////////////////
#if OPTFLOW == ENABLED
static OpticalFlow optflow;
#endif

// gnd speed limit required to observe optical flow sensor limits
static float ekfGndSpdLimit;
// scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
static float ekfNavVelGainScaler;

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static AP_SerialManager serial_manager;
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

////////////////////////////////////////////////////////////////////////////////
// SONAR
#if CONFIG_SONAR == ENABLED
static RangeFinder sonar;
static bool sonar_enabled = true; // enable user switch for sonar
#endif

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
 #include USERHOOK_VARIABLES
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t unused1             : 1; // 0
        uint8_t simple_mode         : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
        uint8_t pre_arm_rc_check    : 1; // 3       // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5       // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 6       // true if dataflash logging has started
        uint8_t land_complete       : 1; // 7       // true if we have detected a landing
        uint8_t new_radio_frame     : 1; // 8       // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag            : 2; // 9,10    // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH9_flag            : 2; // 13,14   // ch9 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH10_flag           : 2; // 15,16   // ch10 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH11_flag           : 2; // 17,18   // ch11 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH12_flag           : 2; // 19,20   // ch12 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected       : 1; // 21      // true if APM is powered from USB connection
        uint8_t rc_receiver_present : 1; // 22      // true if we have an rc receiver present (i.e. if we've ever received an update
        uint8_t compass_mot         : 1; // 23      // true if we are currently performing compassmot calibration
        uint8_t motor_test          : 1; // 24      // true if we are currently performing the motors test
        uint8_t initialised         : 1; // 25      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
        uint8_t land_complete_maybe : 1; // 26      // true if we may have landed (less strict version of land_complete)
        uint8_t throttle_zero       : 1; // 27      // true if the throttle stick is at zero, debounced
        uint8_t system_time_set     : 1; // 28      // true if the system time has been set from the GPS
        uint8_t gps_base_pos_set    : 1; // 29      // true when the gps base position has been set (used for RTK gps only)
        enum HomeState home_state   : 2; // 30,31   // home status (unset, set, locked)
    };
    uint32_t value;
} ap;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Structure used to detect changes in the flight mode control switch
static struct {
    int8_t debounced_switch_position;   // currently used switch position
    int8_t last_switch_position;        // switch position in previous iteration
    uint32_t last_edge_time_ms;         // system time that switch position was last changed
} control_switch_state;

static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// receiver RSSI
static uint8_t receiver_rssi;

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
    uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
    uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe
    uint8_t ekf                 : 1; // 5   // true if ekf failsafe has occurred

    int8_t radio_counter;                  // number of iterations with throttle below throttle_fs_value

    uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} failsafe;

////////////////////////////////////////////////////////////////////////////////
// Motor Output
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsQuad
#elif FRAME_CONFIG == TRI_FRAME
 #define MOTOR_CLASS AP_MotorsTri
#elif FRAME_CONFIG == HEXA_FRAME
 #define MOTOR_CLASS AP_MotorsHexa
#elif FRAME_CONFIG == Y6_FRAME
 #define MOTOR_CLASS AP_MotorsY6
#elif FRAME_CONFIG == OCTA_FRAME
 #define MOTOR_CLASS AP_MotorsOcta
#elif FRAME_CONFIG == OCTA_QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsOctaQuad
#elif FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#elif FRAME_CONFIG == SINGLE_FRAME
 #define MOTOR_CLASS AP_MotorsSingle
#elif FRAME_CONFIG == COAX_FRAME
 #define MOTOR_CLASS AP_MotorsCoax
#else
 #error Unrecognised frame type
#endif

#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.rc_7, g.rc_8, g.heli_servo_1, g.heli_servo_2, g.heli_servo_3, g.heli_servo_4, MAIN_LOOP_RATE);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.rc_7, MAIN_LOOP_RATE);
#elif FRAME_CONFIG == SINGLE_FRAME  // single constructor requires extra servos for flaps
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.single_servo_1, g.single_servo_2, g.single_servo_3, g.single_servo_4, MAIN_LOOP_RATE);
#elif FRAME_CONFIG == COAX_FRAME  // single constructor requires extra servos for flaps
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.single_servo_1, g.single_servo_2, MAIN_LOOP_RATE);
#else
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, MAIN_LOOP_RATE);
#endif

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.
static uint32_t wp_distance;
static uint8_t land_state;              // records state of land (flying to location, descending)

////////////////////////////////////////////////////////////////////////////////
// Auto
////////////////////////////////////////////////////////////////////////////////
static AutoMode auto_mode;   // controls which auto controller is run

////////////////////////////////////////////////////////////////////////////////
// Guided
////////////////////////////////////////////////////////////////////////////////
static GuidedMode guided_mode;  // controls which controller is run (pos or vel)

////////////////////////////////////////////////////////////////////////////////
// RTL
////////////////////////////////////////////////////////////////////////////////
RTLState rtl_state;  // records state of rtl (initial climb, returning home, etc)
bool rtl_state_complete; // set to true if the current state is completed

////////////////////////////////////////////////////////////////////////////////
// Circle
////////////////////////////////////////////////////////////////////////////////
bool circle_pilot_yaw_override; // true if pilot is overriding yaw

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static float simple_cos_yaw = 1.0;
static float simple_sin_yaw;
static int32_t super_simple_last_bearing;
static float super_simple_cos_yaw = 1.0;
static float super_simple_sin_yaw;


// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
static int32_t initial_armed_bearing;

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static float throttle_average;              // estimated throttle required to hover
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only


////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
static float acro_level_mix;                // scales back roll, pitch and yaw inversely proportional to input from pilot

////////////////////////////////////////////////////////////////////////////////
// Loiter control
////////////////////////////////////////////////////////////////////////////////
static uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

////////////////////////////////////////////////////////////////////////////////
// Flip
////////////////////////////////////////////////////////////////////////////////
static Vector3f flip_orig_attitude;         // original copter attitude before flip

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;

////////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
#if FRSKY_TELEM_ENABLED == ENABLED
static AP_Frsky_Telem frsky_telemetry(ahrs, battery);
#endif

////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm - Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
static float target_sonar_alt;      // desired altitude in cm above the ground
static int32_t baro_alt;            // barometer altitude in cm above home
static float baro_climbrate;        // barometer climbrate in cm/s


////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// Current location of the copter (altitude is relative to home)
static struct   Location current_loc;


////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// auto flight mode's yaw mode
static uint8_t auto_yaw_mode = AUTO_YAW_LOOK_AT_NEXT_WP;
// Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
static Vector3f roi_WP;
// bearing from current location to the yaw_look_at_WP
static float yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;
// heading when in yaw_look_ahead_bearing
static float yaw_look_ahead_bearing;



////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
static AP_InertialNav_NavEKF inertial_nav(ahrs);

////////////////////////////////////////////////////////////////////////////////
// Attitude, Position and Waypoint navigation objects
// To-Do: move inertial nav up or other navigation variables down here
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == HELI_FRAME
AC_AttitudeControl_Heli attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                        g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw);
#else
AC_AttitudeControl attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                        g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw);
#endif
AC_PosControl pos_control(ahrs, inertial_nav, motors, attitude_control,
                        g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                        g.p_pos_xy, g.pi_vel_xy);
static AC_WPNav wp_nav(inertial_nav, ahrs, pos_control, attitude_control);
static AC_Circle circle_nav(inertial_nav, ahrs, pos_control);

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
static int16_t pmTest1;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Loiter timer - Records how long we have been in loiter
static uint32_t rtl_loiter_start_time;

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object
static AP_Relay relay;

// handle repeated servo and relay events
static AP_ServoRelayEvents ServoRelayEvents(relay);

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  static AP_Camera camera(&relay);
#endif

// a pin for reading the receiver RSSI voltage.
static AP_HAL::AnalogSource* rssi_analog_source;

#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
static AP_Mount camera_mount(ahrs, current_loc);
#endif

////////////////////////////////////////////////////////////////////////////////
// AC_Fence library to reduce fly-aways
////////////////////////////////////////////////////////////////////////////////
#if AC_FENCE == ENABLED
AC_Fence    fence(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// Rally library
////////////////////////////////////////////////////////////////////////////////
#if AC_RALLY == ENABLED
AP_Rally rally(ahrs);
#endif

////////////////////////////////////////////////////////////////////////////////
// Crop Sprayer
////////////////////////////////////////////////////////////////////////////////
#if SPRAYER == ENABLED
static AC_Sprayer sprayer(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// EPM Cargo Griper
////////////////////////////////////////////////////////////////////////////////
#if EPM_ENABLED == ENABLED
static AP_EPM epm;
#endif

////////////////////////////////////////////////////////////////////////////////
// Parachute release
////////////////////////////////////////////////////////////////////////////////
#if PARACHUTE == ENABLED
static AP_Parachute parachute(relay);
#endif

////////////////////////////////////////////////////////////////////////////////
// Landing Gear Controller
////////////////////////////////////////////////////////////////////////////////
static AP_LandingGear landinggear;

////////////////////////////////////////////////////////////////////////////////
// terrain handling
#if AP_TERRAIN_AVAILABLE
AP_Terrain terrain(ahrs, mission, rally);
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
static bool pre_arm_checks(bool display_failure);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info);

#if MAIN_LOOP_RATE == 400
/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 2.5ms units) and the maximum time they are expected to take (in
  microseconds)
  1    = 400hz
  2    = 200hz
  4    = 100hz
  8    = 50hz
  20   = 20hz
  40   = 10hz
  133  = 3hz
  400  = 1hz
  4000 = 0.1hz
  
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { rc_loop,               4,     10 },
    { throttle_loop,         8,     45 },
    { update_GPS,            8,     90 },
#if OPTFLOW == ENABLED
    { update_optical_flow,   2,     20 },
#endif
    { update_batt_compass,  40,     72 },
    { read_aux_switches,    40,      5 },
    { arm_motors_check,     40,      1 },
    { auto_trim,            40,     14 },
    { update_altitude,      40,    100 },
    { run_nav_updates,       8,     80 },
    { update_thr_average,   40,     10 },
    { three_hz_loop,       133,      9 },
    { compass_accumulate,    8,     42 },
    { barometer_accumulate,  8,     25 },
#if FRAME_CONFIG == HELI_FRAME
    { check_dynamic_flight,  8,     10 },
#endif
    { update_notify,         8,     10 },
    { one_hz_loop,         400,     42 },
    { ekf_dcm_check,        40,      2 },
    { crash_check,          40,      2 },
    { landinggear_update,   40,      1 },
    { gcs_check_input,	     8,    550 },
    { gcs_send_heartbeat,  400,    150 },
    { gcs_send_deferred,     8,    720 },
    { gcs_data_stream_send,  8,    950 },
#if COPTER_LEDS == ENABLED
    { update_copter_leds,   40,      5 },
#endif
    { update_mount,          8,     45 },
    { ten_hz_logging_loop,  40,     30 },
    { fifty_hz_logging_loop, 8,     22 },
    { perf_update,        4000,     20 },
    { read_receiver_rssi,   40,      5 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { frsky_telemetry_send, 80,     10 },
#endif
#if EPM_ENABLED == ENABLED
    { epm_update,           40,     10 },
#endif
#ifdef USERHOOK_FASTLOOP
    { userhook_FastLoop,     4,     10 },
#endif
#ifdef USERHOOK_50HZLOOP
    { userhook_50Hz,         8,     10 },
#endif
#ifdef USERHOOK_MEDIUMLOOP
    { userhook_MediumLoop,  40,     10 },
#endif
#ifdef USERHOOK_SLOWLOOP
    { userhook_SlowLoop,    120,    10 },
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    { userhook_SuperSlowLoop,400,   10 },
#endif
};
#else
/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
  1    = 100hz
  2    = 50hz
  4    = 25hz
  10   = 10hz
  20   = 5hz
  33   = 3hz
  50   = 2hz
  100  = 1hz
  1000 = 0.1hz
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { rc_loop,               1,     100 },
    { throttle_loop,         2,     450 },
    { update_GPS,            2,     900 },
#if OPTFLOW == ENABLED
    { update_optical_flow,   1,     100 },
#endif
    { update_batt_compass,  10,     720 },
    { read_aux_switches,    10,      50 },
    { arm_motors_check,     10,      10 },
    { auto_trim,            10,     140 },
    { update_altitude,      10,    1000 },
    { run_nav_updates,       4,     800 },
    { update_thr_average,    1,      50 },
    { three_hz_loop,        33,      90 },
    { compass_accumulate,    2,     420 },
    { barometer_accumulate,  2,     250 },
#if FRAME_CONFIG == HELI_FRAME
    { check_dynamic_flight,  2,     100 },
#endif
    { update_notify,         2,     100 },
    { one_hz_loop,         100,     420 },
    { ekf_dcm_check,        10,      20 },
    { crash_check,          10,      20 },
    { landinggear_update,   10,      10 },
    { gcs_check_input,	     2,     550 },
    { gcs_send_heartbeat,  100,     150 },
    { gcs_send_deferred,     2,     720 },
    { gcs_data_stream_send,  2,     950 },
    { update_mount,          2,     450 },
    { ten_hz_logging_loop,  10,     300 },
    { fifty_hz_logging_loop, 2,     220 },
    { perf_update,        1000,     200 },
    { read_receiver_rssi,   10,      50 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { frsky_telemetry_send, 20,     100 },
#endif
#if EPM_ENABLED == ENABLED
    { epm_update,           10,      20 },
#endif
#ifdef USERHOOK_FASTLOOP
    { userhook_FastLoop,     1,    100  },
#endif
#ifdef USERHOOK_50HZLOOP
    { userhook_50Hz,         2,    100  },
#endif
#ifdef USERHOOK_MEDIUMLOOP
    { userhook_MediumLoop,   10,    100 },
#endif
#ifdef USERHOOK_SLOWLOOP
    { userhook_SlowLoop,     30,    100 },
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    { userhook_SuperSlowLoop,100,   100 },
#endif
};
#endif


void setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = hal.scheduler->micros();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

static void perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs_send_text_fmt(PSTR("PERF: %u/%u %lu %lu\n"),
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available);
}


// Main loop - 100hz
static void fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // run low level rate controllers that only require IMU data
    attitude_control.rate_controller_run();
    
#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // run the attitude controllers
    update_flight_mode();
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
static void rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
static void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // check if we've landed
    update_land_detector();

    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_low_comp();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif
}

// update_mount - update camera mount position
// should be run at 50hz
static void update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
static void update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS)) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
static void ten_hz_logging_loop()
{
    if (should_log(MASK_LOG_ATTITUDE_MED)) {
        Log_Write_Attitude();
        Log_Write_Rate();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
}

// fifty_hz_logging_loop
// should be run at 50hz
static void fifty_hz_logging_loop()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rate();
    }

    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
}

// three_hz_loop - 3.3hz loop
static void three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
static void one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }

    // auto disarm checks
    auto_disarm_check();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);
    }

    // update assigned functions and enable auxiliar servos
    RC_Channel_aux::enable_aux_servos();

    check_usb_mux();

#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if CONFIG_SONAR == ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        sonar.set_estimated_terrain_height(height);
    }
#endif
#endif

#if AC_FENCE == ENABLED
    // set fence altitude limit in position controller
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        pos_control.set_alt_max(pv_alt_above_origin(fence.get_safe_alt()*100.0f));
    }
#endif
}

// called at 50hz
static void update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

        // update home from GPS location if necessary
        update_home_from_GPS();

        // check gps base position (used for RTK only)
        check_gps_base_pos();

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

static void
init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;
        pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;
        pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    g.rc_1.control_in = rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw();
    g.rc_2.control_in = -rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw();
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
}

// read baro and sonar altitude at 10hz
static void update_altitude()
{
    // read in baro altitude
    read_barometer();

    // read in sonar altitude
    sonar_alt           = read_sonar();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN();

