/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  All APM Project credits from the original work are kept intact below as a
 *  courtesy.
 */

#define THISFIRMWARE   "AvA 1.1"

/*
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
#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Baro_Glitch.h>     // Baro glitch protection library
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>         // Mission command library
#include <AP_Rally.h>           // Rally point library
#include <AC_PID.h>             // PID library
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
//#include <AP_ServoRelayEvents.h>
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
//#include <AP_RCMapper.h>        // RC input mapping library
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
//#include <AP_Parachute.h>		// Parachute release library
#endif
#include <AP_Terrain.h>

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::VTOL aparm;

// Local modules
#include "Parameters.h"


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

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif defined(HAL_BOARD_LOG_DIRECTORY)
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
static const AP_InertialSensor::Sample_rate ins_sample_rate =
        AP_InertialSensor::RATE_100HZ;
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

static AP_GPS gps;

static GPS_Glitch gps_glitch(gps);

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
#error Unrecognized CONFIG_BARO setting
#endif
static Baro_Glitch baro_glitch(barometer);

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
#error Unrecognized CONFIG_COMPASS setting
#endif

#if CONFIG_INS_TYPE == HAL_INS_OILPAN || CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

#if CONFIG_INS_TYPE == HAL_INS_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_INS_TYPE == HAL_INS_PX4
AP_InertialSensor_PX4 ins;
#elif CONFIG_INS_TYPE == HAL_INS_VRBRAIN
AP_InertialSensor_VRBRAIN ins;
#elif CONFIG_INS_TYPE == HAL_INS_HIL
AP_InertialSensor_HIL ins;
#elif CONFIG_INS_TYPE == HAL_INS_OILPAN
AP_InertialSensor_Oilpan ins( &apm1_adc );
#elif CONFIG_INS_TYPE == HAL_INS_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_INS_TYPE == HAL_INS_L3G4200D
AP_InertialSensor_L3G4200D ins;
#elif CONFIG_INS_TYPE == HAL_INS_MPU9250
AP_InertialSensor_MPU9250 ins;
#else
#error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE
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
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

////////////////////////////////////////////////////////////////////////////////
// Plane Items
////////////////////////////////////////////////////////////////////////////////



//this variable keeps track of the ground station commanded altitude in alt_hold mode
float alt_hold_gs_des_alt;

// primary control channels for plane
// these are pointers to rc_channels in the arduplane code. I've made them
// integers just to keep track of the radio_out value
static int16_t channel_roll_out;
static int16_t channel_pitch_out;
static int16_t channel_throttle_out;
static int16_t channel_rudder_out;

//BEV pulling in plane libraries
//Attitude determination
#include <AP_Navigation.h>
#include <AP_L1_Control.h>
#include <AP_Vehicle.h>
static AP_L1_Control L1_controller(ahrs);
#include <APM_Control.h>
#include <BEV_SpdHgt.h>
#include <BEV_TransitionState.h>
#include <BEV_Effectors.h>
#include <BEV_Key.h>
#include "bev_prop_handlers.h"
#include <BEV_Payloads.h>
#include <BEV_Servos.h>

//BEV uORB communication objects
BEV_Key bev_key(g.key_pid, g.key_value);
BEV_PayloadManager payload_manager;
BEV_Servos servos;

//Attitude to servo control
static AP_RollController rollController(ahrs, aparm, DataFlash);
static AP_PitchController pitchController(ahrs, aparm, DataFlash);
static AP_YawController yawController(ahrs, aparm);

// selected navigation controller
static AP_Navigation *nav_controller = &L1_controller;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_roll_cd;

// The instantaneous desired pitch angle.  Hundredths of a degree
static int32_t nav_pitch_cd;

// true if we are in an auto-throttle mode, which means
// we need to run the speed/height controller
static bool auto_throttle_mode;

////////////////////////////////////////////////////////////////////////////////
// CRUISE controller state
////////////////////////////////////////////////////////////////////////////////
static struct {
    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
} cruise_state;

////////////////////////////////////////////////////////////////////////////////
// flight mode specific
////////////////////////////////////////////////////////////////////////////////
static struct {
    // turn angle for next leg of mission
    float next_turn_angle;
}auto_state = {
    next_turn_angle : 90.0f
};

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
        uint8_t home_is_set :1; // 0
        uint8_t pre_arm_rc_check :1; // 3   // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check :1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed :1; // 5   // stops auto missions from beginning until throttle is raised
        uint8_t logging_started :1; // 6   // true if dataflash logging has started
        uint8_t land_complete :1; // 7   // true if we have detected a landing
        uint8_t new_radio_frame :1; // 8       // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag :2; // 9,10   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag :2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected :1; // 13      // true if APM is powered from USB connection
        uint8_t rc_receiver_present :1; // 14  // true if we have an rc receiver present (i.e. if we've ever received an update
        uint8_t compass_mot :1; // 15  // true if we are currently performing compassmot calibration
        uint8_t motor_test :1; // 16  // true if we are currently performing the motors test
        uint8_t initialised :1; // 17  // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
        uint8_t land_complete_maybe :1; // 18  // true if we may have landed (less strict version of land_complete)
        uint8_t unused : 2; //for future things
    };
    uint32_t value;
} ap;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
static uint8_t oldSwitchPosition = 255;
//static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// receiver RSSI
static uint8_t receiver_rssi;

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t rc_override_active :1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio :1; // 1   // A status flag for the radio failsafe
    uint8_t battery :1; // 2   // A status flag for the battery failsafe
    uint8_t gps :1; // 3   // A status flag for the gps failsafe
    uint8_t gcs :1; // 4   // A status flag for the ground station failsafe
    uint8_t ekf :1; // 5   // true if ekf failsafe has occurred
    //BEV adding rc override
    uint8_t rc_override_fs : 1; // 6   //true if rc override failsafe is active

    int8_t radio_counter; // number of iterations with throttle below throttle_fs_value

    uint32_t last_heartbeat_ms; // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
    //BEV adding rcoveride failsafe
    uint32_t last_rc_override_ms; //time of last rc override command
} failsafe;

//BEV flag to force copter RTL state to wpnav
static bool rtl_force_wpnav = false;
//BEV added to switch to roll2yaw ff once arrived at destination
bool guided_arrived = false;

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
// distance between plane and next waypoint in m.
static uint32_t wp_distance;
static uint8_t land_state; // records state of land (flying to location, descending)
/*
 meta data to support counting the number of circles in a loiter
 */
static struct {
    // previous target bearing, used to update sum_cd
    int32_t old_target_bearing_cd;

    // Total desired rotation in a loiter.  Used for Loiter Turns commands.
    int32_t total_cd;

    // total angle completed in the loiter so far
    int32_t sum_cd;

    // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
    int8_t direction;

    // start time of the loiter.  Milliseconds.
    uint32_t start_time_ms;

    // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
    uint32_t time_max_ms;
} loiter;

// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static Location prev_WP_loc;
// The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
static Location next_WP_loc;

////////////////////////////////////////////////////////////////////////////////
// Auto
////////////////////////////////////////////////////////////////////////////////
static AutoMode auto_mode;   // controls which auto controller is run

////////////////////////////////////////////////////////////////////////////////
// RTL
////////////////////////////////////////////////////////////////////////////////
RTLState rtl_state; // records state of rtl (initial climb, returning home, etc)
bool rtl_state_complete; // set to true if the current state is completed

// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
static int32_t initial_armed_bearing;

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static float throttle_avg;                  // g.throttle_cruise as a float
static int16_t desired_climb_rate; // pilot desired climb rate - for logging purposes only

////////////////////////////////////////////////////////////////////////////////
// Loiter control
////////////////////////////////////////////////////////////////////////////////
static uint16_t loiter_time_max; // How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint32_t loiter_time; // How long have we been loitering - The start time in millis

//BEV plane waypoint control
bool pln_nav_position_arrived; //has plane navigation arrive at the correct lat / lon
bool pln_nav_altitude_arrived; //has plane navigation arrived at the correct altitude

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
static float target_sonar_alt;      // desired altitude in cm above the ground
static int32_t baro_alt;            // barometer altitude in cm above home
static float baro_climbrate;        // barometer climbrate in cm/s

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// Current location of the copter
static struct Location current_loc;

////////////////////////////////////////////////////////////////////////////////
// Throttle integrator
////////////////////////////////////////////////////////////////////////////////
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;

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
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
#if AP_AHRS_NAVEKF_AVAILABLE
static AP_InertialNav_NavEKF inertial_nav(ahrs, barometer, gps_glitch, baro_glitch);
#else
static AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch, baro_glitch);
#endif

//BEV speed height control
static BEV_SpdHgt SpdHgt_Controller(ahrs, inertial_nav, ins);

////////////////////////////////////////////////////////////////////////////////
// Attitude, Position and Waypoint navigation objects
// To-Do: move inertial nav up or other navigation variables down here
////////////////////////////////////////////////////////////////////////////////
AC_AttitudeControl attitude_control(ahrs, aparm, motors, g.p_stabilize_roll,
        g.p_stabilize_pitch, g.p_stabilize_yaw, g.pid_rate_roll,
        g.pid_rate_pitch, g.pid_rate_yaw);

AC_PosControl pos_control(ahrs, inertial_nav, motors, attitude_control,
        g.p_alt_hold, g.p_throttle_rate, g.pid_throttle_accel, g.p_loiter_pos,
        g.pid_loiter_rate_lat, g.pid_loiter_rate_lon);
static AC_WPNav wp_nav(inertial_nav, ahrs, pos_control);
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

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
static AP_Relay relay;

// handle repeated servo and relay events
//static AP_ServoRelayEvents ServoRelayEvents(relay);

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
//BEV this is a bad hack but a customer wants to trigger a servo so I'm
//having it open when they trigger the camera. Sorry for the terrible
//coding practices, but I'm passing the RC_Channel to AP_Camera
//and will have it write high when the relay is opened
static AP_Camera camera(&relay, g.rc_13);
#endif


// a pin for reading the receiver RSSI voltage.
static AP_HAL::AnalogSource* rssi_analog_source;

#if CLI_ENABLED == ENABLED
static int8_t setup_show (uint8_t argc, const Menu::arg *argv);
#endif

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
static AP_Mount camera_mount(&current_loc, ahrs, 0);
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
static void pre_arm_checks(bool display_failure);

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
    {   rc_loop, 4, 10},
    {   throttle_loop, 8, 45},
    {   update_GPS, 8, 90},
    {   update_batt_compass, 40, 72},
    {   arm_motors_check, 40, 1},
    {   update_altitude, 40, 100},
    {   run_nav_updates, 40, 80},
    {   update_thr_cruise, 40, 10},
    {   three_hz_loop, 133, 9},
    {   compass_accumulate, 8, 42},
    {   barometer_accumulate, 8, 25},
    //BEV uorb hooks
    {   bev_uorb_update, 8, 160},
    //BEV begin plane hooks
    {   plane_50hz_tasks, 8, 160},
    {   plane_navigate, 40, 100},
    //bev end plane hooks
    {   update_notify, 8, 10},
    {   one_hz_loop, 400, 42},
    {   ekf_dcm_check, 40, 2},
    {   crash_check, 40, 2},
    {   gcs_check_input, 8, 550},
    {   gcs_send_heartbeat, 400, 150},
    {   gcs_send_deferred, 8, 720},
    {   gcs_data_stream_send, 8, 950},
#if COPTER_LEDS == ENABLED
    {   update_copter_leds, 40, 5},
#endif
    {   update_mount, 8, 45},
    {   ten_hz_logging_loop, 40, 30},
    {   fifty_hz_logging_loop, 8, 22},
    {   perf_update, 4000, 20},
    {   read_receiver_rssi, 40, 5},
#if FRSKY_TELEM_ENABLED == ENABLED
    {   telemetry_send, 80, 10},
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
        { rc_loop, 1, 100 }, { throttle_loop, 2, 450 }, { update_GPS, 2, 900 },
        { update_batt_compass, 10, 720 }, { read_aux_switches, 10, 50 }, {
                arm_motors_check, 10, 10 }, { auto_trim, 10, 140 }, {
                update_altitude, 10, 1000 }, { run_nav_updates, 10, 800 }, {
                update_thr_cruise, 1, 50 }, { three_hz_loop, 33, 90 }, {
                compass_accumulate, 2, 420 }, { barometer_accumulate, 2, 250 },
#if FRAME_CONFIG == HELI_FRAME
        {   check_dynamic_flight, 2, 100},
#endif
        { update_notify, 2, 100 }, { one_hz_loop, 100, 420 }, { ekf_dcm_check,
                10, 20 }, { crash_check, 10, 20 }, { gcs_check_input, 2, 550 },
        { gcs_send_heartbeat, 100, 150 }, { gcs_send_deferred, 2, 720 }, {
                gcs_data_stream_send, 2, 950 }, { update_mount, 2, 450 }, {
                ten_hz_logging_loop, 10, 300 },
        { fifty_hz_logging_loop, 2, 220 }, { perf_update, 1000, 200 }, {
                read_receiver_rssi, 10, 50 },
#if FRSKY_TELEM_ENABLED == ENABLED
        {   telemetry_send, 20, 100},
#endif
#ifdef USERHOOK_FASTLOOP
        {   userhook_FastLoop, 1, 100},
#endif
#ifdef USERHOOK_50HZLOOP
        {   userhook_50Hz, 2, 100},
#endif
#ifdef USERHOOK_MEDIUMLOOP
        {   userhook_MediumLoop, 10, 100},
#endif
#ifdef USERHOOK_SLOWLOOP
        {   userhook_SlowLoop, 30, 100},
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
        {   userhook_SuperSlowLoop,100, 100},
#endif
    };
#endif

void setup() {
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0],
            sizeof(scheduler_tasks) / sizeof(scheduler_tasks[0]));
}

/*
 if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void) {
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
 try to accumulate a baro reading
 */
static void barometer_accumulate(void) {
    barometer.accumulate();
}

static void perf_update(void) {
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"),
                (unsigned)perf_info_get_num_long_running(),
                (unsigned)perf_info_get_num_loops(),
                (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void loop() {
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
        return;
    }
    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt = (float) (timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer = timer;

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
static void fast_loop() {

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    //bev only do this if copter is running
    if (is_copter_nav_active()) {
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // run the attitude controllers
    //BEV only update copter flight modes if not entirely plane
    if (is_copter_nav_active()) {
        update_flight_mode();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
static void rc_loop() {
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
static void throttle_loop() {
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // check if we've landed
    update_land_detector();

    // check auto_armed status
    update_auto_armed();
}

// update_mount - update camera mount position
// should be run at 50hz
static void update_mount() {
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
static void update_batt_compass(void) {
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if (g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle((float) g.rc_3.servo_out / 1000.0f);
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS)) {
            Log_Write_Compass();
        }
    }

    // record throttle output
    throttle_integrator += g.rc_3.servo_out;
}

// ten_hz_logging_loop
// should be run at 10hz
static void ten_hz_logging_loop() {
    if (should_log(MASK_LOG_ATTITUDE_MED)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
        //BEV added
        DataFlash.Log_Write_RCOUT_AUX();
    }
    if (should_log(MASK_LOG_NTUN)
            && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }

    //BEV log transition as necessary
    do_transition_logging();
}

// fifty_hz_logging_loop
// should be run at 50hz
static void fifty_hz_logging_loop() {
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
}

// three_hz_loop - 3.3hz loop
static void three_hz_loop() {
    // check if we've lost contact with the ground station
    failsafe_gcs_check();
    //BEV adding rc_override check
    failsafe_rc_override_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED
#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

}

// one_hz_loop - runs at 1Hz
static void one_hz_loop() {
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }

    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    } else {
        pre_arm_checks(false);
    }

    // auto disarm checks
    auto_disarm_check();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // check the user hasn't updated the frame orientation
        //motors.set_frame_orientation(g.frame_orientation);
    }

    // update assigned functions and enable auxiliar servos
    RC_Channel_aux::enable_aux_servos();

#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif

#if MOUNT2 == ENABLED
    camera_mount2.update_mount_type();
#endif

    check_usb_mux();

#if AP_TERRAIN_AVAILABLE
    //terrain.update();
#endif
}

// called at 50hz
static void update_GPS(void) {
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES]; // time of last gps message
    static uint8_t ground_start_count = 10; // counter used to grab at least 10 reads before commiting the Home location
    bool report_gps_glitch;
    bool gps_updated = false;

    gps.update();

    // logging and glitch protection run after every gps message
    for (uint8_t i = 0; i < gps.num_sensors(); i++) {
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
        // run glitch protection and update AP_Notify if home has been initialised
        if (ap.home_is_set) {
            gps_glitch.check_position();
            report_gps_glitch = (gps_glitch.glitching() && !ap.usb_connected
                    && hal.util->safety_switch_state()
                            != AP_HAL::Util::SAFETY_DISARMED);
            if (AP_Notify::flags.gps_glitching != report_gps_glitch) {
                if (gps_glitch.glitching()) {
                    Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_GPS_GLITCH);
                } else {
                    Log_Write_Error(ERROR_SUBSYSTEM_GPS,
                            ERROR_CODE_ERROR_RESOLVED);
                }
                AP_Notify::flags.gps_glitching = report_gps_glitch;
            }
        }

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

            // check if we can initialise home yet
            if (!ap.home_is_set) {
                // if we have a 3d lock and valid location
                if (gps.status() >= AP_GPS::GPS_OK_FIX_3D
                        && gps.location().lat != 0) {
                    if (ground_start_count > 0) {
                        //BEV don't set home location when armed. Otherwise RTL may take you some place interesting...
                        if(!motors.armed()) {
                            ground_start_count--;
                        }
                    } else {
                        // after 10 successful reads store home location
                        // ap.home_is_set will be true so this will only happen once
                        ground_start_count = 0;
                        init_home();

                        // set system clock for log timestamps
                        hal.util->set_system_clock(gps.time_epoch_usec());

                        if (g.compass_enabled) {
                            // Set compass declination automatically
                            compass.set_initial_location(gps.location().lat,
                                    gps.location().lng);
                        }
                    }
                } else {
                    // start again if we lose 3d lock
                    ground_start_count = 10;
                }
            }

            //If we are not currently armed, and we're ready to 
            //enter RTK mode, then capture current state as home,
            //and enter RTK fixes!
            if (!motors.armed() && gps.can_calculate_base_pos()) {

                gps.calculate_base_pos();

            }

#if CAMERA == ENABLED
            //BEV only do distance based camera triggering in auto mode
            if ((control_mode == AUTO) && (camera.update_location(current_loc) == true)) {
                do_take_picture();
            }
#endif
        }
    }

    // check for loss of gps
    failsafe_gps_check();
}

static void read_AHRS(void) {
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
}

// read baro and sonar altitude at 10hz
static void update_altitude() {
    // read in baro altitude
    read_barometer();

    // read in sonar altitude
    sonar_alt = read_sonar();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN();

