/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduPlane V3.2.1alpha2"
/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.com for details

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

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>     // Range finder library
#include <Filter.h>                     // Filter library
#include <AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay.h>       // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed.h>
#include <AP_Terrain.h>

#include <APM_OBC.h>
#include <APM_Control.h>
#include <AP_AutoTune.h>
#include <GCS.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash.h>
#include <SITL.h>
#include <AP_Scheduler.h>       // main loop scheduler

#include <AP_Navigation.h>
#include <AP_L1_Control.h>
#include <AP_RCMapper.h>        // RC input mapping library

#include <AP_Vehicle.h>
#include <AP_SpdHgtControl.h>
#include <AP_TECS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>     // Mission command library

#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library

#include <AP_Arming.h>
#include <AP_BoardConfig.h>
#include <AP_Frsky_Telem.h>
#include <AP_ServoRelayEvents.h>

#include <AP_Rally.h>

#include <AP_OpticalFlow.h>     // Optical Flow library

// Pre-AP_HAL compatibility
#include "compat.h"

// Configuration
#include "config.h"

// Local modules
#include "defines.h"

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::FixedWing aparm;

#include "Parameters.h"

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN.h>

AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_50HZ;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;
 
// mapping between input channels
static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// primary control channels
static RC_Channel *channel_roll;
static RC_Channel *channel_pitch;
static RC_Channel *channel_throttle;
static RC_Channel *channel_rudder;

// notification object for LEDs, buzzers etc (parameter set to false disables external leds)
static AP_Notify notify;

////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);
void gcs_send_text_fmt(const prog_char_t *fmt, ...);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if LOGGING_ENABLED == ENABLED
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif defined(HAL_BOARD_LOG_DIRECTORY)
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);
#else
// no dataflash driver
DataFlash_Empty DataFlash;
#endif
#endif

// has a log download started?
static bool in_log_download;

// scaled roll limit based on pitch
static int32_t roll_limit_cd;
static int32_t pitch_limit_min_cd;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode.  Real sensors are used.
// - HIL Attitude mode.  Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode.  Synthetic sensors are configured that
//   supply data from the simulation.
//

// GPS driver
static AP_GPS gps;

// flight modes convenience array
static AP_Int8          *flight_modes = &g.flight_mode1;

static AP_Baro barometer;

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#elif CONFIG_COMPASS == HAL_COMPASS_AK8963
static AP_Compass_AK8963_MPU9250 compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

AP_InertialSensor ins;

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, barometer, gps);
#else
AP_AHRS_DCM ahrs(ins, barometer, gps);
#endif

static AP_L1_Control L1_controller(ahrs);
static AP_TECS TECS_controller(ahrs, aparm);

// Attitude to servo controllers
static AP_RollController  rollController(ahrs, aparm, DataFlash);
static AP_PitchController pitchController(ahrs, aparm, DataFlash);
static AP_YawController   yawController(ahrs, aparm);
static AP_SteerController steerController(ahrs);

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

// Training mode
static bool training_manual_roll;  // user has manual roll control
static bool training_manual_pitch; // user has manual pitch control

/*
  keep steering and rudder control separated until we update servos,
  to allow for a separate wheel servo from rudder servo
 */
static struct {
    bool ground_steering; // are we doing ground steering?
    int16_t steering; // value for nose/tail wheel
    int16_t rudder;   // value for rudder
} steering_control;

// should throttle be pass-thru in guided?
static bool guided_throttle_passthru;

// are we doing calibration? This is used to allow heartbeat to
// external failsafe boards during baro and airspeed calibration
static bool in_calibration;


////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

// selected navigation controller
static AP_Navigation *nav_controller = &L1_controller;

// selected navigation controller
static AP_SpdHgtControl *SpdHgt_Controller = &TECS_controller;

////////////////////////////////////////////////////////////////////////////////
// Analog Inputs
////////////////////////////////////////////////////////////////////////////////

// a pin for reading the receiver RSSI voltage. 
static AP_HAL::AnalogSource *rssi_analog_source;

////////////////////////////////////////////////////////////////////////////////
// rangefinder
////////////////////////////////////////////////////////////////////////////////
static RangeFinder rangefinder;

static struct {
    bool in_range;
    float correction;
    uint32_t last_correction_time_ms;
    uint8_t in_range_count;
} rangefinder_state;

////////////////////////////////////////////////////////////////////////////////
// Relay
////////////////////////////////////////////////////////////////////////////////
static AP_Relay relay;

// handle servo and relay events
static AP_ServoRelayEvents ServoRelayEvents(relay);

// Camera
#if CAMERA == ENABLED
static AP_Camera camera(&relay);
#endif

////////////////////////////////////////////////////////////////////////////////
// Optical flow sensor
////////////////////////////////////////////////////////////////////////////////
static OpticalFlow optflow;

//Rally Ponints
static AP_Rally rally(ahrs);

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// remember if USB is connected, so we can adjust baud rate
static bool usb_connected;

/* Radio values
 *               Channel assignments
 *                       1   Ailerons
 *                       2   Elevator
 *                       3   Throttle
 *                       4   Rudder
 *                       5   Aux5
 *                       6   Aux6
 *                       7   Aux7
 *                       8   Aux8/Mode
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
static enum FlightMode control_mode  = INITIALISING;
static enum FlightMode previous_mode = INITIALISING;

// Used to maintain the state of the previous control switch position
// This is set to 254 when we need to re-read the switch
static uint8_t oldSwitchPosition = 254;

// This is used to enable the inverted flight feature
static bool inverted_flight     = false;

// This is used to enable the PX4IO override for testing
static bool px4io_override_enabled = false;

static struct {
    // These are trim values used for elevon control
    // For elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are
    // equivalent aileron and elevator, not left and right elevon
    uint16_t trim1;
    uint16_t trim2;
    // These are used in the calculation of elevon1_trim and elevon2_trim
    uint16_t ch1_temp;
    uint16_t ch2_temp;
} elevon = {
	trim1 : 1500,
    trim2 : 1500,
    ch1_temp : 1500,
    ch2_temp : 1500
};


////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
static struct {

    // Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
    // RC receiver should be set up to output a low throttle value when signal is lost
    uint8_t ch3_failsafe:1;

    // has the saved mode for failsafe been set?
    uint8_t saved_mode_set:1;

    // flag to hold whether battery low voltage threshold has been breached
    uint8_t low_battery:1;

    // saved flight mode
    enum FlightMode saved_mode;

    // A tracking variable for type of failsafe active
    // Used for failsafe based on loss of RC signal or GCS signal
    int16_t state;

    // number of low ch3 values
    uint8_t ch3_counter;

    // the time when the last HEARTBEAT message arrived from a GCS
    uint32_t last_heartbeat_ms;

    // A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
    uint32_t ch3_timer_ms;

    uint32_t last_valid_rc_ms;
} failsafe;


////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7                        = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t ground_start_count      = 5;

// true if we have a position estimate from AHRS
static bool have_position;


////////////////////////////////////////////////////////////////////////////////
// Airspeed
////////////////////////////////////////////////////////////////////////////////
// The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
// Also used for flap deployment criteria.  Centimeters per second.
static int32_t target_airspeed_cm;

// The difference between current and desired airspeed.  Used in the pitch controller.  Centimeters per second.
static float airspeed_error_cm;

// An amount that the airspeed should be increased in auto modes based on the user positioning the
// throttle stick in the top half of the range.  Centimeters per second.
static int16_t airspeed_nudge_cm;

// Similar to airspeed_nudge, but used when no airspeed sensor.
// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t throttle_nudge = 0;

// receiver RSSI
static uint8_t receiver_rssi;


////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  Centimeters per second
static int32_t groundspeed_undershoot = 0;

// Difference between current altitude and desired altitude.  Centimeters
static int32_t altitude_error_cm;

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
// Airspeed Sensors
////////////////////////////////////////////////////////////////////////////////
AP_Airspeed airspeed(aparm);

////////////////////////////////////////////////////////////////////////////////
// ACRO controller state
////////////////////////////////////////////////////////////////////////////////
static struct {
    bool locked_roll;
    bool locked_pitch;
    float locked_roll_err;
    int32_t locked_pitch_cd;
} acro_state;

////////////////////////////////////////////////////////////////////////////////
// CRUISE controller state
////////////////////////////////////////////////////////////////////////////////
static struct {
    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
} cruise_state;

////////////////////////////////////////////////////////////////////////////////
// ground steering controller state
////////////////////////////////////////////////////////////////////////////////
static struct {
	// Direction held during phases of takeoff and landing centidegrees
	// A value of -1 indicates the course has not been set/is not in use
	// this is a 0..36000 value, or -1 for disabled
    int32_t hold_course_cd;

    // locked_course and locked_course_cd are used in stabilize mode 
    // when ground steering is active, and for steering in auto-takeoff
    bool locked_course;
    float locked_course_err;
} steer_state = {
	hold_course_cd    : -1,
    locked_course     : false,
    locked_course_err : 0
};

////////////////////////////////////////////////////////////////////////////////
// flight mode specific
////////////////////////////////////////////////////////////////////////////////
static struct {
    // Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
    bool takeoff_complete:1;

    // Flag to indicate if we have landed.
    // Set land_complete if we are within 2 seconds distance or within 3 meters altitude of touchdown
    bool land_complete:1;

    // should we fly inverted?
    bool inverted_flight:1;

    // should we disable cross-tracking for the next waypoint?
    bool next_wp_no_crosstrack:1;

    // should we use cross-tracking for this waypoint?
    bool no_crosstrack:1;

    // in FBWA taildragger takeoff mode
    bool fbwa_tdrag_takeoff_mode:1;

    // have we checked for an auto-land?
    bool checked_for_autoland:1;

    // denotes if a go-around has been commanded for landing
    bool commanded_go_around:1;

    // Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
    int32_t takeoff_altitude_cm;

    // Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
    int16_t takeoff_pitch_cd;

    // the highest airspeed we have reached since entering AUTO. Used
    // to control ground takeoff
    float highest_airspeed;

    // initial pitch. Used to detect if nose is rising in a tail dragger
    int16_t initial_pitch_cd;

    // turn angle for next leg of mission
    float next_turn_angle;

    // filtered sink rate for landing
    float land_sink_rate;

    // time when we first pass min GPS speed on takeoff
    uint32_t takeoff_speed_time_ms;

    // distance to next waypoint
    float wp_distance;

    // proportion to next waypoint
    float wp_proportion;
} auto_state = {
    takeoff_complete : true,
    land_complete : false,
    inverted_flight  : false,
    next_wp_no_crosstrack : true,
    no_crosstrack : true,
    fbwa_tdrag_takeoff_mode : false,
    checked_for_autoland : false,
    commanded_go_around : false,
    takeoff_altitude_cm : 0,
    takeoff_pitch_cd : 0,
    highest_airspeed : 0,
    initial_pitch_cd : 0,
    next_turn_angle  : 90.0f,
    land_sink_rate   : 0,
    takeoff_speed_time_ms : 0
};

// true if we are in an auto-throttle mode, which means
// we need to run the speed/height controller
static bool auto_throttle_mode;

// this controls throttle suppression in auto modes
static bool throttle_suppressed;

AP_SpdHgtControl::FlightStage flight_stage = AP_SpdHgtControl::FLIGHT_NORMAL;

////////////////////////////////////////////////////////////////////////////////
// Loiter management
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_roll_cd;

// The instantaneous desired pitch angle.  Hundredths of a degree
static int32_t nav_pitch_cd;

// the aerodymamic load factor. This is calculated from the demanded
// roll before the roll is clipped, using 1/sqrt(cos(nav_roll))
static float aerodynamic_load_factor = 1.0f;

// a smoothed airspeed estimate, used for limiting roll angle
static float smoothed_airspeed;

////////////////////////////////////////////////////////////////////////////////
// Mission library
////////////////////////////////////////////////////////////////////////////////
// forward declations needed for functions with : in arguments
static bool verify_command_callback(const AP_Mission::Mission_Command &cmd);
static bool start_command_callback(const AP_Mission::Mission_Command &cmd);
AP_Mission mission(ahrs, 
                   &start_command_callback, 
                   &verify_command_callback, 
                   &exit_mission_callback);

////////////////////////////////////////////////////////////////////////////////
// terrain handling
#if AP_TERRAIN_AVAILABLE
AP_Terrain terrain(ahrs, mission, rally);
#endif

////////////////////////////////////////////////////////////////////////////////
// Outback Challenge Failsafe Support
////////////////////////////////////////////////////////////////////////////////
#if OBC_FAILSAFE == ENABLED
APM_OBC obc(mission, barometer, gps, rcmap);
#endif

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


////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t condition_value;
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static uint32_t condition_start;
// A value used in condition commands.  For example the rate at which to change altitude.
static int16_t condition_rate;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// reference to AHRS home
static const struct Location &home = ahrs.get_home();

// Flag for if we have g_gps lock and have set the home location in AHRS
static bool home_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static Location prev_WP_loc;
// The plane's current location
static struct   Location current_loc;
// The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
static Location next_WP_loc;
// The location of the active waypoint in Guided mode.
static struct Location guided_WP_loc;
// special purpose command used only after mission completed to return vehicle to home or rally point
static struct AP_Mission::Mission_Command auto_rtl_command;

////////////////////////////////////////////////////////////////////////////////
// Altitude control
static struct {
    // target altitude above sea level in cm. Used for barometric
    // altitude navigation
    int32_t amsl_cm;

    // Altitude difference between previous and current waypoint in
    // centimeters. Used for glide slope handling
    int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
    // are we trying to follow terrain?
    bool terrain_following;

    // target altitude above terrain in cm, valid if terrain_following
    // is set
    int32_t terrain_alt_cm;

    // lookahead value for height error reporting
    float lookahead;
#endif
} target_altitude;

////////////////////////////////////////////////////////////////////////////////
// INS variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt                                               = 0.02f;

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static uint32_t perf_mon_timer;
// The maximum and minimum main loop execution time recorded in the current performance monitoring interval
static uint32_t G_Dt_max = 0;
static uint32_t G_Dt_min = 0;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of start of main control loop
static uint32_t fast_loopTimer_us;

// Number of milliseconds used in last main loop cycle
static uint32_t delta_us_fast_loop;

// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount(&current_loc, ahrs, 0);
#endif

#if MOUNT2 == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount2(&current_loc, ahrs, 1);
#endif

////////////////////////////////////////////////////////////////////////////////
// Arming/Disarming mangement class
////////////////////////////////////////////////////////////////////////////////
static AP_Arming arming(ahrs, barometer, compass, home_is_set, gcs_send_text_P);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { read_radio,             1,    700 }, // 0
    { check_short_failsafe,   1,   1000 },
    { ahrs_update,            1,   6400 },
    { update_speed_height,    1,   1600 },
    { update_flight_mode,     1,   1400 },
    { stabilize,              1,   3500 },
    { set_servos,             1,   1600 },
    { read_control_switch,    7,   1000 },
    { gcs_retry_deferred,     1,   1000 },
    { update_GPS_50Hz,        1,   2500 },
    { update_GPS_10Hz,        5,   2500 }, // 10
    { navigate,               5,   3000 },
    { update_compass,         5,   1200 },
    { read_airspeed,          5,   1200 },
    { update_alt,             5,   3400 },
    { adjust_altitude_target, 5,   1000 },
    { obc_fs_check,           5,   1000 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { update_events,		  1,   1500 }, // 20
    { check_usb_mux,          5,    300 },
    { read_battery,           5,   1000 },
    { compass_accumulate,     1,   1500 },
    { barometer_accumulate,   1,    900 },
    { update_notify,          1,    300 },
    { read_rangefinder,       1,    500 },
#if OPTFLOW == ENABLED
    { update_optical_flow,    1,    500 },
#endif
    { one_second_loop,       50,   1000 },
    { check_long_failsafe,   15,   1000 },
    { read_receiver_rssi,     5,   1000 },
    { airspeed_ratio_update, 50,   1000 }, // 30
    { update_mount,           1,   1500 },
    { log_perf_info,        500,   1000 },
    { compass_save,        3000,   2500 },
    { update_logging1,        5,   1700 },
    { update_logging2,        5,   1700 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { telemetry_send,        10,    100 },	
#endif
    { terrain_update,         5,    500 },
};

// setup the var_info table
AP_Param param_loader(var_info);

void setup() {
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

void loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = hal.scheduler->micros();

    delta_us_fast_loop  = timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;

    if (delta_us_fast_loop > G_Dt_max && fast_loopTimer_us != 0) {
        G_Dt_max = delta_us_fast_loop;
    }

    if (delta_us_fast_loop < G_Dt_min || G_Dt_min == 0) {
        G_Dt_min = delta_us_fast_loop;
    }
    fast_loopTimer_us   = timer;

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t remaining = (timer + 20000) - hal.scheduler->micros();
    if (remaining > 19500) {
        remaining = 19500;
    }
    scheduler.run(remaining);
}

// update AHRS system
static void ahrs_update()
{
    ahrs.set_armed(arming.is_armed() && 
                   hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    ahrs.update();

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_IMU))
        Log_Write_IMU();

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = g.roll_limit_cd * cosf(ahrs.pitch);
    pitch_limit_min_cd = aparm.pitch_limit_min_cd * fabsf(cosf(ahrs.roll));

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);
}

/*
  update 50Hz speed/height controller
 */
static void update_speed_height(void)
{
    if (auto_throttle_mode) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        SpdHgt_Controller->update_50hz(relative_altitude());
    }
}


/*
  update camera mount
 */
static void update_mount(void)
{
#if MOUNT == ENABLED
    camera_mount.update_mount_position();
#endif

#if MOUNT2 == ENABLED
    camera_mount2.update_mount_position();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

/*
  read and update compass
 */
static void update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            Log_Write_Compass();
        }
    } else {
        ahrs.set_compass(NULL);
    }
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

/*
  do 10Hz logging
 */
static void update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_IMU))
        Log_Write_IMU();
}

/*
  do 10Hz logging - part2
 */
static void update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();
    
    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();
}


/*
  check for OBC failsafe check
 */
static void obc_fs_check(void)
{
#if OBC_FAILSAFE == ENABLED
    // perform OBC failsafe checks
    obc.check(OBC_MODE(control_mode), failsafe.last_heartbeat_ms, geofence_breached(), failsafe.last_valid_rc_ms);
#endif
}


/*
  update aux servo mappings
 */
static void update_aux(void)
{
    if (!px4io_override_enabled) {
        RC_Channel_aux::enable_aux_servos();
    }

#if MOUNT == ENABLED
        camera_mount.update_mount_type();
#endif
#if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
#endif
}

static void one_second_loop()
{
    if (should_log(MASK_LOG_CURRENT))
        Log_Write_Current();

    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // make it possible to change control channel ordering at runtime
    set_control_channels();

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    update_aux();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data(DataFlash);
    }
#endif
}

static void log_perf_info()
{
    if (scheduler.debug() != 0) {
        gcs_send_text_fmt(PSTR("G_Dt_max=%lu G_Dt_min=%lu\n"), 
                          (unsigned long)G_Dt_max, 
                          (unsigned long)G_Dt_min);
    }
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    G_Dt_max = 0;
    G_Dt_min = 0;
    resetPerfData();
}

static void compass_save()
{
    if (g.compass_enabled) {
        compass.save_offsets();
    }
}

static void terrain_update(void)
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();
#endif
}

/*
  once a second update the airspeed calibration ratio
 */
static void airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (abs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg);
    gcs_send_airspeed_calibration(vg);
}


/*
  read the GPS and update position
 */
static void update_GPS_50Hz(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
    gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                Log_Write_GPS(i);
            }
        }
    }
}

/*
  read update GPS position - 10Hz update
 */
static void update_GPS_10Hz(void)
{
    // get position from AHRS
    have_position = ahrs.get_position(current_loc);

    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 5;

            } else {
                init_home();

                // set system clock for log timestamps
                hal.util->set_system_clock(gps.time_epoch_usec());

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    const Location &loc = gps.location();
                    compass.set_initial_location(loc.lat, loc.lng);
                }
                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        geofence_check(false);

#if CAMERA == ENABLED
        if (camera.update_location(current_loc) == true) {
            do_take_picture();
        }
#endif        

        if (!ahrs.get_armed()) {
            update_home();
        }

        // update wind estimate
        ahrs.estimate_wind();
    }

    calc_gndspeed_undershoot();
}

/*
  main handling for AUTO mode
 */
static void handle_auto_mode(void)
{
    uint8_t nav_cmd_id;

    // we should be either running a mission or RTLing home
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        nav_cmd_id = mission.get_current_nav_cmd().id;
    }else{
        nav_cmd_id = auto_rtl_command.id;
    }

    switch(nav_cmd_id) {
    case MAV_CMD_NAV_TAKEOFF:
        takeoff_calc_roll();
        takeoff_calc_pitch();
        
        // max throttle for takeoff
        channel_throttle->servo_out = takeoff_throttle();
        break;

    case MAV_CMD_NAV_LAND:
        calc_nav_roll();
        calc_nav_pitch();
        
        if (auto_state.land_complete) {
            // during final approach constrain roll to the range
            // allowed for level flight
            nav_roll_cd = constrain_int32(nav_roll_cd, -g.level_roll_limit*100UL, g.level_roll_limit*100UL);
        } else {
            if (!airspeed.use()) {
                // when not under airspeed control, don't allow
                // down pitch in landing
                nav_pitch_cd = constrain_int32(nav_pitch_cd, 0, nav_pitch_cd);
            }
        }
        calc_throttle();
        
        if (auto_state.land_complete) {
            // we are in the final stage of a landing - force
            // zero throttle
            channel_throttle->servo_out = 0;
        }
        break;
        
    default:
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        steer_state.hold_course_cd = -1;
        auto_state.land_complete = false;
        auto_state.land_sink_rate = 0;
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;
    }
}

/*
  main flight mode dependent update code 
 */
static void update_flight_mode(void)
{
    enum FlightMode effective_mode = control_mode;
    if (control_mode == AUTO && g.auto_fbw_steer) {
        effective_mode = FLY_BY_WIRE_A;
    }

    if (effective_mode != AUTO) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    switch (effective_mode) 
    {
    case AUTO:
        handle_auto_mode();
        break;

    case RTL:
    case LOITER:
    case GUIDED:
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;
        
    case TRAINING: {
        training_manual_roll = false;
        training_manual_pitch = false;
        
        // if the roll is past the set roll limit, then
        // we set target roll to the limit
        if (ahrs.roll_sensor >= roll_limit_cd) {
            nav_roll_cd = roll_limit_cd;
        } else if (ahrs.roll_sensor <= -roll_limit_cd) {
            nav_roll_cd = -roll_limit_cd;                
        } else {
            training_manual_roll = true;
            nav_roll_cd = 0;
        }
        
        // if the pitch is past the set pitch limits, then
        // we set target pitch to the limit
        if (ahrs.pitch_sensor >= aparm.pitch_limit_max_cd) {
            nav_pitch_cd = aparm.pitch_limit_max_cd;
        } else if (ahrs.pitch_sensor <= pitch_limit_min_cd) {
            nav_pitch_cd = pitch_limit_min_cd;
        } else {
            training_manual_pitch = true;
            nav_pitch_cd = 0;
        }
        if (fly_inverted()) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        break;
    }

    case ACRO: {
        // handle locked/unlocked control
        if (acro_state.locked_roll) {
            nav_roll_cd = acro_state.locked_roll_err;
        } else {
            nav_roll_cd = ahrs.roll_sensor;
        }
        if (acro_state.locked_pitch) {
            nav_pitch_cd = acro_state.locked_pitch_cd;
        } else {
            nav_pitch_cd = ahrs.pitch_sensor;
        }
        break;
    }

    case AUTOTUNE:
    case FLY_BY_WIRE_A: {
        // set nav_roll and nav_pitch using sticks
        nav_roll_cd  = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        float pitch_input = channel_pitch->norm_input();
        if (pitch_input > 0) {
            nav_pitch_cd = pitch_input * aparm.pitch_limit_max_cd;
        } else {
            nav_pitch_cd = -(pitch_input * pitch_limit_min_cd);
        }
        adjust_nav_pitch_throttle();
        nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
        if (fly_inverted()) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        if (failsafe.ch3_failsafe && g.short_fs_action == 2) {
            // FBWA failsafe glide
            nav_roll_cd = 0;
            nav_pitch_cd = 0;
            channel_throttle->servo_out = 0;
        }
        if (g.fbwa_tdrag_chan > 0) {
            // check for the user enabling FBWA taildrag takeoff mode
            bool tdrag_mode = (hal.rcin->read(g.fbwa_tdrag_chan-1) > 1700);
            if (tdrag_mode && !auto_state.fbwa_tdrag_takeoff_mode) {
                if (auto_state.highest_airspeed < g.takeoff_tdrag_speed1) {
                    auto_state.fbwa_tdrag_takeoff_mode = true;
                    gcs_send_text_P(SEVERITY_LOW, PSTR("FBWA tdrag mode\n"));
                }
            }
        }
        break;
    }

    case FLY_BY_WIRE_B:
        // Thanks to Yury MonZon for the altitude limit code!
        nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        update_fbwb_speed_height();
        break;
        
    case CRUISE:
        /*
          in CRUISE mode we use the navigation code to control
          roll when heading is locked. Heading becomes unlocked on
          any aileron or rudder input
        */
        if ((channel_roll->control_in != 0 ||
             channel_rudder->control_in != 0)) {                
            cruise_state.locked_heading = false;
            cruise_state.lock_timer_ms = 0;
        }                 
        
        if (!cruise_state.locked_heading) {
            nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
            nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
            update_load_factor();
        } else {
            calc_nav_roll();
        }
        update_fbwb_speed_height();
        break;
        
    case STABILIZE:
        nav_roll_cd        = 0;
        nav_pitch_cd       = 0;
        // throttle is passthrough
        break;
        
    case CIRCLE:
        // we have no GPS installed and have lost radio contact
        // or we just want to fly around in a gentle circle w/o GPS,
        // holding altitude at the altitude we set when we
        // switched into the mode
        nav_roll_cd  = roll_limit_cd / 3;
        update_load_factor();
        calc_nav_pitch();
        calc_throttle();
        break;

    case MANUAL:
        // servo_out is for Sim control only
        // ---------------------------------
        channel_roll->servo_out = channel_roll->pwm_to_angle();
        channel_pitch->servo_out = channel_pitch->pwm_to_angle();
        steering_control.steering = steering_control.rudder = channel_rudder->pwm_to_angle();
        break;
        //roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000
        
    case INITIALISING:
        // handled elsewhere
        break;
    }
}

static void update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    // distance and bearing calcs only
    switch(control_mode) {
    case AUTO:
        update_commands();
        break;
            
    case RTL:
        if (g.rtl_autoland && 
            !auto_state.checked_for_autoland &&
            nav_controller->reached_loiter_target() && 
            labs(altitude_error_cm) < 1000) {
            // we've reached the RTL point, see if we have a landing sequence
            jump_to_landing_sequence();

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        // fall through to LOITER

    case LOITER:
    case GUIDED:
        // allow loiter direction to be changed in flight
        if (g.loiter_radius < 0) {
            loiter.direction = -1;
        } else {
            loiter.direction = 1;
        }
        update_loiter();
        break;

    case CRUISE:
        update_cruise();
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case INITIALISING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CIRCLE:
        // nothing to do
        break;
    }
}

/*
  set the flight stage
 */
static void set_flight_stage(AP_SpdHgtControl::FlightStage fs) 
{
    //if just now entering land flight stage
    if (fs == AP_SpdHgtControl::FLIGHT_LAND_APPROACH &&
        flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {

#if GEOFENCE_ENABLED == ENABLED 
        if (g.fence_autoenable == 1) {
            if (! geofence_set_enabled(false, AUTO_TOGGLED)) {
                gcs_send_text_P(SEVERITY_HIGH, PSTR("Disable fence failed (autodisable)"));
            } else {
                gcs_send_text_P(SEVERITY_HIGH, PSTR("Fence disabled (autodisable)"));
            }
        }
#endif
    }
    
    flight_stage = fs;
}

static void update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }

    geofence_check(true);

    update_flight_stage();
}

/*
  recalculate the flight_stage
 */
static void update_flight_stage(void)
{
    // Update the speed & height controller states
    if (auto_throttle_mode && !throttle_suppressed) {        
        if (control_mode==AUTO) {
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_TAKEOFF);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND && 
                       auto_state.land_complete == true) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_FINAL);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_APPROACH); 
            } else {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
            }
        } else {
            set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
        }

        SpdHgt_Controller->update_pitch_throttle(relative_target_altitude_cm(),
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 auto_state.takeoff_pitch_cd,
                                                 throttle_nudge,
                                                 relative_altitude(),
                                                 aerodynamic_load_factor);
        if (should_log(MASK_LOG_TECS)) {
            Log_Write_TECS_Tuning();
        }
    }

    // tell AHRS the airspeed to true airspeed ratio
    airspeed.set_EAS2TAS(barometer.get_EAS2TAS());
}

#if OPTFLOW == ENABLED
// called at 50hz
static void update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        // Use range from a separate range finder if available, not the PX4Flows built in sensor which is ineffective
        float ground_distance_m = 0.01f*rangefinder.distance_cm();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update, rangefinder_state.in_range_count, ground_distance_m);
        Log_Write_Optflow();
    }
}
#endif
AP_HAL_MAIN();
