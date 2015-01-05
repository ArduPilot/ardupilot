/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.47"
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
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Andrew Tridgell

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat... 

   Please contribute your ideas! See http://dev.ardupilot.com for details
*/

// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Steering
// Rec ch2: not used
// Rec ch3: Throttle
// Rec ch4: not used
// Rec ch5: not used
// Rec ch6: not used
// Rec ch7: Option channel to 2 position switch
// Rec ch8: Mode channel to 6 position switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <AP_NavEKF.h>
#include <AP_Mission.h>     // Mission command library
#include <AP_Rally.h>
#include <AP_Terrain.h>
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <Filter.h>			// Filter library
#include <Butter.h>			// Filter library - butterworth filter
#include <AP_Buffer.h>      // FIFO buffer library
#include <ModeFilter.h>		// Mode Filter from Filter library
#include <AverageFilter.h>	// Mode Filter from Filter library
#include <AP_Relay.h>       // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Mount.h>		// Camera/Antenna mount
#include <AP_Camera.h>		// Camera triggering
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Airspeed.h>    // needed for AHRS build
#include <AP_Vehicle.h>     // needed for AHRS build
#include <DataFlash.h>
#include <AP_RCMapper.h>        // RC input mapping library
#include <SITL.h>
#include <AP_Scheduler.h>       // main loop scheduler
#include <stdarg.h>
#include <AP_Navigation.h>
#include <APM_Control.h>
#include <AP_L1_Control.h>
#include <AP_BoardConfig.h>
#include <AP_Frsky_Telem.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include "compat.h"

#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library
#include <AP_OpticalFlow.h>     // Optical Flow library

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"

#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library

AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
AP_Param param_loader(var_info);

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
static Parameters      g;

// main loop scheduler
static AP_Scheduler scheduler;

// mapping between input channels
static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// primary control channels
static RC_Channel *channel_steer;
static RC_Channel *channel_throttle;
static RC_Channel *channel_learn;

////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);
void gcs_send_text_fmt(const prog_char_t *fmt, ...);
static void print_mode(AP_HAL::BetterStream *port, uint8_t mode);

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif defined(HAL_BOARD_LOG_DIRECTORY)
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);
#else
DataFlash_Empty DataFlash;
#endif

static bool in_log_download;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal driving mode.  Real sensors are used.
// - HIL Attitude mode.  Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode.  Synthetic sensors are configured that
//   supply data from the simulation.
//

// GPS driver
static AP_GPS gps;

// flight modes convenience array
static AP_Int8		*modes = &g.mode1;

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

// selected navigation controller
static AP_Navigation *nav_controller = &L1_controller;

// steering controller
static AP_SteerController steerController(ahrs);

////////////////////////////////////////////////////////////////////////////////
// Mission library
// forward declaration to avoid compiler errors
////////////////////////////////////////////////////////////////////////////////
static bool start_command(const AP_Mission::Mission_Command& cmd);
static bool verify_command(const AP_Mission::Mission_Command& cmd);
static void exit_mission();
AP_Mission mission(ahrs, &start_command, &verify_command, &exit_mission);

static OpticalFlow optflow;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

// a pin for reading the receiver RSSI voltage. The scaling by 0.25 
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource *rssi_analog_source;

////////////////////////////////////////////////////////////////////////////////
// SONAR
static RangeFinder sonar;

// relay support
AP_Relay relay;

AP_ServoRelayEvents ServoRelayEvents(relay);

// Camera
#if CAMERA == ENABLED
static AP_Camera camera(&relay);
#endif

// The rover's current location
static struct 	Location current_loc;


// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
AP_Mount camera_mount(&current_loc, ahrs, 0);
#endif


////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// if USB is connected
static bool usb_connected;

/* Radio values
		Channel assignments
			1   Steering
			2   ---
			3   Throttle
			4   ---
			5   Aux5
			6   Aux6
			7   Aux7/learn
			8   Aux8/Mode
		Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
		See libraries/RC_Channel/RC_Channel_aux.h for more information
*/

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
enum mode   control_mode        = INITIALISING;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
uint8_t 	oldSwitchPosition;
// These are values received from the GCS if the user is using GCS joystick
// control and are substituted for the values coming from the RC radio
static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
// A flag if GCS joystick control is in use
static bool rc_override_active = false;

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
// A tracking variable for type of failsafe active
// Used for failsafe based on loss of RC signal or GCS signal. See 
// FAILSAFE_EVENT_*
static struct {
    uint8_t bits;
    uint32_t rc_override_timer;
    uint32_t start_time;
    uint8_t triggered;
    uint32_t last_valid_rc_ms;
} failsafe;

// notification object for LEDs, buzzers etc (parameter set to false disables external leds)
static AP_Notify notify;

// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t 	ground_start_count	= 20;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Constants
const	float radius_of_earth 	= 6378100;	// meters


// true if we have a position estimate from AHRS
static bool have_position;

static bool rtl_complete = false;


// angle of our next navigation waypoint
static int32_t next_navigation_leg_cd;

// ground speed error in m/s
static float	groundspeed_error;	
// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t     throttle_nudge = 0;

// receiver RSSI
static uint8_t receiver_rssi;

// the time when the last HEARTBEAT message arrived from a GCS
static uint32_t last_heartbeat_ms;

// obstacle detection information
static struct {
    // have we detected an obstacle?
    uint8_t detected_count;
    float turn_angle;
    uint16_t sonar1_distance_cm;
    uint16_t sonar2_distance_cm;

    // time when we last detected an obstacle, in milliseconds
    uint32_t detected_time_ms;
} obstacle;

// this is set to true when auto has been triggered to start
static bool auto_triggered;

////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  meters per second
static float 	ground_speed = 0;
static int16_t throttle_last = 0, throttle = 500;

////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////

// Used to track the CH7 toggle state.
// When CH7 goes LOW PWM from HIGH PWM, this value will have been set true
// This allows advanced functionality to know when to execute
static bool ch7_flag;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
#if FRSKY_TELEM_ENABLED == ENABLED
static AP_Frsky_Telem frsky_telemetry(ahrs, battery);
#endif

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired lateral acceleration in m/s/s
static float lateral_acceleration;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between rover and next waypoint.  Meters
static float wp_distance;
// Distance between previous and next waypoint.  Meters
static int32_t wp_totalDistance;

////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t 	condition_value;
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static int32_t 	condition_start;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// The home location used for RTL.  The location is set when we first get stable GPS lock
static const struct	Location &home = ahrs.get_home();
// Flag for if we have gps lock and have set the home location
static bool	home_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static struct 	Location prev_WP;
// The location of the current/active waypoint.  Used for track following
static struct 	Location next_WP;
// The location of the active waypoint in Guided mode.
static struct  	Location guided_WP;

////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt						= 0.02;		

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static int32_t 	perf_mon_timer;
// The maximum main loop execution time recorded in the current performance monitoring interval
static uint32_t 	G_Dt_max;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of start of main control loop. 
static uint32_t 	fast_loopTimer_us;
// Number of milliseconds used in last main loop cycle
static uint32_t		delta_us_fast_loop;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t			mainLoop_count;

// set if we are driving backwards
static bool in_reverse;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

/*
  scheduler table - all regular tasks should be listed here, along
  with how often they should be called (in 20ms units) and the maximum
  time they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
	{ read_radio,             1,   1000 },
    { ahrs_update,            1,   6400 },
    { read_sonars,            1,   2000 },
    { update_current_mode,    1,   1500 },
    { set_servos,             1,   1500 },
    { update_GPS_50Hz,        1,   2500 },
    { update_GPS_10Hz,        5,   2500 },
    { update_alt,             5,   3400 },
    { navigate,               5,   1600 },
    { update_compass,         5,   2000 },
    { update_commands,        5,   1000 },
    { update_logging1,        5,   1000 },
    { update_logging2,        5,   1000 },
    { gcs_retry_deferred,     1,   1000 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { read_control_switch,   15,   1000 },
    { read_trim_switch,       5,   1000 },
    { read_battery,           5,   1000 },
    { read_receiver_rssi,     5,   1000 },
    { update_events,          1,   1000 },
    { check_usb_mux,         15,   1000 },
    { mount_update,           1,    600 },
    { gcs_failsafe_check,     5,    600 },
    { compass_accumulate,     1,    900 },
    { update_notify,          1,    300 },
    { one_second_loop,       50,   3000 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { telemetry_send,        10,    100 }	
#endif
};


/*
  setup is called when the sketch starts
 */
void setup() {
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // rover does not use arming nor pre-arm checks
    AP_Notify::flags.armed = true;
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);

    battery.init();

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);

	init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/*
  loop() is called rapidly while the sketch is running
 */
void loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = hal.scheduler->micros();

    delta_us_fast_loop	= timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = timer;

	if (delta_us_fast_loop > G_Dt_max)
		G_Dt_max = delta_us_fast_loop;

    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    scheduler.run(19500U);
}

// update AHRS system
static void ahrs_update()
{
    ahrs.set_armed(hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    // when in reverse we need to tell AHRS not to assume we are a
    // 'fly forward' vehicle, otherwise it will see a large
    // discrepancy between the mag and the GPS heading and will try to
    // correct for it, leading to a large yaw error
    ahrs.set_fly_forward(!in_reverse);

    ahrs.update();

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = pythagorous2(velocity.x, velocity.y);
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_IMU(ins);
}

/*
  update camera mount - 50Hz
 */
static void mount_update(void)
{
#if MOUNT == ENABLED
	camera_mount.update_mount_position();
#endif
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

static void update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
static void gcs_failsafe_check(void)
{
	if (g.fs_gcs_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_GCS, last_heartbeat_ms != 0 && (millis() - last_heartbeat_ms) > 2000);
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
  check for new compass data - 10Hz
 */
static void update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            Log_Write_Compass();
        }
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  log some key data - 10Hz
 */
static void update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST))
        Log_Write_Attitude();
    
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();

    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();
}

/*
  log some key data - 10Hz
 */
static void update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        if (control_mode == STEERING || control_mode == AUTO || control_mode == RTL || control_mode == GUIDED) {
            Log_Write_Steering();
        }
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();
}


/*
  update aux servo mappings
 */
static void update_aux(void)
{
    RC_Channel_aux::enable_aux_servos();
        
#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif
}

/*
  once a second events
 */
static void one_second_loop(void)
{
	if (should_log(MASK_LOG_CURRENT))
		Log_Write_Current();
	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    update_aux();

#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    static uint8_t counter;

    counter++;

    // write perf data every 20s
    if (counter % 10 == 0) {
        if (scheduler.debug() != 0) {
            hal.console->printf_P(PSTR("G_Dt_max=%lu\n"), (unsigned long)G_Dt_max);
        }
        if (should_log(MASK_LOG_PM))
            Log_Write_Performance();
        G_Dt_max = 0;
        resetPerfData();
    }

    // save compass offsets once a minute
    if (counter >= 60) {				
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }
}

static void update_GPS_50Hz(void)
{        
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
	gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }
        }
    }
}


static void update_GPS_10Hz(void)
{        
    have_position = ahrs.get_position(current_loc);

	if (have_position && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

		if (ground_start_count > 1){
			ground_start_count--;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 20;
			} else {
                init_home();

                // set system clock for log timestamps
                hal.util->set_system_clock(gps.time_epoch_usec());

				if (g.compass_enabled) {
					// Set compass declination automatically
					compass.set_initial_location(gps.location().lat, gps.location().lng);
				}
				ground_start_count = 0;
			}
		}
        Vector3f velocity;
        if (ahrs.get_velocity_NED(velocity)) {
            ground_speed = pythagorous2(velocity.x, velocity.y);
        } else {
            ground_speed   = gps.ground_speed();
        }

#if CAMERA == ENABLED
        if (camera.update_location(current_loc) == true) {
            do_take_picture();
        }
#endif        
	}
}

static void update_current_mode(void)
{ 
    switch (control_mode){
    case AUTO:
    case RTL:
    case GUIDED:
        set_reverse(false);
        calc_lateral_acceleration();
        calc_nav_steer();
        calc_throttle(g.speed_cruise);
        break;

    case STEERING: {
        /*
          in steering mode we control lateral acceleration
          directly. We first calculate the maximum lateral
          acceleration at full steering lock for this speed. That is
          V^2/R where R is the radius of turn. We get the radius of
          turn from half the STEER2SRV_P.
         */
        float max_g_force = ground_speed * ground_speed / steerController.get_turn_radius();

        // constrain to user set TURN_MAX_G
        max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

        lateral_acceleration = max_g_force * (channel_steer->pwm_to_angle()/4500.0f);
        calc_nav_steer();

        // and throttle gives speed in proportion to cruise speed, up
        // to 50% throttle, then uses nudging above that.
        float target_speed = channel_throttle->pwm_to_angle() * 0.01 * 2 * g.speed_cruise;
        set_reverse(target_speed < 0);
        if (in_reverse) {
            target_speed = constrain_float(target_speed, -g.speed_cruise, 0);
        } else {
            target_speed = constrain_float(target_speed, 0, g.speed_cruise);
        }
        calc_throttle(target_speed);
        break;
    }

    case LEARNING:
    case MANUAL:
        /*
          in both MANUAL and LEARNING we pass through the
          controls. Setting servo_out here actually doesn't matter, as
          we set the exact value in set_servos(), but it helps for
          logging
         */
        channel_throttle->servo_out = channel_throttle->control_in;
        channel_steer->servo_out = channel_steer->pwm_to_angle();

        // mark us as in_reverse when using a negative throttle to
        // stop AHRS getting off
        set_reverse(channel_throttle->servo_out < 0);
        break;

    case HOLD:
        // hold position - stop motors and center steering
        channel_throttle->servo_out = 0;
        channel_steer->servo_out = 0;
        set_reverse(false);
        break;

    case INITIALISING:
        break;
	}
}

static void update_navigation()
{
    switch (control_mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
    case INITIALISING:
        break;

    case AUTO:
		mission.update();
        break;

    case RTL:
    case GUIDED:
        // no loitering around the wp with the rover, goes direct to the wp position
        calc_lateral_acceleration();
        calc_nav_steer();
        if (verify_RTL()) {  
            channel_throttle->servo_out = g.throttle_min.get();
            set_mode(HOLD);
        }
        break;
	}
}

AP_HAL_MAIN();
