/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduRover v2.30"

// This is the APMrover firmware derived from the Arduplane v2.32 by Jean-Louis Naudin (JLN) 
/*
Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin
Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 
Please contribute your ideas!
APMrover alpha version tester: Franco Borasio, Daniel Chapelat... 

This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
//
// JLN updates: last update 2012-06-21
// DOLIST:
//-------------------------------------------------------------------------------------------------------------------------
// Dev Startup : 2012-04-21
//
//  2012-06-21: Update for HIL mode with mavlink 1.0 (new lib)
//  2012-06-13: use RangeFinder optical SharpGP2Y instead of ultrasonic sonar
//  2012-06-13: added Test sonar
//  2012-05-17: added speed_boost during straight line
//  2012-05-17: New update about the throttle rate control based on the field test done by Franco Borasio (Thanks Franco..)
//  2012-05-15: The Throttle rate can be controlled by the THROTTLE_SLEW_LIMIT (the value give the step increase, 1 = 0.1)
//  2012-05-14: Update about mavlink library (now compatible with the latest version of mavlink)
//  2012-05-14: Added option (hold roll to full right + SW7 ON/OFF) to init_home during the wp_list reset
//  2012-05-13: Add ROV_SONAR_TRIG (default = 200 cm)
//  2012-05-13: Restart_nav() added and heading bug correction, tested OK in the field
//  2012-05-12: RTL then stop update - Tested in the field
//  2012-05-11: The rover now STOP after the RTL... (special update for Franco...)
//  2012-05-11: Added SONAR detection for obstacle avoidance (alpha version for SONAR testing)
//  2012-05-04: Added #define LITE ENABLED  for the APM1280 or APM2560 CPU IMUless version
//  2012-05-03: Successful missions tests with a full APM2560 kit (GPS MT3329 + magnetometer HMC5883L)
//  2012-05-03: removing stick mixing in auto mode
//  2012-05-01: special update for rover about ground_course if compass is enabled
//  2012-04-30: Successfully tested in autonomous nav with a waypoints list recorded in live mode
//  2012-04-30: Now a full version for APM v1 or APM v2 with magnetometer
//  2012-04-27: Cosmetic changes
//  2012-04-26: Only one PID (pidNavRoll) for steering the wheel with nav_roll
//  2012-04-26: Added ground_speed and ground_course variables in Update_GPS
//  2012-04-26: Set GPS to 10 Hz (updated in the AP_GPS lib)
//  2012-04-22: Tested on Traxxas Monster Jam Grinder XL-5 3602
//  2012-04-21: Roll set to wheels control and Throttle neutral to 50% (0 -100)  - Forward>50, Backward<50
//
// Radio setup:
// APM INPUT (Rec = receiver)
// Rec ch1: Roll 
// Rec ch2: Throttle
// Rec ch3: Pitch
// Rec ch4: Yaw
// Rec ch5: not used
// Rec ch6: not used
// Rec ch7: Option channel to 2 positions switch
// Rec ch8: Mode channel to 3 positions switch
// APM OUTPUT
// Ch1: Wheel servo (direction)
// Ch2: not used
// Ch3: to the motor ESC
// Ch4: not used
//
// more infos about this experimental version: http://diydrones.com/profile/JeanLouisNaudin
// =======================================================================================================
*/

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
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <Filter.h>			// Filter library
#include <AP_Buffer.h>      // FIFO buffer library
#include <ModeFilter.h>		// Mode Filter from Filter library
#include <AverageFilter.h>	// Mode Filter from Filter library
#include <AP_Relay.h>       // APM relay
#include <AP_Mount.h>		// Camera/Antenna mount
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Airspeed.h>    // needed for AHRS build
#include <memcheck.h>
#include <DataFlash.h>
#include <SITL.h>
#include <stdarg.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include "compat.h"

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
AP_Param param_loader(var_info, WP_START_BYTE);

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


////////////////////////////////////////////////////////////////////////////////
// prototypes
static void update_events(void);

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#endif


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

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8		*flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
#if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844          adc;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
static AP_Compass_HIL compass;
static SITL sitl;
#else
static AP_Compass_HMC5843      compass;
#endif

// real GPS selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK16
AP_GPS_MTK16    g_gps_driver();

#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver();

#else
 #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL

# if CONFIG_INS_TYPE == CONFIG_INS_MPU6000
  AP_InertialSensor_MPU6000 ins;
# elif CONFIG_INS_TYPE == CONFIG_INS_SITL
  AP_InertialSensor_Stub ins;
#else
  AP_InertialSensor_Oilpan ins( &adc );
#endif // CONFIG_INS_TYPE

AP_AHRS_DCM  ahrs(&ins, g_gps);

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL              adc;
AP_Compass_HIL          compass;
AP_GPS_HIL              g_gps_driver(NULL);
AP_InertialSensor_Oilpan ins( &adc );
AP_AHRS_DCM  ahrs(&ins, g_gps);

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_ADC_HIL              adc;
AP_AHRS_HIL             ahrs(&ins, g_gps);
AP_GPS_HIL              g_gps_driver(NULL);
AP_Compass_HIL          compass; // never used
#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
GCS_MAVLINK	gcs0;
GCS_MAVLINK	gcs3;

// a pin for reading the receiver RSSI voltage. The scaling by 0.25 
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource *rssi_analog_source;

AP_HAL::AnalogSource *vcc_pin;

AP_HAL::AnalogSource * batt_volt_pin;
AP_HAL::AnalogSource * batt_curr_pin;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size5 sonar_mode_filter(2);
#if CONFIG_SONAR == ENABLED
    AP_HAL::AnalogSource *sonar_analog_source;
    AP_RangeFinder_MaxsonarXL *sonar;
#endif

// relay support
AP_Relay relay;

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
AP_Mount camera_mount(g_gps, &dcm);
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// APM2 only
#if USB_MUX_PIN > 0
static bool usb_connected;
#endif

static const char *comma = ",";

/* Radio values
		Channel assignments
			1   Steering
			2   ---
			3   Throttle
			4   ---
			5   Aux5
			6   Aux6
			7   Aux7
			8   Aux8/Mode
		Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
		See libraries/RC_Channel/RC_Channel_aux.h for more information
*/

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
uint8_t    control_mode        = INITIALISING;
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
// Used for failsafe based on loss of RC signal or GCS signal
static int16_t 	failsafe;					
// Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
// RC receiver should be set up to output a low throttle value when signal is lost
static bool 	ch3_failsafe;

// A timer used to track how long since we have received the last GCS heartbeat or rc override message
static uint32_t rc_override_fs_timer = 0;
// A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
static uint32_t ch3_failsafe_timer = 0;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// state of the GPS light (on/off)
static bool GPS_light;							

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const 	float t7			= 10000000.0;	
// We use atan2 and other trig techniques to calaculate angles

// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t 	ground_start_count	= 5;
// Used to compute a speed estimate from the first valid gps fixes to decide if we are 
// on the ground or in the air.  Used to decide if a ground start is appropriate if we
// booted with an air start.
static int16_t     ground_start_avg;
static int32_t          gps_base_alt;		

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Constants
const	float radius_of_earth 	= 6378100;	// meters
const	float gravity 			= 9.81;		// meters/ sec^2


// true if we have a position estimate from AHRS
static bool have_position;

// This is the currently calculated direction to fly.  
// deg * 100 : 0 to 360
static int32_t nav_bearing;
// This is the direction to the next waypoint
// deg * 100 : 0 to 360
static int32_t target_bearing;	
//This is the direction from the last waypoint to the next waypoint 
// deg * 100 : 0 to 360
static int32_t crosstrack_bearing;
// A gain scaler to account for ground speed/headwind/tailwind
static float	nav_gain_scaler 		= 1;		
static bool rtl_complete = false;

// There may be two active commands in Auto mode.  
// This indicates the active navigation command by index number
static uint8_t	nav_command_index;					
// This indicates the active non-navigation command by index number
static uint8_t	non_nav_command_index;				
// This is the command type (eg navigate to waypoint) of the active navigation command
static uint8_t	nav_command_ID		= NO_COMMAND;	
static uint8_t	non_nav_command_ID	= NO_COMMAND;	

static float	groundspeed_error;	
// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t     throttle_nudge = 0;

// receiver RSSI
static uint8_t receiver_rssi;

// the time when the last HEARTBEAT message arrived from a GCS
static uint32_t last_heartbeat_ms;

// The distance as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t		sonar_dist;
static bool obstacle = false;

////////////////////////////////////////////////////////////////////////////////
// Ground speed
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  Centimeters per second
static int32_t 	groundspeed_undershoot = 0;
static int32_t 	ground_speed = 0;
static int16_t throttle_last = 0, throttle = 500;

////////////////////////////////////////////////////////////////////////////////
// Location Errors
////////////////////////////////////////////////////////////////////////////////
// Difference between current bearing and desired bearing.  Hundredths of a degree
static int32_t bearing_error;
// Difference between current altitude and desired altitude.  Centimeters
static int32_t altitude_error;
// Distance perpandicular to the course line that we are off trackline.  Meters 
static float	crosstrack_error;

////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////

// Used to track the CH7 toggle state.
// When CH7 goes LOW PWM from HIGH PWM, this value will have been set true
// This allows advanced functionality to know when to execute
static bool trim_flag;
// This register tracks the current Mission Command index when writing
// a mission using CH7 in flight
static int8_t CH7_wp_index;
float tuning_value;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery pack 1 voltage.  Initialized above the low voltage threshold to pre-load the filter and prevent low voltage events at startup.
static float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;
// Battery pack 1 instantaneous currrent draw.  Amperes
static float	current_amps1;
// Totalized current (Amp-hours) from battery 1
static float	current_total1;									

// To Do - Add support for second battery pack
//static float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery 2 Voltage, initialized above threshold for filter
//static float	current_amps2;									// Current (Amperes) draw from battery 2
//static float	current_total2;									// Totalized current (Amp-hours) from battery 2

// JLN Update
uint32_t  timesw                  = 0;
static bool speed_boost = false;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_roll;
// Calculated radius for the wp turn based on ground speed and max turn angle
static int32_t    wp_radius;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between plane and next waypoint.  Meters
static int32_t wp_distance;
// Distance between previous and next waypoint.  Meters
static int32_t wp_totalDistance;

////////////////////////////////////////////////////////////////////////////////
// repeating event control
////////////////////////////////////////////////////////////////////////////////
// Flag indicating current event type
static uint8_t 		event_id;
// when the event was started in ms
static int32_t 		event_timer;
// how long to delay the next firing of event in millis
static uint16_t 	event_delay;					
// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
static int16_t 		event_repeat = 0;
// per command value, such as PWM for servos
static int16_t 		event_value; 
// the value used to cycle events (alternate value to event_value)
static int16_t 		event_undo_value;

////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t 	condition_value;
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static int32_t 	condition_start;
// A value used in condition commands.  For example the rate at which to change altitude.
static int16_t 		condition_rate;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// The home location used for RTL.  The location is set when we first get stable GPS lock
static struct 	Location home;
// Flag for if we have g_gps lock and have set the home location
static bool	home_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static struct 	Location prev_WP;
// The plane's current location
static struct 	Location current_loc;
// The location of the current/active waypoint.  Used for track following
static struct 	Location next_WP;
// The location of the active waypoint in Guided mode.
static struct  	Location guided_WP;

// The location structure information from the Nav command being processed
static struct 	Location next_nav_command;	
// The location structure information from the Non-Nav command being processed
static struct 	Location next_nonnav_command;

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
static int16_t 	G_Dt_max = 0;
// The number of gps fixes recorded in the current performance monitoring interval
static int16_t 	gps_fix_count = 0;
// A variable used by developers to track performanc metrics.
// Currently used to record the number of GCS heartbeat messages received
static int16_t pmTest1 = 0;


////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in miliseconds of start of main control loop.  Milliseconds
static uint32_t 	fast_loopTimer;
// Time Stamp when fast loop was complete.  Milliseconds
static uint32_t 	fast_loopTimeStamp;
// Number of milliseconds used in last main loop cycle
static uint8_t 		delta_ms_fast_loop;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t			mainLoop_count;

// Time in miliseconds of start of medium control loop.  Milliseconds
static uint32_t 	medium_loopTimer;
// Counters for branching from main control loop to slower loops
static uint8_t 			medium_loopCounter;	
// Number of milliseconds used in last medium loop cycle
static uint8_t			delta_ms_medium_loop;

// Counters for branching from medium control loop to slower loops
static uint8_t 			slow_loopCounter;
// Counter to trigger execution of very low rate processes
static uint8_t 			superslow_loopCounter;
// Counter to trigger execution of 1 Hz processes
static uint8_t				counter_one_herz;

// % MCU cycles used
static float 			load;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {
	memcheck_init();
    cliSerial = hal.console;

    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE, 0.25);
    vcc_pin = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
    batt_volt_pin = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_pin = hal.analogin->channel(g.battery_curr_pin);

#if CONFIG_SONAR == ENABLED
 #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar_analog_source = new AP_ADC_AnalogSource(
        &adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
 #elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
    sonar_analog_source = hal.analogin->channel(
        CONFIG_SONAR_SOURCE_ANALOG_PIN);
 #else
  #warning "Invalid CONFIG_SONAR_SOURCE"
 #endif
    sonar = new AP_RangeFinder_MaxsonarXL(sonar_analog_source,
                                          &sonar_mode_filter);
#endif

	init_ardupilot();
}

void loop()
{
    // We want this to execute at 50Hz, but synchronised with the gyro/accel
    uint16_t num_samples = ins.num_samples_available();
    if (num_samples >= 1) {
		delta_ms_fast_loop	= millis() - fast_loopTimer;
		load                = (float)(fast_loopTimeStamp - fast_loopTimer)/delta_ms_fast_loop;
		G_Dt                = (float)delta_ms_fast_loop / 1000.f;
		fast_loopTimer      = millis();

		mainLoop_count++;

		// Execute the fast loop
		// ---------------------
		fast_loop();

		// Execute the medium loop
		// -----------------------
		medium_loop();

		counter_one_herz++;
		if(counter_one_herz == 50){
			one_second_loop();
			counter_one_herz = 0;
		}

		if (millis() - perf_mon_timer > 20000) {
			if (mainLoop_count != 0) {
  #if LITE == DISABLED
				if (g.log_bitmask & MASK_LOG_PM)
					#if HIL_MODE != HIL_MODE_ATTITUDE
					Log_Write_Performance();
					#endif
 #endif
				resetPerfData();
			}
		}

		fast_loopTimeStamp = millis();
    } else if (millis() - fast_loopTimeStamp < 19) {
        // less than 19ms has passed. We have at least one millisecond
        // of free time. The most useful thing to do with that time is
        // to accumulate some sensor readings, specifically the
        // compass, which is often very noisy but is not interrupt
        // driven, so it can't accumulate readings by itself
        if (g.compass_enabled) {
            compass.accumulate();
        }
    }
}

// Main loop 50Hz
static void fast_loop()
{
	// This is the fast loop - we want it to execute at 50Hz if possible
	// -----------------------------------------------------------------
	if (delta_ms_fast_loop > G_Dt_max)
		G_Dt_max = delta_ms_fast_loop;

	// Read radio
	// ----------
	read_radio();

    // try to send any deferred messages if the serial port now has
    // some space available
    gcs_send_message(MSG_RETRY_DEFERRED);

	// check for loss of control signal failsafe condition
	// ------------------------------------
	check_short_failsafe();

	#if HIL_MODE == HIL_MODE_SENSORS
		// update hil before dcm update
		gcs_update();
	#endif

#if LITE == DISABLED
	ahrs.update();
#endif 
	// Read Sonar
	// ----------
#if CONFIG_SONAR == ENABLED
	if(g.sonar_enabled){
		sonar_dist = sonar->read();

	if(sonar_dist <= g.sonar_trigger)  {  // obstacle detected in front 
            obstacle = true;
      } else  { 
            obstacle = false;
            }
	}
#endif

	// uses the yaw from the DCM to give more accurate turns
	calc_bearing_error();

#if LITE == DISABLED
	# if HIL_MODE == HIL_MODE_DISABLED
		if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)
			Log_Write_Attitude((int)ahrs.roll_sensor, (int)ahrs.pitch_sensor, (uint16_t)ahrs.yaw_sensor);

		if (g.log_bitmask & MASK_LOG_RAW)
			Log_Write_Raw();
	#endif
#endif
	// inertial navigation
	// ------------------
	#if INERTIAL_NAVIGATION == ENABLED
		// TODO: implement inertial nav function
		inertialNavigation();
	#endif

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_current_flight_mode();

	// apply desired roll, pitch and yaw to the plane
	// ----------------------------------------------
	if (control_mode > LEARNING)
		learning();

	// write out the servo PWM values
	// ------------------------------
	set_servos();


	// XXX is it appropriate to be doing the comms below on the fast loop?

    gcs_update();
    gcs_data_stream_send();
}

static void medium_loop()
{
#if MOUNT == ENABLED
	camera_mount.update_mount_position();
#endif

	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS
		//-------------------------------
		case 0:
			medium_loopCounter++;
            update_GPS();
            calc_gndspeed_undershoot();
            
//#if LITE == DISABLED
			#if HIL_MODE != HIL_MODE_ATTITUDE
            if (g.compass_enabled && compass.read()) {
                ahrs.set_compass(&compass);
                // Calculate heading
                compass.null_offsets();
            } else {
                ahrs.set_compass(NULL);
            }
			#endif
//#endif
/*{
cliSerial->print(ahrs.roll_sensor, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->print(ahrs.pitch_sensor, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->print(ahrs.yaw_sensor, DEC);	cliSerial->printf_P(PSTR("\t"));
Vector3f tempaccel = ins.get_accel();
cliSerial->print(tempaccel.x, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->print(tempaccel.y, DEC);	cliSerial->printf_P(PSTR("\t"));
cliSerial->println(tempaccel.z, DEC);
}*/

			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;
            navigate();
			break;

		// command processing
		//------------------------------
		case 2:
			medium_loopCounter++;

			// perform next command
			// --------------------
			update_commands();
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;
#if LITE == DISABLED
			#if HIL_MODE != HIL_MODE_ATTITUDE
				if ((g.log_bitmask & MASK_LOG_ATTITUDE_MED) && !(g.log_bitmask & MASK_LOG_ATTITUDE_FAST))
					Log_Write_Attitude((int)ahrs.roll_sensor, (int)ahrs.pitch_sensor, (uint16_t)ahrs.yaw_sensor);

				if (g.log_bitmask & MASK_LOG_CTUN)
					Log_Write_Control_Tuning();
			#endif

			if (g.log_bitmask & MASK_LOG_NTUN)
				Log_Write_Nav_Tuning();

			if (g.log_bitmask & MASK_LOG_GPS)
				Log_Write_GPS(g_gps->time, current_loc.lat, current_loc.lng, g_gps->altitude, current_loc.alt, (long) g_gps->ground_speed, g_gps->ground_course, g_gps->fix, g_gps->num_sats);
#endif
			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter = 0;
			delta_ms_medium_loop	= millis() - medium_loopTimer;
			medium_loopTimer      	= millis();

			if (g.battery_monitoring != 0){
				read_battery();
			}

			read_trim_switch();

			slow_loop();
			break;
	}
}

static void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			check_long_failsafe();
			superslow_loopCounter++;
			if(superslow_loopCounter >=200) {				//	200 = Execute every minute
#if LITE == DISABLED
				#if HIL_MODE != HIL_MODE_ATTITUDE
					if(g.compass_enabled) {
						compass.save_offsets();
					}
				#endif
#endif
				superslow_loopCounter = 0;
			}
			break;

		case 1:
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			read_control_switch();

			update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);

#if MOUNT == ENABLED
			camera_mount.update_mount_type();
#endif
			break;

		case 2:
			slow_loopCounter = 0;
                        
			update_events();

            mavlink_system.sysid = g.sysid_this_mav;		// This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter

#if USB_MUX_PIN > 0
            check_usb_mux();
#endif

#if TRACE == ENABLED
         //     cliSerial->printf_P(PSTR("NAV->gnd_crs=%3.0f, nav_brg=%3.0f, tgt_brg=%3.0f, brg_err=%3.0f, nav_rll=%3.1f rsvo=%3.1f\n"), 
         //           ahrs.yaw_sensor*0.01, (float)nav_bearing/100, (float)target_bearing/100, (float)bearing_error/100, (float)nav_roll/100, (float)g.channel_roll.servo_out/100);           
             // cliSerial->printf_P(PSTR("WPL->g.command_total=%d, g.command_index=%d, nav_command_index=%d\n"), 
                //    g.command_total, g.command_index, nav_command_index);      
    	   cliSerial->printf_P(PSTR("NAV->gnd_crs=%3.0f,  sonar_dist = %d    obstacle = %d\n"), ahrs.yaw_sensor*0.01, (int)sonar_dist, obstacle);                
#endif                          
			break;
	}
}

static void one_second_loop()
{
#if LITE == DISABLED
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();
#endif
	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);
}

static void update_GPS(void)
{        
	g_gps->update();
	update_GPS_light();

    have_position = ahrs.get_position(&current_loc);

	if (g_gps->new_data && g_gps->status() == GPS::GPS_OK) {
		gps_fix_count++;

		if(ground_start_count > 1){
			ground_start_count--;
			ground_start_avg += g_gps->ground_speed;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 5;

			} else {
                init_home();
				if (g.compass_enabled) {
					// Set compass declination automatically
					compass.set_initial_location(g_gps->latitude, g_gps->longitude);
				}
				ground_start_count = 0;
			}
		}
        ground_speed   = g_gps->ground_speed;
	}
}

static void update_current_flight_mode(void)
{ 
    switch(control_mode){
    case AUTO:
    case RTL:
        calc_nav_roll();
        calc_throttle();
        break;

    case LEARNING:
    case MANUAL:
        nav_roll        = 0;
        g.channel_roll.servo_out = g.channel_roll.pwm_to_angle();
        g.channel_pitch.servo_out = g.channel_pitch.pwm_to_angle();
        g.channel_rudder.servo_out = g.channel_roll.pwm_to_angle();
        break;
	}
}

static void update_navigation()
{
    switch (control_mode) {
    case AUTO:
		verify_commands();
        break;

    case RTL:
    case GUIDED:
        // no loitering around the wp with the rover, goes direct to the wp position
        calc_nav_roll();
        calc_bearing_error();
        if(verify_RTL()) {  
            g.channel_throttle.servo_out = g.throttle_min.get();
            set_mode(MANUAL);
        }
        break;
	}
}

AP_HAL_MAIN();
