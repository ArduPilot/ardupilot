/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V2.0.44 Beta"
/*
ArduCopter Version 2.0 Beta
Authors:	Jason Short
Based on code and ideas from the Arducopter team: Jose Julio, Randy Mackay, Jani Hirvinen
Thanks to:	Chris Anderson, Mike Smith, Jordi Munoz, Doug Weibel, James Goppert, Benjamin Pelletier


This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

Special Thanks for Contributors:

Hein Hollander 		:Octo Support
Dani Saez 			:V Ocoto Support
Max Levine			:Tri Support, Graphics
Jose Julio			:Stabilization Control laws
Randy MacKay		:Heli Support
Jani Hiriven		:Testing feedback
Andrew Tridgell		:Mavlink Support
James Goppert		:Mavlink Support
Doug Weibel			:Libraries
Mike Smith			:Libraries, Coding support
HappyKillmore		:Mavlink GCS
Michael Oborne		:Mavlink GCS
Jack Dunkle			:Alpha testing
Christof Schmid		:Alpha testing
Oliver				:Piezo support
Guntars				:Arming safety suggestion

And much more so PLEASE PM me on DIYDRONES to add your contribution to the List

*/

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_GPS.h>         // ArduPilot GPS library
#include <Wire.h>			// Arduino I2C lib
#include <SPI.h>			// Arduino SPI lib
#include <DataFlash.h>      // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <APM_BMP085.h>     // ArduPilot Mega BMP085 Library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
#include <AP_DCM.h>         // ArduPilot Mega DCM Library
#include <APM_PI.h>            	// PI library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <AP_OpticalFlow.h> // Optical Flow library
#include <ModeFilter.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <memcheck.h>

// Configuration
#include "defines.h"
#include "config.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"
#include "HIL.h"

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console
FastSerialPort1(Serial1);       // GPS port
FastSerialPort3(Serial3);       // Telemetry port

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
static AP_Int8                *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

	// real sensors
	AP_ADC_ADS7844          adc;
	APM_BMP085_Class        barometer;
    AP_Compass_HMC5843      compass(Parameters::k_param_compass);


	// real GPS selection
	#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
		AP_GPS_Auto     g_gps_driver(&Serial1, &g_gps);

	#elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
		AP_GPS_NMEA     g_gps_driver(&Serial1);

	#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
		AP_GPS_SIRF     g_gps_driver(&Serial1);

	#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
		AP_GPS_UBLOX    g_gps_driver(&Serial1);

	#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
		AP_GPS_MTK      g_gps_driver(&Serial1);

	#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK16
		AP_GPS_MTK16    g_gps_driver(&Serial1);

	#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
		AP_GPS_None     g_gps_driver(NULL);

	#else
		#error Unrecognised GPS_PROTOCOL setting.
	#endif // GPS PROTOCOL

#elif HIL_MODE == HIL_MODE_SENSORS
	// sensor emulators
	AP_ADC_HIL              adc;
	APM_BMP085_HIL_Class    barometer;
	AP_Compass_HIL          compass;
	AP_GPS_HIL              g_gps_driver(NULL);

#elif HIL_MODE == HIL_MODE_ATTITUDE
	AP_ADC_HIL              adc;
	AP_DCM_HIL              dcm;
	AP_GPS_HIL              g_gps_driver(NULL);
	AP_Compass_HIL          compass; // never used
	AP_IMU_Shim             imu; // never used
	#ifdef OPTFLOW_ENABLED
		AP_OpticalFlow_ADNS3080 optflow;
	#endif
    static int32_t          gps_base_alt;
#else
	#error Unrecognised HIL_MODE setting.
#endif // HIL MODE

#if HIL_MODE != HIL_MODE_DISABLED
	#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK
		GCS_MAVLINK	hil(Parameters::k_param_streamrates_port0);
	#elif HIL_PROTOCOL == HIL_PROTOCOL_XPLANE
		HIL_XPLANE hil;
	#endif // HIL PROTOCOL
#endif // HIL_MODE

//  We may have a hil object instantiated just for mission planning
#if HIL_MODE == HIL_MODE_DISABLED && HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && HIL_PORT == 0
	GCS_MAVLINK	hil(Parameters::k_param_streamrates_port0);
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE
	#if HIL_MODE != HIL_MODE_SENSORS
		// Normal
		AP_IMU_Oilpan imu(&adc, Parameters::k_param_IMU_calibration);
	#else
		// hil imu
		AP_IMU_Shim imu;
	#endif
	// normal dcm
	AP_DCM  dcm(&imu, g_gps);

 	#ifdef OPTFLOW_ENABLED
	AP_OpticalFlow_ADNS3080 optflow;
 	#endif
#endif

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
#if   GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
	GCS_MAVLINK	gcs(Parameters::k_param_streamrates_port3);
#else
	// If we are not using a GCS, we need a stub that does nothing.
	GCS_Class           gcs;
#endif

//#include <GCS_SIMPLE.h>
//GCS_SIMPLE    gcs_simple(&Serial);

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilter sonar_mode_filter;

#if SONAR_TYPE == MAX_SONAR_XL
	AP_RangeFinder_MaxsonarXL sonar(&adc, &sonar_mode_filter);//(SONAR_PORT, &adc);
#else
    #error Unrecognised SONAR_TYPE setting.
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////
static const char *comma = ",";

static const char* flight_mode_strings[] = {
	"STABILIZE",
	"ACRO",
	"ALT_HOLD",
	"AUTO",
	"GUIDED",
	"LOITER",
	"RTL",
	"CIRCLE"};

/* Radio values
		Channel assignments
			1	Ailerons (rudder if no ailerons)
			2	Elevator
			3	Throttle
			4	Rudder (if we have ailerons)
			5	Mode - 3 position switch
			6 	User assignable
			7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
			8	TBD
*/

// test
//Vector3f accels_rot;
//float	accel_gain = 20;

// temp
int y_actual_speed;
int y_rate_error;

// calc the
int x_actual_speed;
int x_rate_error;

// Radio
// -----
static byte 	control_mode		= STABILIZE;
static byte 	old_control_mode	= STABILIZE;
static byte 	oldSwitchPosition;					// for remembering the control mode switch
static int 		motor_out[8];
static bool		do_simple = false;

// Heli
// ----
#if FRAME_CONFIG ==	HELI_FRAME
static float heli_rollFactor[3], heli_pitchFactor[3];  // only required for 3 swashplate servos
static int heli_servo_min[3], heli_servo_max[3];       // same here.  for yaw servo we use heli_servo4_min/max parameter directly
static int heli_servo_out[4];
#endif

// Failsafe
// --------
static boolean 		failsafe;						// did our throttle dip below the failsafe value?
static boolean 		ch3_failsafe;
static boolean		motor_armed;
static boolean		motor_auto_armed;				// if true,

// PIDs
// ----
//int 	max_stabilize_dampener;				//
//int 	max_yaw_dampener;					//
static Vector3f omega;
float tuning_value;

// LED output
// ----------
static boolean motor_light;						// status of the Motor safety
static boolean GPS_light;							// status of the GPS light
static byte	led_mode = NORMAL_LEDS;

// GPS variables
// -------------
static const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
static float 	scaleLongUp			= 1;			// used to reverse longitude scaling
static float 	scaleLongDown 		= 1;			// used to reverse longitude scaling
static byte 	ground_start_count	= 10;			// have we achieved first lock and set Home?
static bool 	did_ground_start	= false;		// have we ground started after first arming

// Location & Navigation
// ---------------------
static const float radius_of_earth 	= 6378100;		// meters
static const float gravity 			= 9.81;			// meters/ sec^2
//static long		nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
static long		target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target

//static bool		xtrack_enabled = false;
//static long		crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
//static long		crosstrack_correction;				// deg * 100 : 0 to 360 desired angle of plane to target

static int		climb_rate;							// m/s * 100  - For future implementation of controlled ascent/descent by rate
static byte	wp_control;							// used to control - navgation or loiter

static byte	command_must_index;					// current command memory location
static byte	command_may_index;					// current command memory location
static byte	command_must_ID;					// current command ID
static byte	command_may_ID;						// current command ID
static byte wp_verify_byte;						// used for tracking state of navigating waypoints

static float cos_roll_x 	= 1;
static float cos_pitch_x 	= 1;
static float cos_yaw_x 		= 1;
static float sin_pitch_y, sin_yaw_y, sin_roll_y;
static long initial_simple_bearing;				// used for Simple mode
static float simple_sin_y, simple_cos_x;

// Acro
#if CH7_OPTION == CH7_FLIP
static bool do_flip = false;
#endif

// Airspeed
// --------
static int		airspeed;							// m/s * 100

// Location Errors
// ---------------
//static long		bearing_error;						// deg * 100 : 0 to 36000
static long		altitude_error;						// meters * 100 we are off in altitude
static long 	old_altitude;
static long 	yaw_error;							// how off are we pointed
static long		long_error, lat_error;				// temp for debugging

// Battery Sensors
// ---------------
static float	battery_voltage		= LOW_VOLTAGE * 1.05;		// Battery Voltage of total battery, initialized above threshold for filter
static float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cell 1, initialized above threshold for filter
static float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2, initialized above threshold for filter
static float 	battery_voltage3 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3, initialized above threshold for filter
static float 	battery_voltage4 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3 + 4, initialized above threshold for filter

static float	current_amps;
static float	current_total;
static bool		low_batt = false;

// Barometer Sensor variables
// --------------------------
static long 	abs_pressure;
static long 	ground_pressure;
static int 		ground_temperature;

// Altitude Sensor variables
// ----------------------
static int		sonar_alt;
static int		baro_alt;
//static int		baro_alt_offset;
static byte 	altitude_sensor = BARO;				// used to know which sensor is active, BARO or SONAR
static int		altitude_rate;

// flight mode specific
// --------------------
static byte		yaw_mode;
static byte		roll_pitch_mode;
static byte		throttle_mode;

static boolean	takeoff_complete;					// Flag for using take-off controls
static boolean	land_complete;
//static int		landing_distance;					// meters;
static long 	old_alt;							// used for managing altitude rates
static int		velocity_land;
static byte 	yaw_tracking = MAV_ROI_WPNEXT;		// no tracking, point at next wp, or at a target

// Loiter management
// -----------------
static long 	original_target_bearing;			// deg * 100, used to check we are not passing the WP
static long 	old_target_bearing;					// used to track difference in angle

static int		loiter_total; 						// deg : how many times to loiter * 360
static int		loiter_sum;							// deg : how far we have turned around a waypoint
static long 	loiter_time;						// millis : when we started LOITER mode
static int 		loiter_time_max;					// millis : how long to stay in LOITER mode


// these are the values for navigation control functions
// ----------------------------------------------------
static long		nav_roll;							// deg * 100 : target roll angle
static long		nav_pitch;							// deg * 100 : target pitch angle
static long		nav_yaw;							// deg * 100 : target yaw angle
static long		auto_yaw;							// deg * 100 : target yaw angle
static long		nav_lat;							// for error calcs
static long		nav_lon;							// for error calcs
static int		nav_throttle;						// 0-1000 for throttle control

static unsigned long throttle_integrator;				// used to integrate throttle output to predict battery life
static bool 	invalid_throttle;					// used to control when we calculate nav_throttle
//static bool 	set_throttle_cruise_flag = false;	// used to track the throttle crouse value

static long 	command_yaw_start;					// what angle were we to begin with
static unsigned long 	command_yaw_start_time;				// when did we start turning
static unsigned int	command_yaw_time;					// how long we are turning
static long 	command_yaw_end;					// what angle are we trying to be
static long 	command_yaw_delta;					// how many degrees will we turn
static int		command_yaw_speed;					// how fast to turn
static byte		command_yaw_dir;
static byte		command_yaw_relative;

static int 	auto_level_counter;

// Waypoints
// ---------
static long	wp_distance;						// meters - distance between plane and next waypoint
static long	wp_totalDistance;					// meters - distance between old and next waypoint
//static byte	next_wp_index;						// Current active command index

// repeating event control
// -----------------------
static byte 	event_id; 							// what to do - see defines
static unsigned long 	event_timer; 						// when the event was asked for in ms
static unsigned int 	event_delay; 						// how long to delay the next firing of event in millis
static int 		event_repeat;						// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
static int 		event_value; 						// per command value, such as PWM for servos
static int 		event_undo_value;					// the value used to undo commands
//static byte 	repeat_forever;
static byte 	undo_event;							// counter for timing the undo

// delay command
// --------------
static long 	condition_value;					// used in condition commands (eg delay, change alt, etc.)
static long 	condition_start;
static int 		condition_rate;

// land command
// ------------
static long 	land_start;							// when we intiated command in millis()
static long 	original_alt;						// altitide reference for start of command

// 3D Location vectors
// -------------------
static struct 	Location home;						// home location
static struct 	Location prev_WP;					// last waypoint
static struct 	Location current_loc;				// current location
static struct 	Location next_WP;					// next waypoint
static struct 	Location target_WP;					// where do we want to you towards?
static struct 	Location simple_WP;					//
static struct 	Location next_command;				// command preloaded
static struct   Location guided_WP;					// guided mode waypoint
static long 	target_altitude;					// used for
static boolean	home_is_set; 						// Flag for if we have g_gps lock and have set the home location
static boolean  new_location;						// flag to tell us if location has been updated

// IMU variables
// -------------
static float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)

// Performance monitoring
// ----------------------
static long 			perf_mon_timer;
//static float 			imu_health; 						// Metric based on accel gain deweighting
static int 				G_Dt_max;							// Max main loop cycle time in milliseconds
static int 				gps_fix_count;
static byte				gps_watchdog;

// System Timers
// --------------
static unsigned long 	fast_loopTimer;				// Time in miliseconds of main control loop
static byte 			medium_loopCounter;			// Counters for branching from main control loop to slower loops
static unsigned long	throttle_timer;

static unsigned long	fiftyhz_loopTimer;

static byte 			slow_loopCounter;
static int 				superslow_loopCounter;
static byte				simple_timer;				// for limiting the execution of flight mode thingys


static float 			dTnav;						// Delta Time in milliseconds for navigation computations
static unsigned long 	nav_loopTimer;				// used to track the elapsed ime for GPS nav

static byte				counter_one_herz;
static bool				GPS_enabled 	= false;
static byte				loop_step;
static bool				new_radio_frame;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {
	memcheck_init();
	init_ardupilot();
}

void loop()
{
	long timer 			= micros();
	// We want this to execute fast
	// ----------------------------
	if ((timer - fast_loopTimer) >= 4000) {
		//PORTK |= B00010000;
		G_Dt 				= (float)(timer - fast_loopTimer) / 1000000.f;		// used by PI Loops
		fast_loopTimer 		= timer;

		// Execute the fast loop
		// ---------------------
		fast_loop();
	}
	//PORTK &= B11101111;

	if ((timer - fiftyhz_loopTimer) >= 20000) {
		fiftyhz_loopTimer		= timer;
		//PORTK |= B01000000;

		// reads all of the necessary trig functions for cameras, throttle, etc.
		update_trig();

		medium_loop();

		// Stuff to run at full 50hz, but after the loops
		fifty_hz_loop();

		counter_one_herz++;

		if(counter_one_herz == 50){
			super_slow_loop();
			counter_one_herz = 0;
		}

		if (millis() - perf_mon_timer > 20000) {
			gcs.send_message(MSG_PERF_REPORT);

			if (g.log_bitmask & MASK_LOG_PM)
				Log_Write_Performance();

			resetPerfData();
        }
		//PORTK &= B10111111;
	}
}
//  PORTK |= B01000000;
//	PORTK &= B10111111;

// Main loop
static void fast_loop()
{
    // try to send any deferred messages if the serial port now has
    // some space available
    gcs.send_message(MSG_RETRY_DEFERRED);
#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && (HIL_MODE != HIL_MODE_DISABLED || HIL_PORT == 0)
    hil.send_message(MSG_RETRY_DEFERRED);
#endif

	// Read radio
	// ----------
	read_radio();

	// IMU DCM Algorithm
	read_AHRS();

	// Look for slow loop times
	// ------------------------
	//if (delta_ms_fast_loop > G_Dt_max)
	//	G_Dt_max = delta_ms_fast_loop;

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_yaw_mode();
	update_roll_pitch_mode();

	// write out the servo PWM values
	// ------------------------------
	set_servos_4();
}

static void medium_loop()
{
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS and Compass
		//-----------------------------------------
		case 0:
			loop_step = 1;

			medium_loopCounter++;

			#ifdef OPTFLOW_ENABLED
			if(g.optflow_enabled){
				optflow.read();
				optflow.update_position(dcm.roll, dcm.pitch, cos_yaw_x, sin_yaw_y, current_loc.alt);  // updates internal lon and lat with estimation based on optical flow

				// write to log
				if (g.log_bitmask & MASK_LOG_OPTFLOW){
					Log_Write_Optflow();
				}
			}
			#endif

			if(GPS_enabled){
				update_GPS();
			}

			//readCommands();

			#if HIL_MODE != HIL_MODE_ATTITUDE
				if(g.compass_enabled){
					compass.read();		 						// Read magnetometer
					compass.calculate(dcm.get_dcm_matrix());  	// Calculate heading
					compass.null_offsets(dcm.get_dcm_matrix());
				}
			#endif

			// auto_trim, uses an auto_level algorithm
			auto_trim();

			// record throttle output
			// ------------------------------
			throttle_integrator += g.rc_3.servo_out;
			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			loop_step = 2;
			medium_loopCounter++;

			// Auto control modes:
			if(g_gps->new_data && g_gps->fix){
				loop_step = 11;

				// invalidate GPS data
				g_gps->new_data 	= false;

				// we are not tracking I term on navigation, so this isn't needed
				dTnav 				= (float)(millis() - nav_loopTimer)/ 1000.0;
				nav_loopTimer 		= millis();

				// calculate the copter's desired bearing and WP distance
				// ------------------------------------------------------
				if(navigate()){

					// control mode specific updates
					// -----------------------------
					update_navigation();

					if (g.log_bitmask & MASK_LOG_NTUN)
						Log_Write_Nav_Tuning();
				}
			}

			break;

		// command processing
		//-------------------
		case 2:
			loop_step = 3;
			medium_loopCounter++;

			// Read altitude from sensors
			// --------------------------
			update_altitude();

			// invalidate the throttle hold value
			// ----------------------------------
			invalid_throttle = true;

			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			loop_step = 4;
			medium_loopCounter++;

			// perform next command
			// --------------------
			if(control_mode == AUTO){
				update_commands();
			}

			#if HIL_MODE != HIL_MODE_ATTITUDE
				if(motor_armed){
					if (g.log_bitmask & MASK_LOG_ATTITUDE_MED)
						Log_Write_Attitude();

					if (g.log_bitmask & MASK_LOG_CTUN)
						Log_Write_Control_Tuning();
				}
			#endif

			#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
				gcs.data_stream_send(5,45);
				// send all requested output streams with rates requested
				// between 5 and 45 Hz
			#else
				gcs.send_message(MSG_ATTITUDE);     // Sends attitude data
			#endif

			#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && (HIL_MODE != HIL_MODE_DISABLED || HIL_PORT == 0)
				hil.data_stream_send(5,45);
			#endif

			if (g.log_bitmask & MASK_LOG_MOTORS)
				Log_Write_Motors();

			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
			loop_step = 5;
			medium_loopCounter = 0;

			if (g.battery_monitoring != 0){
				read_battery();
			}

			// Accel trims 		= hold > 2 seconds
			// Throttle cruise  = switch less than 1 second
			// --------------------------------------------
			read_trim_switch();

			// Check for engine arming
			// -----------------------
			arm_motors();


			slow_loop();
			break;

		default:
			// this is just a catch all
			// ------------------------
			medium_loopCounter = 0;
			break;
	}

}

// stuff that happens at 50 hz
// ---------------------------
static void fifty_hz_loop()
{
	// moved to slower loop
	// --------------------
	update_throttle_mode();

	// Read Sonar
	// ----------
	if(g.sonar_enabled){
		sonar_alt = sonar.read();
	}

	#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && HIL_MODE != HIL_MODE_DISABLED
		// HIL for a copter needs very fast update of the servo values
		hil.send_message(MSG_RADIO_OUT);
	#endif

	camera_stabilization();

	# if HIL_MODE == HIL_MODE_DISABLED
		if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)
			Log_Write_Attitude();

		if (g.log_bitmask & MASK_LOG_RAW)
			Log_Write_Raw();
	#endif

	#if HIL_MODE != HIL_MODE_DISABLED && HIL_PORT != GCS_PORT
		// kick the HIL to process incoming sensor packets
		hil.update();

		#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK
			hil.data_stream_send(45,1000);
		#else
			hil.send_message(MSG_SERVO_OUT);
		#endif
	#elif HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && HIL_MODE == HIL_MODE_DISABLED && HIL_PORT == 0
		// Case for hil object on port 0 just for mission planning
		hil.update();
		hil.data_stream_send(45,1000);
	#endif

	// kick the GCS to process uplink data
	gcs.update();

	#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
		gcs.data_stream_send(45,1000);
	#endif

	#if FRAME_CONFIG == TRI_FRAME
		// servo Yaw
		g.rc_4.calc_pwm();
		APM_RC.OutputCh(CH_7, g.rc_4.radio_out);
	#endif
}


static void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			loop_step = 6;
			slow_loopCounter++;
			superslow_loopCounter++;

			if(superslow_loopCounter > 800){ // every 4 minutes
				#if HIL_MODE != HIL_MODE_ATTITUDE
					if(g.rc_3.control_in == 0 && g.compass_enabled){
						compass.save_offsets();
						superslow_loopCounter = 0;
					}
				#endif
            }
			break;

		case 1:
			loop_step = 7;
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			read_control_switch();

			// Read main battery voltage if hooked up - does not read the 5v from radio
			// ------------------------------------------------------------------------
			//#if BATTERY_EVENT == 1
			//	read_battery();
			//#endif

			#if AUTO_RESET_LOITER == 1
			if(control_mode == LOITER){
				//if((abs(g.rc_2.control_in) + abs(g.rc_1.control_in)) > 1500){
					// reset LOITER to current position
					//next_WP 	= current_loc;
				//}
			}
			#endif

			break;

		case 2:
			loop_step = 8;
			slow_loopCounter = 0;
			update_events();

			// blink if we are armed
			update_lights();

			#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
				gcs.data_stream_send(1,5);
				// send all requested output streams with rates requested
				// between 1 and 5 Hz
			#else
				gcs.send_message(MSG_LOCATION);
				//gcs.send_message(MSG_CPU_LOAD, load*100);
			#endif

			#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && (HIL_MODE != HIL_MODE_DISABLED || HIL_PORT == 0)
				hil.data_stream_send(1,5);
			#endif

			if(g.radio_tuning > 0)
				tuning();

			#if MOTOR_LEDS == 1
				update_motor_leds();
			#endif

			break;

		default:
			slow_loopCounter = 0;
			break;

	}
}

// 1Hz loop
static void super_slow_loop()
{
	loop_step = 9;
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();

    gcs.send_message(MSG_HEARTBEAT);

	#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && (HIL_MODE != HIL_MODE_DISABLED || HIL_PORT == 0)
		hil.send_message(MSG_HEARTBEAT);
	#endif
}

static void update_GPS(void)
{
	loop_step = 10;
	g_gps->update();
	update_GPS_light();

	//current_loc.lng =   377697000;		// Lon * 10 * *7
	//current_loc.lat = -1224318000;		// Lat * 10 * *7
	//current_loc.alt = 100;				// alt * 10 * *7
	//return;
	if(gps_watchdog < 10){
		gps_watchdog++;
	}else{
		// we have lost GPS signal for a moment. Reduce our error to avoid flyaways
		nav_roll  >>= 1;
		nav_pitch >>= 1;
	}

    if (g_gps->new_data && g_gps->fix) {
		gps_watchdog = 0;

		// XXX We should be sending GPS data off one of the regular loops so that we send
		// no-GPS-fix data too
		#if GCS_PROTOCOL != GCS_PROTOCOL_MAVLINK
			gcs.send_message(MSG_LOCATION);
		#endif

		// for performance
		// ---------------
		gps_fix_count++;

		if(ground_start_count > 1){
			ground_start_count--;

		} else if (ground_start_count == 1) {

			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				//SendDebugln("!! bad loc");
				ground_start_count = 5;

			}else{
				//Serial.printf("init Home!");

				// reset our nav loop timer
				//nav_loopTimer = millis();
				init_home();

				// init altitude
				// commented out because we aren't using absolute altitude
				// current_loc.alt = home.alt;
				ground_start_count = 0;
			}
		}

		current_loc.lng = g_gps->longitude;	// Lon * 10 * *7
		current_loc.lat = g_gps->latitude;	// Lat * 10 * *7

		if (g.log_bitmask & MASK_LOG_GPS){
			Log_Write_GPS();
		}
	}
}


void update_yaw_mode(void)
{
	switch(yaw_mode){
		case YAW_ACRO:
			g.rc_4.servo_out = get_rate_yaw(g.rc_4.control_in);
			return;
			break;

		case YAW_HOLD:
			// calcualte new nav_yaw offset
			if (control_mode <= STABILIZE){
				nav_yaw = get_nav_yaw_offset(g.rc_4.control_in, g.rc_3.control_in);
			}else{
				nav_yaw = get_nav_yaw_offset(g.rc_4.control_in, 1);
			}
			break;

		case YAW_LOOK_AT_HOME:
			//nav_yaw updated in update_navigation()
			break;

		case YAW_AUTO:
			nav_yaw += constrain(wrap_180(auto_yaw - nav_yaw), -20, 20);
			nav_yaw  = wrap_360(nav_yaw);
			break;
	}

	// Yaw control
	g.rc_4.servo_out = get_stabilize_yaw(nav_yaw);

	//Serial.printf("4: %d\n",g.rc_4.servo_out);
}

void update_roll_pitch_mode(void)
{
	#if CH7_OPTION == CH7_FLIP
	if (do_flip){
		roll_flip();
		return;
	}
	#endif

	int control_roll = 0, control_pitch = 0;

	//read_radio();

	if(do_simple && new_radio_frame){
		new_radio_frame = false;
		simple_timer++;

		int delta = wrap_360(dcm.yaw_sensor - initial_simple_bearing)/100;

		if (simple_timer == 1){
			// roll
			simple_cos_x = sin(radians(90 - delta));

		}else if (simple_timer > 2){
			// pitch
			simple_sin_y = cos(radians(90 - delta));
			simple_timer = 0;
		}

		// Rotate input by the initial bearing
		control_roll 	= g.rc_1.control_in * simple_cos_x + g.rc_2.control_in * simple_sin_y;
		control_pitch 	= -(g.rc_1.control_in * simple_sin_y - g.rc_2.control_in * simple_cos_x);

		g.rc_1.control_in = control_roll;
		g.rc_2.control_in = control_pitch;
	}


	switch(roll_pitch_mode){
		case ROLL_PITCH_ACRO:
			// Roll control
			g.rc_1.servo_out = get_rate_roll(g.rc_1.control_in);

			// Pitch control
			g.rc_2.servo_out = get_rate_pitch(g.rc_2.control_in);
			break;

		case ROLL_PITCH_STABLE:
			g.rc_1.servo_out = get_stabilize_roll(g.rc_1.control_in);
			g.rc_2.servo_out = get_stabilize_pitch(g.rc_2.control_in);
			break;

		case ROLL_PITCH_AUTO:
			// mix in user control with Nav control
			control_roll 		= g.rc_1.control_mix(nav_roll);
			control_pitch 		= g.rc_2.control_mix(nav_pitch);
			g.rc_1.servo_out 	= get_stabilize_roll(control_roll);
			g.rc_2.servo_out 	= get_stabilize_pitch(control_pitch);
			break;
	}

}


// 50 hz update rate, not 250
void update_throttle_mode(void)
{
	switch(throttle_mode){

		case THROTTLE_MANUAL:
			if (g.rc_3.control_in > 0){
				g.rc_3.servo_out = g.rc_3.control_in + get_angle_boost();
			}else{
				g.rc_3.servo_out = 0;
			}
			break;

		case THROTTLE_HOLD:
			// allow interactive changing of atitude
			adjust_altitude();
			// fall through

		case THROTTLE_AUTO:
				// 10hz, 			don't run up i term
			if(invalid_throttle && motor_auto_armed == true){

				// how far off are we
				altitude_error = get_altitude_error();

				// get the AP throttle
				nav_throttle = get_nav_throttle(altitude_error, 200); //150 =  target speed of 1.5m/s
				//Serial.printf("in:%d, cr:%d, NT:%d, I:%1.4f\n", g.rc_3.control_in,altitude_error,  nav_throttle, g.pi_throttle.get_integrator());

				// clear the new data flag
				invalid_throttle = false;
			}

			// apply throttle control at 200 hz
			g.rc_3.servo_out = g.throttle_cruise + nav_throttle + get_angle_boost();
			break;
	}
}

// called after a GPS read
static void update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------
	switch(control_mode){
		case AUTO:
			verify_commands();
			// note: wp_control is handled by commands_logic

			// calculates desired Yaw
			update_auto_yaw();

			// calculates the desired Roll and Pitch
			update_nav_wp();
			break;

		case GUIDED:
			wp_control = WP_MODE;

			update_auto_yaw();
			update_nav_wp();
			break;

		case RTL:
			if(wp_distance > 4){
				// calculates desired Yaw
				// XXX this is an experiment
				#if FRAME_CONFIG ==	HELI_FRAME
				update_auto_yaw();
				#endif

				wp_control = WP_MODE;
			}else{
				// lets just jump to Loiter Mode after RTL
				set_mode(LOITER);
				//xtrack_enabled = false;
			}

			// calculates the desired Roll and Pitch
			update_nav_wp();
			break;

			// switch passthrough to LOITER
		case LOITER:
			wp_control 		= LOITER_MODE;

			// calculates the desired Roll and Pitch
			update_nav_wp();
			break;

		case CIRCLE:
			yaw_tracking	= MAV_ROI_WPNEXT;
			wp_control 		= CIRCLE_MODE;

			// calculates desired Yaw
			update_auto_yaw();
			update_nav_wp();
			break;

	}

	if(yaw_mode == YAW_LOOK_AT_HOME){
		if(home_is_set){
			//nav_yaw = point_at_home_yaw();
			nav_yaw = get_bearing(&current_loc, &home);
		} else {
			nav_yaw = 0;
		}
	}
}

static void read_AHRS(void)
{
	// Perform IMU calculations and get attitude info
	//-----------------------------------------------
	#if HIL_MODE == HIL_MODE_SENSORS
		// update hil before dcm update
		hil.update();
	#endif

	dcm.update_DCM_fast();
	omega = dcm.get_gyro();
}

static void update_trig(void){
	Vector2f yawvector;
	Matrix3f temp 	= dcm.get_dcm_matrix();

	yawvector.x 	= temp.a.x; // sin
	yawvector.y 	= temp.b.x;	// cos
	yawvector.normalize();


	sin_pitch_y 	= -temp.c.x;
	cos_pitch_x 	= sqrt(1 - (temp.c.x * temp.c.x));

	cos_roll_x 		= temp.c.z / cos_pitch_x;
	sin_roll_y 		= temp.c.y / cos_pitch_x;

	cos_yaw_x 		= yawvector.y;	// 0 x = north
	sin_yaw_y 		= yawvector.x;	// 1 y

	//flat:
	// 0 ° = cp: 1.00, sp: 0.00, cr: 1.00, sr:  0.00, cy:  0.00, sy:  1.00,
	// 90° = cp: 1.00, sp: 0.00, cr: 1.00, sr:  0.00, cy:  1.00, sy:  0.00,
	// 180 = cp: 1.00, sp: 0.10, cr: 1.00, sr: -0.01, cy:  0.00, sy: -1.00,
	// 270 = cp: 1.00, sp: 0.10, cr: 1.00, sr: -0.01, cy: -1.00, sy:  0.00,


	//Vector3f accel_filt	= imu.get_accel_filtered();
	//accels_rot 	= dcm.get_dcm_matrix() * imu.get_accel_filtered();
}

// updated at 10hz
static void update_altitude()
{
	altitude_sensor = BARO;

	#if HIL_MODE == HIL_MODE_ATTITUDE
	current_loc.alt = g_gps->altitude - gps_base_alt;
	return;
	#else

	if(g.sonar_enabled){
		// filter out offset
		float scale;

		// read barometer
		baro_alt 			= read_barometer();

		if(baro_alt < 1000){

			#if SONAR_TILT_CORRECTION == 1
				// correct alt for angle of the sonar
				float temp = cos_pitch_x * cos_roll_x;
				temp = max(temp, 0.707);
				sonar_alt = (float)sonar_alt * temp;
			#endif

			scale = (sonar_alt - 400) / 200;
			scale = constrain(scale, 0, 1);
			current_loc.alt = ((float)sonar_alt * (1.0 - scale)) + ((float)baro_alt * scale) + home.alt;
		}else{
			current_loc.alt = baro_alt + home.alt;
		}

	}else{
		baro_alt 		= read_barometer();
		// no sonar altitude
		current_loc.alt = baro_alt + home.alt;
	}

	altitude_rate 	= (current_loc.alt - old_altitude) * 10; // 10 hz timer
	old_altitude 	= current_loc.alt;
	#endif
}

static void
adjust_altitude()
{
	if(g.rc_3.control_in <= 200){
		next_WP.alt -= 1;												// 1 meter per second
		next_WP.alt = max(next_WP.alt, (current_loc.alt - 400));		// don't go less than 4 meters below current location
		next_WP.alt = max(next_WP.alt, 100);							// don't go less than 1 meter
	}else if (g.rc_3.control_in > 700){
		next_WP.alt += 1;												// 1 meter per second
		next_WP.alt = min(next_WP.alt, (current_loc.alt + 400));		// don't go more than 4 meters below current location
	}
}

static void tuning(){
	tuning_value = (float)g.rc_6.control_in / 1000.0;

	switch(g.radio_tuning){
		case CH6_STABILIZE_KP:
			g.rc_6.set_range(0,8000); 		// 0 to 8
			g.pi_stabilize_roll.kP(tuning_value);
			g.pi_stabilize_pitch.kP(tuning_value);
			break;

		case CH6_STABILIZE_KI:
			g.rc_6.set_range(0,300); 		// 0 to .3
			tuning_value = (float)g.rc_6.control_in / 1000.0;
			g.pi_stabilize_roll.kI(tuning_value);
			g.pi_stabilize_pitch.kI(tuning_value);
			break;

		case CH6_RATE_KP:
			g.rc_6.set_range(0,300);		 // 0 to .3
			g.pi_rate_roll.kP(tuning_value);
			g.pi_rate_pitch.kP(tuning_value);
			break;

		case CH6_RATE_KI:
			g.rc_6.set_range(0,300);		 // 0 to .3
			g.pi_rate_roll.kI(tuning_value);
			g.pi_rate_pitch.kI(tuning_value);
			break;

		case CH6_YAW_KP:
			g.rc_6.set_range(0,1000);
			g.pi_stabilize_yaw.kP(tuning_value);
			break;

		case CH6_YAW_RATE_KP:
			g.rc_6.set_range(0,1000);
			g.pi_rate_yaw.kP(tuning_value);
			break;

		case CH6_THROTTLE_KP:
			g.rc_6.set_range(0,1000);
			g.pi_throttle.kP(tuning_value);
			break;

		case CH6_TOP_BOTTOM_RATIO:
			g.rc_6.set_range(800,1000); 	// .8 to 1
			g.top_bottom_ratio = tuning_value;
			break;

		case CH6_RELAY:
			g.rc_6.set_range(0,1000);
		  	if (g.rc_6.control_in > 525) relay_on();
		  	if (g.rc_6.control_in < 475) relay_off();
			break;

		case CH6_TRAVERSE_SPEED:
			g.rc_6.set_range(0,1000);
			g.waypoint_speed_max = g.rc_6.control_in;
			break;

		case CH6_NAV_P:
			g.rc_6.set_range(0,6000);
			g.pi_nav_lat.kP(tuning_value);
			g.pi_nav_lon.kP(tuning_value);
			break;
	}
}

static void update_nav_wp()
{
	if(wp_control == LOITER_MODE){

		// calc a pitch to the target
		calc_location_error(&next_WP);

		// use error as the desired rate towards the target
		calc_nav_rate(long_error, lat_error, g.waypoint_speed_max, 0);

	}else if(wp_control == CIRCLE_MODE){

		// check if we have missed the WP
		int loiter_delta = (target_bearing - old_target_bearing)/100;

		// reset the old value
		old_target_bearing = target_bearing;

		// wrap values
		if (loiter_delta > 180) loiter_delta -= 360;
		if (loiter_delta < -180) loiter_delta += 360;

		// sum the angle around the WP
		loiter_sum += abs(loiter_delta);


		// creat a virtual waypoint that circles the next_WP
		// Count the degrees we have circulated the WP
		int circle_angle = wrap_360(target_bearing + 3000 + 18000) / 100;

		target_WP.lng = next_WP.lng + (g.loiter_radius * cos(radians(90 - circle_angle)));
		target_WP.lat = next_WP.lat + (g.loiter_radius * sin(radians(90 - circle_angle)));

		// calc the lat and long error to the target
		calc_location_error(&target_WP);

		// use error as the desired rate towards the target
		// nav_lon, nav_lat is calculated
		calc_nav_rate(long_error, lat_error, 200, 0);

	} else {
		// for long journey's reset the wind resopnse
		// it assumes we are standing still.
		g.pi_loiter_lat.reset_I();
		g.pi_loiter_lat.reset_I();

		// calc the lat and long error to the target
		calc_location_error(&next_WP);

		// use error as the desired rate towards the target
		calc_nav_rate(long_error, lat_error, g.waypoint_speed_max, 100);

	}
	// rotate pitch and roll to the copter frame of reference
	calc_nav_pitch_roll();
}

static void update_auto_yaw()
{
	// this tracks a location so the copter is always pointing towards it.
	if(yaw_tracking == MAV_ROI_LOCATION){
		auto_yaw = get_bearing(&current_loc, &target_WP);

	}else if(yaw_tracking == MAV_ROI_WPNEXT){
		auto_yaw = target_bearing;
	}
	// MAV_ROI_NONE = basic Yaw hold
}



