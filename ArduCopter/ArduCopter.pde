/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V2.1.1 alpha"
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

Requires modified "mrelax" version of Arduino, which can be found here:
http://code.google.com/p/ardupilot-mega/downloads/list

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
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_GPS.h>         // ArduPilot GPS library
#include <I2C.h>			// Arduino I2C lib
#include <SPI.h>			// Arduino SPI lib
#include <DataFlash.h>      // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
#include <AP_PeriodicProcess.h>         // Parent header of Timer
                                        // (only included for makefile libpath to work)
#include <AP_TimerProcess.h>            // TimerProcess is the scheduler for MPU6000 reads.
#include <AP_DCM.h>         // ArduPilot Mega DCM Library
#include <APM_PI.h>            	// PI library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <AP_OpticalFlow.h> // Optical Flow library
#include <ModeFilter.h>
#include <AP_Relay.h>		// APM relay
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <memcheck.h>

// Configuration
#include "defines.h"
#include "config.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"

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

Arduino_Mega_ISR_Registry isr_registry;

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
// RC Hardware
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
    APM_RC_APM2 APM_RC;
#else
    APM_RC_APM1 APM_RC;
#endif

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
    DataFlash_APM2 DataFlash;
#else
    DataFlash_APM1   DataFlash;
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
static AP_Int8                *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
#if CONFIG_ADC == ENABLED
	AP_ADC_ADS7844          adc;
#endif

#ifdef DESKTOP_BUILD
    AP_Baro_BMP085_HIL barometer;
    AP_Compass_HIL          compass;
#else

#if CONFIG_BARO == AP_BARO_BMP085
# if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
	AP_Baro_BMP085 barometer(true);
# else
	AP_Baro_BMP085 barometer(false);
# endif
#elif CONFIG_BARO == AP_BARO_MS5611
    AP_Baro_MS5611 barometer;
#endif

    AP_Compass_HMC5843      compass(Parameters::k_param_compass);
#endif

#ifdef OPTFLOW_ENABLED
	AP_OpticalFlow_ADNS3080 optflow;
#endif

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

#if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
AP_InertialSensor_MPU6000 ins( CONFIG_MPU6000_CHIP_SELECT_PIN );
#else
AP_InertialSensor_Oilpan ins(&adc);
#endif
AP_IMU_INS  imu(&ins, Parameters::k_param_IMU_calibration);
AP_DCM  dcm(&imu, g_gps);
AP_TimerProcess timer_scheduler;

#elif HIL_MODE == HIL_MODE_SENSORS
	// sensor emulators
	AP_ADC_HIL              adc;
	AP_Baro_BMP085_HIL      barometer;
	AP_Compass_HIL          compass;
	AP_GPS_HIL              g_gps_driver(NULL);
    AP_IMU_Shim imu;
    AP_DCM  dcm(&imu, g_gps);
    AP_PeriodicProcessStub timer_scheduler;
    AP_InertialSensor_Stub ins;

    static int32_t          gps_base_alt;

#elif HIL_MODE == HIL_MODE_ATTITUDE
	AP_ADC_HIL              adc;
	AP_DCM_HIL              dcm;
	AP_GPS_HIL              g_gps_driver(NULL);
	AP_Compass_HIL          compass; // never used
	AP_IMU_Shim             imu; // never used
    AP_InertialSensor_Stub ins;
    AP_PeriodicProcessStub timer_scheduler;
	#ifdef OPTFLOW_ENABLED
		AP_OpticalFlow_ADNS3080 optflow;
	#endif
    static int32_t          gps_base_alt;
#else
	#error Unrecognised HIL_MODE setting.
#endif // HIL MODE



////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
GCS_MAVLINK	gcs0(Parameters::k_param_streamrates_port0);
GCS_MAVLINK	gcs3(Parameters::k_param_streamrates_port3);

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilter sonar_mode_filter;
#if CONFIG_SONAR == ENABLED
	#if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
	AP_AnalogSource_ADC sonar_analog_source( &adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
	#elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
		AP_AnalogSource_Arduino sonar_analog_source(CONFIG_SONAR_SOURCE_ANALOG_PIN);
	#endif
	AP_RangeFinder_MaxsonarXL sonar(&sonar_analog_source, &sonar_mode_filter);
#endif

// agmatthews USERHOOKS
////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
#include USERHOOK_VARIABLES
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
	"CIRCLE",
	"POSITION",
	"LAND"};

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

// temp
static int16_t x_GPS_speed;
static int16_t y_GPS_speed;
static int16_t x_actual_speed;
static int16_t y_actual_speed;
static int16_t x_rate_error;
static int16_t y_rate_error;

// Radio
// -----
static byte 	control_mode		= STABILIZE;
static byte 	old_control_mode	= STABILIZE;
static byte 	oldSwitchPosition;					// for remembering the control mode switch
static int16_t  motor_out[11];
static int16_t  motor_filtered[11];					// added to try and deal with biger motors
static bool		do_simple 			= false;

static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
static bool rc_override_active = false;
static uint32_t rc_override_fs_timer = 0;


// Heli
// ----
#if FRAME_CONFIG ==	HELI_FRAME
static float heli_rollFactor[3], heli_pitchFactor[3];  // only required for 3 swashplate servos
static int16_t heli_servo_min[3], heli_servo_max[3];       // same here.  for yaw servo we use heli_servo4_min/max parameter directly
static int32_t heli_servo_out[4];                         // used for servo averaging for analog servos
static int16_t heli_servo_out_count = 0;                   // use for servo averaging
#endif

// Failsafe
// --------
static boolean 		failsafe;						// did our throttle dip below the failsafe value?
static boolean 		ch3_failsafe;
static boolean		motor_armed;
static boolean		motor_auto_armed;				// if true,

// PIDs
// ----
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
static bool	nav_ok;
static const float radius_of_earth 	= 6378100;		// meters
static const float gravity 			= 9.81;			// meters/ sec^2
static int32_t	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
static int32_t 	home_bearing;						// used to track difference in angle

static byte	wp_control;								// used to control - navgation or loiter

static byte	command_nav_index;						// current command memory location
static byte	prev_nav_index;
static byte	command_cond_index;						// current command memory location
//static byte	command_nav_ID;							// current command ID
//static byte	command_cond_ID;						// current command ID
static byte wp_verify_byte;							// used for tracking state of navigating waypoints

static float cos_roll_x 	= 1;
static float cos_pitch_x 	= 1;
static float cos_yaw_x 		= 1;
static float sin_pitch_y, sin_yaw_y, sin_roll_y;
static int32_t initial_simple_bearing;					// used for Simple mode
static float simple_sin_y, simple_cos_x;
static int8_t jump = -10;								// used to track loops in jump command
static int16_t waypoint_speed_gov;

static float circle_angle;
// replace with param
static const float circle_rate = 0.0872664625;

// Acro
#if CH7_OPTION == CH7_FLIP
static bool do_flip = false;
#endif

static boolean trim_flag;
static int8_t CH7_wp_index;

// Airspeed
// --------
static int16_t		airspeed;						// m/s * 100
static float 		thrust = .005;					// for estimating the velocity

// Location Errors
// ---------------
static int32_t	long_error, lat_error;				// temp for debugging

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
static int32_t 	abs_pressure;
static int32_t 	ground_pressure;
static int16_t 		ground_temperature;

// Altitude Sensor variables
// ----------------------
static byte 	altitude_sensor = BARO;				// used to know which sensor is active, BARO or SONAR
static int32_t		altitude_error;						// meters * 100 we are off in altitude

static int16_t		climb_rate;							// m/s * 100

static int16_t		sonar_alt;
static int16_t 		old_sonar_alt;
static int16_t		sonar_rate;

static int32_t		baro_alt;
static int32_t 		old_baro_alt;
static int16_t		baro_rate;



// flight mode specific
// --------------------
static byte		yaw_mode;
static byte		roll_pitch_mode;
static byte		throttle_mode;

static boolean	takeoff_complete;					// Flag for using take-off controls
static boolean	land_complete;
static int32_t 	old_alt;							// used for managing altitude rates
static int16_t	velocity_land;
static byte 	yaw_tracking = MAV_ROI_WPNEXT;		// no tracking, point at next wp, or at a target
static int16_t 	manual_boost;						// used in adjust altitude to make changing alt faster
static int16_t 	angle_boost;						// used in adjust altitude to make changing alt faster

// Loiter management
// -----------------
static int32_t 	original_target_bearing;			// deg * 100, used to check we are not passing the WP
static int32_t 	old_target_bearing;					// used to track difference in angle

static int16_t	loiter_total; 						// deg : how many times to loiter * 360
static int16_t	loiter_sum;							// deg : how far we have turned around a waypoint
static uint32_t loiter_time;       			// millis : when we started LOITER mode
static unsigned loiter_time_max;					// millis : how long to stay in LOITER mode


// these are the values for navigation control functions
// ----------------------------------------------------
static int32_t	nav_roll;							// deg * 100 : target roll angle
static int32_t	nav_pitch;							// deg * 100 : target pitch angle
static int32_t	nav_yaw;							// deg * 100 : target yaw angle
static int32_t	home_to_copter_bearing;				// deg * 100 : target yaw angle
static int32_t	auto_yaw;							// deg * 100 : target yaw angle
static int32_t	nav_lat;							// for error calcs
static int32_t	nav_lon;							// for error calcs
static int16_t	nav_throttle;						// 0-1000 for throttle control
static int16_t	crosstrack_error;

static uint32_t throttle_integrator;				// used to integrate throttle output to predict battery life
static float	throttle_avg = THROTTLE_CRUISE;
static bool 	invalid_throttle;					// used to control when we calculate nav_throttle
//static bool 	set_throttle_cruise_flag = false;	// used to track the throttle crouse value

static int32_t 	command_yaw_start;					// what angle were we to begin with
static uint32_t command_yaw_start_time;				// when did we start turning
static uint16_t	command_yaw_time;					// how long we are turning
static int32_t 	command_yaw_end;					// what angle are we trying to be
static int32_t 	command_yaw_delta;					// how many degrees will we turn
static int16_t	command_yaw_speed;					// how fast to turn
static byte		command_yaw_dir;
static byte		command_yaw_relative;

static int16_t 	auto_level_counter;

// Waypoints
// ---------
static int32_t	home_distance;						// meters - distance between plane and next waypoint
static int32_t	wp_distance;						// meters - distance between plane and next waypoint
static int32_t	wp_totalDistance;					// meters - distance between old and next waypoint
//static byte	next_wp_index;						// Current active command index

// repeating event control
// -----------------------
static byte    	event_id; 							// what to do - see defines
static uint32_t event_timer; 						// when the event was asked for in ms
static uint16_t event_delay; 						// how long to delay the next firing of event in millis
static int16_t 	event_repeat;						// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
static int16_t 	event_value; 						// per command value, such as PWM for servos
static int16_t 	event_undo_value;					// the value used to undo commands
//static byte 	repeat_forever;
//static byte 	undo_event;							// counter for timing the undo

// delay command
// --------------
static int32_t 	condition_value;					// used in condition commands (eg delay, change alt, etc.)
static int32_t 	condition_start;
//static int16_t 		condition_rate;

// land command
// ------------
static int32_t 	land_start;							// when we intiated command in millis()
static int32_t 	original_alt;						// altitide reference for start of command

// 3D Location vectors
// -------------------
static struct 	Location home;						// home location
static struct 	Location prev_WP;					// last waypoint
static struct 	Location current_loc;				// current location
static struct 	Location next_WP;					// next waypoint
static struct 	Location target_WP;					// where do we want to you towards?
static struct 	Location command_nav_queue;			// command preloaded
static struct 	Location command_cond_queue;		// command preloaded
static struct   Location guided_WP;					// guided mode waypoint
static int32_t 	target_altitude;					// used for
static boolean	home_is_set; 						// Flag for if we have g_gps lock and have set the home location

Vector3f accels_rot;

// this is just me playing with the sensors
// the 2 code is not functioning and you should try 1 instead
#if ACCEL_ALT_HOLD == 2
	static float Z_integrator;
	static float Z_gain = 3;
	static float Z_offset = 0;
#endif

// IMU variables
// -------------
static float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)

// Performance monitoring
// ----------------------
static int32_t 			perf_mon_timer;
//static float 			imu_health; 						// Metric based on accel gain deweighting
static int16_t			gps_fix_count;
static byte				gps_watchdog;
static int				pmTest1;

// System Timers
// --------------
static uint32_t 	    fast_loopTimer;				// Time in miliseconds of main control loop
static byte 			medium_loopCounter;			// Counters for branching from main control loop to slower loops

static uint32_t	        fiftyhz_loopTimer;

static byte 			slow_loopCounter;
static int16_t			superslow_loopCounter;
static byte				simple_timer;				// for limiting the execution of flight mode thingys


static float 			dTnav;						// Delta Time in milliseconds for navigation computations
static uint32_t         nav_loopTimer;				// used to track the elapsed ime for GPS nav

static byte				counter_one_herz;
static bool				GPS_enabled 	= false;
static bool				new_radio_frame;

AP_Relay relay;

#if USB_MUX_PIN > 0
	static bool usb_connected;
#endif

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {
	memcheck_init();
	init_ardupilot();
}

void loop()
{
	int32_t timer 			= micros();
	// We want this to execute fast
	// ----------------------------
	if ((timer - fast_loopTimer) >= 5000) {
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

		// update our velocity estimate based on IMU at 50hz
		// -------------------------------------------------
		esitmate_velocity();

		// perform 10hz tasks
		// ------------------
		medium_loop();

		// Stuff to run at full 50hz, but after the med loops
		// --------------------------------------------------
		fifty_hz_loop();

		counter_one_herz++;

		if(counter_one_herz == 50){
			super_slow_loop();
			counter_one_herz = 0;
		}

		if (millis() - perf_mon_timer > 1200 /*20000*/) {
			if (g.log_bitmask & MASK_LOG_PM)
				Log_Write_Performance();

			gps_fix_count 		= 0;
			perf_mon_timer 		= millis();
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
    gcs_send_message(MSG_RETRY_DEFERRED);

	// Read radio
	// ----------
	read_radio();

	// IMU DCM Algorithm
	read_AHRS();

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_yaw_mode();
	update_roll_pitch_mode();

	// write out the servo PWM values
	// ------------------------------
	set_servos_4();

	//if(motor_armed)
		//Log_Write_Attitude();

// agmatthews - USERHOOKS
#ifdef USERHOOK_FASTLOOP
   USERHOOK_FASTLOOP
#endif

}

static void medium_loop()
{
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS and Compass
		//-----------------------------------------
		case 0:
			medium_loopCounter++;

			if(GPS_enabled){
				update_GPS();
			}

			#ifdef OPTFLOW_ENABLED
			if(g.optflow_enabled){
				update_optical_flow();
			}
			#endif

			#if HIL_MODE != HIL_MODE_ATTITUDE					// don't execute in HIL mode
				if(g.compass_enabled){
					if (compass.read()) {
                        compass.calculate(dcm.get_dcm_matrix());  	// Calculate heading
                        compass.null_offsets(dcm.get_dcm_matrix());
                    }
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
			medium_loopCounter++;

			// Auto control modes:
			if(nav_ok){
				// clear nav flag
				nav_ok = false;

				// invalidate GPS data
				// -------------------
				g_gps->new_data 	= false;


				// calculate the copter's desired bearing and WP distance
				// ------------------------------------------------------
				if(navigate()){

					// this calculates the velocity for Loiter
					// only called when there is new data
					// ----------------------------------
					calc_XY_velocity();

					// If we have optFlow enabled we can grab a more accurate speed
					// here and override the speed from the GPS
					// ----------------------------------------
					#ifdef OPTFLOW_ENABLED
					if(g.optflow_enabled && current_loc.alt < 500){
						// optflow wont be enabled on 1280's
						x_GPS_speed 	= optflow.x_cm;
						y_GPS_speed 	= optflow.y_cm;
					}
					#endif
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
			medium_loopCounter++;

			// Read altitude from sensors
			// --------------------------
			#if HIL_MODE != HIL_MODE_ATTITUDE					// don't execute in HIL mode
			update_altitude();
			#endif

			// invalidate the throttle hold value
			// ----------------------------------
			invalid_throttle = true;

			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;

			// perform next command
			// --------------------
			if(control_mode == AUTO){
				if(home_is_set == true && g.command_total > 1){
					update_commands();
				}
			}

            if(motor_armed){
                if (g.log_bitmask & MASK_LOG_ATTITUDE_MED)
                    Log_Write_Attitude();

                if (g.log_bitmask & MASK_LOG_CTUN)
                    Log_Write_Control_Tuning();
            }

				// send all requested output streams with rates requested
				// between 5 and 45 Hz
				gcs_data_stream_send(5,45);

			if (g.log_bitmask & MASK_LOG_MOTORS)
				Log_Write_Motors();

			break;

		// This case controls the slow loop
		//---------------------------------
		case 4:
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

			// Do an extra baro read
			// ---------------------
			#if HIL_MODE != HIL_MODE_ATTITUDE
			barometer.read();
			#endif

			// agmatthews - USERHOOKS
			#ifdef USERHOOK_MEDIUMLOOP
			   USERHOOK_MEDIUMLOOP
			#endif

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
    # if CONFIG_SONAR == ENABLED
	if(g.sonar_enabled){
		sonar_alt = sonar.read();
	}
    #endif

	// agmatthews - USERHOOKS
	#ifdef USERHOOK_50HZLOOP
	  USERHOOK_50HZLOOP
	#endif

	#if HIL_MODE != HIL_MODE_DISABLED && FRAME_CONFIG != HELI_FRAME
		// HIL for a copter needs very fast update of the servo values
		gcs_send_message(MSG_RADIO_OUT);
	#endif

	camera_stabilization();

	# if HIL_MODE == HIL_MODE_DISABLED
		if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)
			Log_Write_Attitude();

		if (g.log_bitmask & MASK_LOG_RAW)
			Log_Write_Raw();
	#endif

	// kick the GCS to process uplink data
	gcs_update();
    gcs_data_stream_send(45,1000);

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
			slow_loopCounter++;
			superslow_loopCounter++;

			if(superslow_loopCounter > 1200){
				#if HIL_MODE != HIL_MODE_ATTITUDE
					if(g.rc_3.control_in == 0 && control_mode == STABILIZE && g.compass_enabled){
						compass.save_offsets();
						superslow_loopCounter = 0;
					}
				#endif
            }
			break;

		case 1:
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
				if((abs(g.rc_2.control_in) + abs(g.rc_1.control_in)) > 500){
					// reset LOITER to current position
					next_WP 	= current_loc;
					// clear Loiter Iterms
					reset_nav();
				}
			}
			#endif

			break;

		case 2:
			slow_loopCounter = 0;
			update_events();

			// blink if we are armed
			update_lights();

            // send all requested output streams with rates requested
            // between 3 and 5 Hz
            gcs_data_stream_send(3,5);

			if(g.radio_tuning > 0)
				tuning();

			#if MOTOR_LEDS == 1
				update_motor_leds();
			#endif

			#if USB_MUX_PIN > 0
            check_usb_mux();
			#endif
			break;

		default:
			slow_loopCounter = 0;
			break;

	}
	// agmatthews - USERHOOKS
	#ifdef USERHOOK_SLOWLOOP
	   USERHOOK_SLOWLOOP
	#endif

}

// 1Hz loop
static void super_slow_loop()
{
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();

    gcs_send_message(MSG_HEARTBEAT);
    gcs_data_stream_send(1,3);
	// agmatthews - USERHOOKS
	#ifdef USERHOOK_SUPERSLOWLOOP
	   USERHOOK_SUPERSLOWLOOP
	#endif

	/*
	Serial.printf("alt %d, next.alt %d, alt_err: %d, cruise: %d, Alt_I:%1.2f, wp_dist %d, tar_bear %d, home_d %d, homebear %d\n",
					current_loc.alt,
					next_WP.alt,
					altitude_error,
					g.throttle_cruise.get(),
					g.pi_alt_hold.get_integrator(),
					wp_distance,
					target_bearing,
					home_distance,
					home_to_copter_bearing);
	*/
}

// updated at 10 Hz
#ifdef OPTFLOW_ENABLED
static void update_optical_flow(void)
{
	optflow.read();
	optflow.update_position(dcm.roll, dcm.pitch, cos_yaw_x, sin_yaw_y, current_loc.alt);  // updates internal lon and lat with estimation based on optical flow

	// write to log
	if (g.log_bitmask & MASK_LOG_OPTFLOW){
		Log_Write_Optflow();
	}

	if(g.optflow_enabled && current_loc.alt < 500){
		if(GPS_enabled){
			// if we have a GPS, we add some detail to the GPS
			// XXX this may not ne right
			current_loc.lng += optflow.vlon;
			current_loc.lat += optflow.vlat;

			// some sort of error correction routine
			//current_loc.lng -= ERR_GAIN * (float)(current_loc.lng - x_GPS_speed); // error correction
			//current_loc.lng -= ERR_GAIN * (float)(current_loc.lng - x_GPS_speed); // error correction
		}else{
			// if we do not have a GPS, use relative from 0 for lat and lon
			current_loc.lng = optflow.vlon;
			current_loc.lat = optflow.vlat;
		}
		// OK to run the nav routines
		nav_ok = true;
	}
}
#endif

static void update_GPS(void)
{
	g_gps->update();
	update_GPS_light();

	//current_loc.lng =   377697000;		// Lon * 10 * *7
	//current_loc.lat = -1224318000;		// Lat * 10 * *7
	//current_loc.alt = 100;				// alt * 10 * *7
	//return;
	if(gps_watchdog < 12){
		gps_watchdog++;
	}else{
		// we have lost GPS signal for a moment. Reduce our error to avoid flyaways
		nav_roll  >>= 1;
		nav_pitch >>= 1;
	}

    if (g_gps->new_data && g_gps->fix) {
		gps_watchdog = 0;

		// OK to run the nav routines
		nav_ok = true;

		// for performance
		// ---------------
		gps_fix_count++;

		// we are not tracking I term on navigation, so this isn't needed
		dTnav 				= (float)(millis() - nav_loopTimer)/ 1000.0;
		nav_loopTimer 		= millis();

		// prevent runup from bad GPS
		dTnav = min(dTnav, 1.0);

		if(ground_start_count > 1){
			ground_start_count--;

		} else if (ground_start_count == 1) {

			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				ground_start_count = 5;

			}else{
				// block until we get a good fix
				// -----------------------------
				while (!g_gps->new_data || !g_gps->fix) {
					g_gps->update();
					// we need GCS update while waiting for GPS, to ensure
					// we react to HIL mavlink
					gcs_update();
				}
				init_home();
				ground_start_count = 0;
			}
		}

		current_loc.lng = g_gps->longitude;	// Lon * 10 * *7
		current_loc.lat = g_gps->latitude;	// Lat * 10 * *7

		if (g.log_bitmask & MASK_LOG_GPS){
			Log_Write_GPS();
		}

		#if HIL_MODE == HIL_MODE_ATTITUDE					// only execute in HIL mode
			update_altitude();
		#endif

	} else {
		g_gps->new_data = false;
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
			g.rc_1.servo_out = get_rate_roll(g.rc_1.control_in);
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

#define THROTTLE_FILTER_SIZE 4

// 50 hz update rate, not 250
void update_throttle_mode(void)
{
	int16_t throttle_out;

	switch(throttle_mode){
		case THROTTLE_MANUAL:
			if (g.rc_3.control_in > 0){
			    #if FRAME_CONFIG == HELI_FRAME
				    g.rc_3.servo_out = heli_get_angle_boost(g.rc_3.control_in);
				#else
					angle_boost = get_angle_boost(g.rc_3.control_in);
					g.rc_3.servo_out = g.rc_3.control_in + angle_boost;
				#endif
				// calc average throttle
				if ((g.rc_3.control_in > MINIMUM_THROTTLE)){
					//throttle_avg = throttle_avg * .98 + rc_3.control_in * .02;
					//g.throttle_cruise = throttle_avg;
				}
			}else{
				g.pi_stabilize_roll.reset_I();
				g.pi_stabilize_pitch.reset_I();
				g.pi_rate_roll.reset_I();
				g.pi_rate_pitch.reset_I();
				g.rc_3.servo_out = 0;
			}
			break;

		case THROTTLE_HOLD:
			// allow interactive changing of atitude
			adjust_altitude();
			// fall through

		case THROTTLE_AUTO:
			// calculate angle boost
			angle_boost = get_angle_boost(g.throttle_cruise);

			// manual command up or down?
			if(manual_boost != 0){
				#if FRAME_CONFIG == HELI_FRAME
					throttle_out = heli_get_angle_boost(g.throttle_cruise + manual_boost);
				#else
					throttle_out = g.throttle_cruise + angle_boost + manual_boost;
				#endif

				// reset next_WP.alt and don't go below 1 meter
				next_WP.alt = max(current_loc.alt, 100);

			}else{
				// 10hz, 			don't run up i term
				if(invalid_throttle && motor_auto_armed == true){

					// how far off are we
					altitude_error = get_altitude_error();

					// get the AP throttle
					nav_throttle = get_nav_throttle(altitude_error);

					// clear the new data flag
					invalid_throttle = false;
					/*
					Serial.printf("tar_alt: %d, actual_alt: %d \talt_err: %d, \tnav_thr: %d, \talt Int: %d, \trate_int %d \n",
										next_WP.alt,
										current_loc.alt,
										altitude_error,
										nav_throttle,
										(int16_t)g.pi_alt_hold.get_integrator(),
										(int16_t) g.pi_throttle.get_integrator());
					*/
				}

				#if FRAME_CONFIG == HELI_FRAME
					throttle_out = heli_get_angle_boost(g.throttle_cruise + nav_throttle + get_z_damping());
				#else
					throttle_out = g.throttle_cruise + nav_throttle + angle_boost + get_z_damping();
				#endif
			}

			// light filter of output
			g.rc_3.servo_out = (g.rc_3.servo_out * (THROTTLE_FILTER_SIZE - 1) + throttle_out) / THROTTLE_FILTER_SIZE;
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
			// note: wp_control is handled by commands_logic
			verify_commands();

			// calculates desired Yaw
			update_auto_yaw();

			// calculates the desired Roll and Pitch
			update_nav_wp();
			break;

		case GUIDED:
			wp_control = WP_MODE;
            // check if we are close to point > loiter
			wp_verify_byte = 0;
			verify_nav_wp();

			if (wp_control == WP_MODE) {
				update_auto_yaw();
			} else {
				set_mode(LOITER);
			}
			update_nav_wp();
			break;

		case RTL:
			if((wp_distance <= g.waypoint_radius) || check_missed_wp()){
				//lets just jump to Loiter Mode after RTL
				//if(land after RTL)
				//set_mode(LAND);
				//else
				set_mode(LOITER);

			}else{
				// calculates desired Yaw
				#if FRAME_CONFIG ==	HELI_FRAME
				update_auto_yaw();
				#endif

				wp_control = WP_MODE;
			}

			// calculates the desired Roll and Pitch
			update_nav_wp();
			break;

			// switch passthrough to LOITER
		case LOITER:
		case POSITION:
			wp_control 		= LOITER_MODE;

			// calculates the desired Roll and Pitch
			update_nav_wp();
			break;

		case LAND:
			wp_control 		= LOITER_MODE;

			if (current_loc.alt < 250){
				wp_control = NO_NAV_MODE;
				next_WP.alt = -200; // force us down
			}
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

	// are we in SIMPLE mode?
	if(do_simple && g.super_simple){
		// get distance to home
		if(home_distance > 10){ // 10m from home
			// we reset the angular offset to be a vector from home to the quad
			initial_simple_bearing = home_to_copter_bearing;
			//Serial.printf("ISB: %d\n", initial_simple_bearing);
		}
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
	#if HIL_MODE != HIL_MODE_DISABLED
		// update hil before dcm update
		gcs_update();
	#endif

	dcm.update_DCM_fast();
	omega = imu.get_gyro();
}

static void update_trig(void){
	Vector2f yawvector;
	Matrix3f temp 	= dcm.get_dcm_matrix();

	yawvector.x 	= temp.a.x; // sin
	yawvector.y 	= temp.b.x;	// cos
	yawvector.normalize();


	sin_pitch_y 	= -temp.c.x;						// level = 0
	cos_pitch_x 	= sqrt(1 - (temp.c.x * temp.c.x));	// level = 1

	sin_roll_y 		= temp.c.y / cos_pitch_x;			// level = 0
	cos_roll_x 		= temp.c.z / cos_pitch_x;			// level = 1

	sin_yaw_y 		= yawvector.x;						// 1y = north
	cos_yaw_x 		= yawvector.y;						// 0x = north

	//flat:
	// 0 ° = cos_yaw:  0.00, sin_yaw:  1.00,
	// 90° = cos_yaw:  1.00, sin_yaw:  0.00,
	// 180 = cos_yaw:  0.00, sin_yaw: -1.00,
	// 270 = cos_yaw: -1.00, sin_yaw:  0.00,
}

// updated at 10hz
static void update_altitude()
{
	altitude_sensor = BARO;
		//current_loc.alt = g_gps->altitude - gps_base_alt;
		//climb_rate = (g_gps->altitude - old_baro_alt) * 10;
		//old_baro_alt = g_gps->altitude;
		//baro_alt = g_gps->altitude;

	#if HIL_MODE == HIL_MODE_ATTITUDE
		// we are in the SIM, fake out the baro and Sonar
		int fake_relative_alt = g_gps->altitude - gps_base_alt;
		baro_alt			= fake_relative_alt;
		sonar_alt			= fake_relative_alt;

		baro_rate 			= (baro_alt - old_baro_alt) * 5; // 5hz
		old_baro_alt		= baro_alt;

	#else
		// This is real life

		// read in Actual Baro Altitude
		baro_alt 			= (baro_alt + read_barometer()) >> 1;

		// calc the vertical accel rate
		int temp			= (baro_alt - old_baro_alt) * 10;
		baro_rate 			= (temp + baro_rate) >> 1;
		old_baro_alt		= baro_alt;

		// sonar_alt is calculaed in a faster loop and filtered with a mode filter
	#endif


	if(g.sonar_enabled){
		// filter out offset
		float scale;

		// calc rate of change for Sonar
		#if HIL_MODE == HIL_MODE_ATTITUDE
			// we are in the SIM, fake outthe Sonar rate
			sonar_rate		= baro_rate;
		#else
			// This is real life
			// calc the vertical accel rate
			sonar_rate 		= (sonar_alt - old_sonar_alt) * 10;
			old_sonar_alt 	= sonar_alt;
		#endif

		if(baro_alt < 800){
			#if SONAR_TILT_CORRECTION == 1
				// correct alt for angle of the sonar
				float temp = cos_pitch_x * cos_roll_x;
				temp = max(temp, 0.707);
				sonar_alt = (float)sonar_alt * temp;
			#endif

			scale = (sonar_alt - 400) / 200;
			scale = constrain(scale, 0, 1);

			current_loc.alt = ((float)sonar_alt  * (1.0 - scale)) + ((float)baro_alt * scale) + home.alt;

			// solve for a blended climb_rate
			climb_rate 		= ((float)sonar_rate * (1.0 - scale)) + (float)baro_rate * scale;

		}else{
			// we must be higher than sonar (>800), don't get tricked by bad sonar reads
			current_loc.alt = baro_alt + home.alt; // home alt = 0
			// dont blend, go straight baro
			climb_rate 		= baro_rate;
		}

	}else{

		// NO Sonar case
		current_loc.alt = baro_alt + home.alt;
		climb_rate 		= baro_rate;
	}

	// manage bad data
	climb_rate = constrain(climb_rate, -300, 300);
}

static void
adjust_altitude()
{
	/*
	// old vert control
	if(g.rc_3.control_in <= 200){
		next_WP.alt -= 1;												// 1 meter per second
		next_WP.alt = max(next_WP.alt, (current_loc.alt - 500));		// don't go less than 4 meters below current location
		next_WP.alt = max(next_WP.alt, 100);							// don't go less than 1 meter
		//manual_boost = (g.rc_3.control_in == 0) ? -20 : 0;

	}else if (g.rc_3.control_in > 700){
		next_WP.alt += 1;												// 1 meter per second
		next_WP.alt = min(next_WP.alt, (current_loc.alt + 500));		// don't go more than 4 meters below current location
		//manual_boost = (g.rc_3.control_in == 800) ? 20 : 0;
	}*/

	if(g.rc_3.control_in <= 180 && !failsafe){
		// we remove 0 to 100 PWM from hover
		manual_boost = g.rc_3.control_in - 180;
		manual_boost = max(-120, manual_boost);
		g.throttle_cruise += g.pi_alt_hold.get_integrator();
		g.pi_alt_hold.reset_I();
		g.pi_throttle.reset_I();

	}else if  (g.rc_3.control_in >= 650 && !failsafe){
		// we add 0 to 100 PWM to hover
		manual_boost = g.rc_3.control_in - 650;
		g.throttle_cruise += g.pi_alt_hold.get_integrator();
		g.pi_alt_hold.reset_I();
		g.pi_throttle.reset_I();

	}else {
		manual_boost = 0;
	}
}

static void tuning(){
	tuning_value = (float)g.rc_6.control_in / 1000.0;

	switch(g.radio_tuning){

		/*
		case CH6_THRUST:
			g.rc_6.set_range(0,1000); 		// 0 to 1
			//Z_gain = tuning_value * 5;
			thrust	= tuning_value * .2;
			break;*/

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
			g.rc_6.set_range(40,300);		 // 0 to .3
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
			g.rc_6.set_range(0,1000);       // 0 to 1
			g.pi_throttle.kP(tuning_value);
			break;

		case CH6_TOP_BOTTOM_RATIO:
			g.rc_6.set_range(800,1000); 	// .8 to 1
			g.top_bottom_ratio = tuning_value;
			break;

		case CH6_RELAY:
			g.rc_6.set_range(0,1000);
		  	if (g.rc_6.control_in > 525) relay.on();
		  	if (g.rc_6.control_in < 475) relay.off();
			break;

		case CH6_TRAVERSE_SPEED:
			g.rc_6.set_range(0,1000);
			g.waypoint_speed_max = g.rc_6.control_in;
			break;

		case CH6_LOITER_P:
			g.rc_6.set_range(0,1000);
			g.pi_loiter_lat.kP(tuning_value);
			g.pi_loiter_lon.kP(tuning_value);
			break;

		case CH6_NAV_P:
			g.rc_6.set_range(0,6000);
			g.pi_nav_lat.kP(tuning_value);
			g.pi_nav_lon.kP(tuning_value);
			break;

		#if FRAME_CONFIG == HELI_FRAME
		case CH6_HELI_EXTERNAL_GYRO:
			g.rc_6.set_range(1000,2000);
			g.heli_ext_gyro_gain = tuning_value * 1000;
			break;
		#endif

		case CH6_THR_HOLD_KP:
			g.rc_6.set_range(0,1000);     // 0 to 1
			g.pi_alt_hold.kP(tuning_value);
			break;
	}
}

static void update_nav_wp()
{
	if(wp_control == LOITER_MODE){

		// calc a pitch to the target
		calc_location_error(&next_WP);

		// use error as the desired rate towards the target
		calc_loiter(long_error, lat_error);

		// rotate pitch and roll to the copter frame of reference
		calc_loiter_pitch_roll();

	}else if(wp_control == CIRCLE_MODE){

		// check if we have missed the WP
		int loiter_delta = (target_bearing - old_target_bearing)/100;

		// reset the old value
		old_target_bearing = target_bearing;

		// wrap values
		if (loiter_delta > 180) loiter_delta -= 360;
		if (loiter_delta < -180) loiter_delta += 360;

		// sum the angle around the WP
		loiter_sum += loiter_delta;

		// create a virtual waypoint that circles the next_WP
		// Count the degrees we have circulated the WP
		//int circle_angle = wrap_360(target_bearing + 3000 + 18000) / 100;

		circle_angle += (circle_rate * dTnav);
		//1° = 0.0174532925 radians

		// wrap
		if (circle_angle > 6.28318531)
			circle_angle -= 6.28318531;

		target_WP.lng = next_WP.lng + (g.loiter_radius * 100 * cos(1.57 - circle_angle) * scaleLongUp);
		target_WP.lat = next_WP.lat + (g.loiter_radius * 100 * sin(1.57 - circle_angle));

		// calc the lat and long error to the target
		calc_location_error(&target_WP);

		// use error as the desired rate towards the target
		// nav_lon, nav_lat is calculated
		calc_loiter(long_error, lat_error);

		//CIRCLE: angle:29, dist:0, lat:400, lon:242

		// rotate pitch and roll to the copter frame of reference
		calc_loiter_pitch_roll();
		//int angleTest = degrees(circle_angle);
		//int nroll = nav_roll;
		//int npitch = nav_pitch;
		//Serial.printf("CIRCLE: angle:%d, dist:%d, X:%d, Y:%d, P:%d, R:%d  \n", angleTest, (int)wp_distance , (int)long_error, (int)lat_error, npitch, nroll);

	}else if(wp_control == WP_MODE){
		// use error as the desired rate towards the target
		calc_nav_rate(g.waypoint_speed_max);
		// rotate pitch and roll to the copter frame of reference
		calc_nav_pitch_roll();

	}else if(wp_control == NO_NAV_MODE){
		nav_roll = 0;
		nav_pitch = 0;
	}
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



