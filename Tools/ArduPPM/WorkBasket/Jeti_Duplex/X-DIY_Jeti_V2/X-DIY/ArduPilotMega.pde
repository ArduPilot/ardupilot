// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
ArduPilotMega Version 1.0.3 Public Alpha
Authors:	Doug Weibel, Jose Julio, Jordi Munoz, Jason Short
Thanks to:	Chris Anderson, HappyKillMore, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi
Please contribute your ideas!


This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
*/

// Libraries
#include <FastSerial.h>
#include <JETI_Box.h>
#include <AP_Common.h>
#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <APM_ADC.h>		// ArduPilot Mega Analog to Digital Converter Library 
#include <AP_GPS.h>			// ArduPilot GPS library
#include <Wire.h>
#include <APM_BMP085.h> 	// ArduPilot Mega BMP085 Library 
#include <DataFlash.h>		// ArduPilot Mega Flash Memory Library
#include <APM_Compass.h>	// ArduPilot Mega Magnetometer Library

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>

// Configuration
#include "config.h"

// Local modules
#include "defines.h"

// Serial ports
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);		// FTDI/console
FastSerialPort1(Serial1);		// GPS port (except for GPS_PROTOCOL_IMU)
//FastSerialPort3(Serial3);		// Telemetry port (optional, Standard and ArduPilot protocols only)

// GPS selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA		GPS(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF		GPS(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX	GPS(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_IMU
AP_GPS_IMU		GPS(&Serial);	// note, console port
#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK		GPS(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_NONE		GPS(NULL);
#else
# error Must define GPS_PROTOCOL in your configuration file.
#endif

// The X-Plane GCS requires the IMU GPS configuration
#if (ENABLE_HIL == ENABLED) && (GPS_PROTOCOL != GPS_PROTOCOL_IMU)
# error Must select GPS_PROTOCOL_IMU when configuring for X-Plane
#endif

// GENERAL VARIABLE DECLARATIONS
// --------------------------------------------
byte 	control_mode		= MANUAL;
boolean failsafe			= false;	// did our throttle dip below the failsafe value?
boolean ch3_failsafe		= false;
byte 	crash_timer;
byte 	oldSwitchPosition;				// for remembering the control mode switch
boolean reverse_switch 		= 1;		// do we read the reversing switches after startup?

byte 	ground_start_count	= 6;		// have we achieved first lock and set Home?
int 	ground_start_avg;				// 5 samples to avg speed for ground start
boolean ground_start		= false;	// have we started on the ground?
char *comma = ",";

char* flight_mode_strings[] = {	
	"Manual",
	"Circle",
	"Stabilize",
	"",
	"",
	"FBW_A",
	"FBW_B",
	"",
	"",
	"",
	"Auto",
	"RTL",
	"Loiter",
	"Takeoff",
	"Land"};


/* Radio values
		Channel assignments	
			1	Ailerons (rudder if no ailerons)
			2	Elevator
			3	Throttle
			4	Rudder (if we have ailerons)
			5	Mode
			6 	TBD
			7	TBD
			8	TBD
*/

int radio_min[8];			// may be reset by init sequence
int radio_trim[8];			// may be reset by init sequence
int radio_max[8];			// may be reset by init sequence
int radio_in[8];			// current values from the transmitter - microseconds
int radio_out[8];			// Send to the PWM library
int servo_out[8];			// current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100

int elevon1_trim 	= 1500;
int elevon2_trim 	= 1500;
int ch1_temp 		= 1500;			// Used for elevon mixing
int ch2_temp 		= 1500;

int reverse_roll 	= 1;
int reverse_pitch 	= 1;
int reverse_rudder  = 1;
byte mix_mode 		= 0; // 0 = normal , 1 = elevons
int reverse_elevons = 1;
int reverse_ch1_elevon = 1;
int reverse_ch2_elevon = 1;
byte yaw_mode;

byte flight_mode_channel;
byte flight_modes[6];
byte auto_trim;

// for elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon


/*	PID Control variables
		Cases
		1	Aileron servo control	(rudder if no ailerons)
		2	Elevator servo control
		3	Rudder servo control 	(if we have ailerons)
		4	Roll set-point control
		5	Pitch set-point based on airspeed error control		
		6	Pitch set-point based on altitude error control		(if we do not have airspeed sensor)
		7	Throttle based on Energy Height (Total Energy) error control
		8	Throttle based on altitude error control
*/
		
float 		kp[8];
float 		ki[8];
float 		kd[8];
uint16_t 	integrator_max[8];			// PID Integrator Max  	deg * 100
float		integrator[8];				// PID Integrators		deg * 100
long		last_error[8];				// PID last error for derivative
float 		nav_gain_scaler	= 1;		// Gain scaling for headwind/tailwind


/*	Feed Forward gains
		Cases
		1	Bank angle to desired pitch	(Pitch Comp)
		2	Aileron Servo to Rudder Servo
		3	Pitch to Throttle
*/

float kff[3];


// GPS variables
// -------------
const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
float 	scaleLongUp;						// used to reverse longtitude scaling
float 	scaleLongDown;						// used to reverse longtitude scaling
boolean GPS_light			= false;		// status of the GPS light

// Location & Navigation 
// ---------------------
byte 	wp_radius			= 20;			// meters
long 	hold_course 		= -1;			// deg * 100 dir of plane
long	nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
long 	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
long 	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
int 	climb_rate;							// m/s * 100  - For future implementation of controlled ascent/descent by rate
byte	loiter_radius; // meters
float	x_track_gain;
int		x_track_angle;

long	head_max;
long	pitch_max;
long	pitch_min;
float	altitude_mix;

byte	command_must_index;					// current command memory location
byte	command_may_index;					// current command memory location
byte	command_must_ID;					// current command ID
byte	command_may_ID;						// current command ID
//byte	EEPROM_command						// 1 = from the list, 0 = generated


// Airspeed
// --------
int 	airspeed;							// m/s * 100
int		airspeed_cruise;					// m/s * 100 : target airspeed sensor value
float 	airspeed_ratio;						// used to scale airspeed
byte 	airspeed_fbw_min;					// m/s
byte 	airspeed_fbw_max;					// m/s

// Throttle Failsafe
// ------------------
byte 		throttle_failsafe_enabled;
int 		throttle_failsafe_value;
byte 		throttle_failsafe_action;
uint16_t 	log_bitmask;

// Location Errors
// ---------------
long 	bearing_error;						// deg * 100 : 0 to 36000 
long 	altitude_error;						// meters * 100 we are off in altitude
float 	airspeed_error;						// m/s * 100 
float	crosstrack_error;					// meters we are off trackline
long 	energy_error;						// energy state error (kinetic + potential) for altitude hold
long	airspeed_energy_error;				// kinetic portion of energy error

// Sensors 
// --------
float 	airpressure_raw;								// Airspeed Sensor - is a float to better handle filtering
int 	airpressure_offset;								// analog air pressure sensor while still
int 	airpressure;									// airspeed as a pressure value
float   battery_voltage		= LOW_VOLTAGE * 1.05;		// Battery Voltage of total battery, initialized above threshold for filter
float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cell 1, initialized above threshold for filter
float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1+2, initialized above threshold for filter
float 	battery_voltage3 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1+2+3, initialized above threshold for filter
float 	battery_voltage4 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1+2+3+4, initialized above threshold for filter

// From IMU
long	roll_sensor;						// degrees * 100
long	pitch_sensor;						// degrees * 100
long	yaw_sensor;							// degrees * 100

float 	roll;								// radians
float 	pitch;								// radians
float 	yaw;								// radians


// Magnetometer variables
int 	magnetom_x;							// Doug to do
int 	magnetom_y;			
int 	magnetom_z;			
float 	MAG_Heading;			

// Pressure Sensor variables
unsigned long abs_press;
unsigned long abs_press_filt;
unsigned long abs_press_gnd;
int 	ground_temperature;
int 	temp_unfilt; 
long 	ground_alt;			 				// Ground altitude from gps at startup in centimeters
long 	press_alt;			 				// Pressure altitude 

// flight mode specific
// --------------------
//boolean takeoff_complete	= false;		// Flag for using take-off controls
boolean land_complete		= false;
int landing_pitch;							// pitch for landing set by commands
int takeoff_pitch;			
int takeoff_altitude;		
int landing_distance;						// meters;

// Loiter management
// -----------------
long 	old_target_bearing;					// deg * 100
int		loiter_total; 						// deg : how many times to loiter * 360
int 	loiter_delta;						// deg : how far we just turned
int		loiter_sum;							// deg : how far we have turned around a waypoint
long 	loiter_time;						// millis : when we started LOITER mode
int 	loiter_time_max;					// millis : how long to stay in LOITER mode

// these are the values for navigation control functions
// ----------------------------------------------------
long 	nav_roll;							// deg * 100 : target roll angle
long 	nav_pitch;							// deg * 100 : target pitch angle
long	altitude_estimate;					// for smoothing GPS output

int 	throttle_min;						// 0-100 : target throttle output for average speed
int 	throttle_cruise;					// 0-100 : target throttle output for average speed
int 	throttle_max;						// 0-100 : target throttle output for average speed

// Waypoints
// ---------
long 	wp_distance;						// meters - distance between plane and next waypoint
long 	wp_totalDistance;					// meters - distance between old and next waypoint
byte 	wp_total;							// # of Commands total including way
byte 	wp_index;							// Current active command index
byte 	next_wp_index;						// Current active command index

// repeating event control
// -----------------------
byte 	event_id; 							// what to do - see defines
long 	event_timer; 						// when the event was asked for in ms
int 	event_delay; 						// how long to delay the next firing of event in millis
int 	event_repeat;						// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
int 	event_value; 						// per command value, such as PWM for servos
int 	event_undo_value;					// the value used to undo commands
byte 	repeat_forever;
byte 	undo_event;							// counter for timing the undo

// delay command
// --------------
int 	delay_timeout				= 0;	// used to delay commands
long 	delay_start					= 0;	// used to delay commands

// 3D Location vectors
// -------------------
struct Location home;						// home location
struct Location prev_WP;					// last waypoint
struct Location current_loc;				// current location
struct Location next_WP;					// next waypoint
struct Location tell_command;				// command for telemetry
struct Location next_command;				// command preloaded
long 	target_altitude;					// used for 
long 	offset_altitude;					// used for 
int 	hold_alt_above_home;
boolean	home_is_set					= false; // Flag for if we have gps lock and have set the home location


// IMU variables
// -------------

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
uint8_t sensors[6] 	= {1,2,0,4,5,6};				// For ArduPilot Mega Sensor Shield Hardware
int SENSOR_SIGN[]	= { 1, -1, -1,
					   -1,  1,  1,
					   -1, -1, -1};	

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X / Y/Z gyro][A / B/C or 0 order / 1st order / 2nd order constants]
// values may migrate to a Config file
float GTC[3][3]={{1665, 0, 0},
				 {1665, 0, 0},
				 {1665, 0, 0}};	

float AN[6]; 									// array that store the 6 ADC channels used by IMU
float AN_OFFSET[6]; 							// Array that store the Offset of the gyros and accelerometers
float G_Dt							= 0.02;		// Integration time for the gyros (DCM algorithm)
float Accel_Vector[3];					 		// Store the acceleration in a vector
float Accel_Vector_unfiltered[3];				// Store the acceleration in a vector
float Gyro_Vector[3];							// Store the gyros turn rate in a vector
float Omega_Vector[3];							// Corrected Gyro_Vector data
float Omega_P[3];								// Omega Proportional correction
float Omega_I[3];								// Omega Integrator
float Omega[3];
float errorRollPitch[3];
float errorYaw[3];
float errorCourse;
float COGX;								 		// Course overground X axis
float COGY							= 1; 		// Course overground Y axis

float DCM_Matrix[3][3] = {{1,0,0},
						  {0,1,0},
						  {0,0,1}};

float Update_Matrix[3][3] = {{0,1,2},
						 	 {3,4,5},
							 {6,7,8}};

float Temporary_Matrix[3][3];

// Performance monitoring
// ----------------------
long 	perf_mon_timer;
float 	imu_health; 							// Metric based on accel gain deweighting
int 	G_Dt_max;								// Max main loop cycle time in milliseconds
byte 	gyro_sat_count;
byte 	adc_constraints;
byte 	renorm_sqrt_count;
byte 	renorm_blowup_count;
int 	gps_fix_count;
byte	gcs_messages_sent;


// GCS
// ---
char GCS_buffer[53];

// System Timers
// --------------
unsigned long fast_loopTimer;				// Time in miliseconds of main control loop
unsigned long medium_loopTimer;				// Time in miliseconds of navigation control loop
byte medium_loopCounter = 0;				// Counters for branching from main control loop to slower loops
byte slow_loopCounter = 0;
byte superslow_loopCounter = 0;
unsigned long deltaMiliSeconds; 			// Delta Time in miliseconds
unsigned long dTnav;						// Delta Time in milliseconds for navigation computations
int mainLoop_count;
unsigned long elapsedTime;					// for doing custom events
unsigned int GPS_timer;			

//********************************************************************************************************************************
//********************************************************************************************************************************
//********************************************************************************************************************************
//********************************************************************************************************************************
//********************************************************************************************************************************

// Basic Initialization
//---------------------
void setup() {
	jeti_init();
	jeti_status("  ** X-DIY **");
	jeti_update;
	init_ardupilot();
}

void loop()
{
	// We want this to execute at 50Hz if possible
	// -------------------------------------------
	if (millis()-fast_loopTimer > 19) {
		deltaMiliSeconds 	= millis() - fast_loopTimer;
		G_Dt 				= (float)deltaMiliSeconds / 1000.f;
		fast_loopTimer		= millis();
		mainLoop_count++;

		// Execute the fast loop
		// ---------------------
		fast_loop();
		
		// Execute the medium loop 
		// -----------------------
		medium_loop();
		
		if (millis()- perf_mon_timer > 20000) {
			if (mainLoop_count != 0) {
				send_message(MSG_PERF_REPORT);
				if (log_bitmask & MASK_LOG_PM)
					Log_Write_Performance();
					
				resetPerfData();
			}
		}
	}
}

void fast_loop()
{
	// This is the fast loop - we want it to execute at 50Hz if possible
	// -----------------------------------------------------------------
	if (deltaMiliSeconds > G_Dt_max) 
		G_Dt_max = deltaMiliSeconds;
	
	// Read radio
	// ----------
	read_radio();

	// check for throttle failsafe condition
	// ------------------------------------
#if THROTTLE_FAILSAFE == 1
		set_failsafe(ch3_failsafe);
#endif
	

	// read in the plane's attitude
	// ----------------------------
#if GPS_PROTOCOL == GPS_PROTOCOL_IMU
		GPS.update();
		airspeed 		= (float)GPS.airspeed; 	//airspeed is * 100 coming in from Xplane for accuracy
		calc_airspeed_errors();
		
		if(digitalRead(SLIDE_SWITCH_PIN) == 0) {
			read_AHRS();
			roll_sensor 	= -roll_sensor;
			pitch_sensor 	= -pitch_sensor;
			//yaw_sensor      = -yaw_sensor;
		}else{
			roll_sensor 	= GPS.roll_sensor;
			pitch_sensor 	= GPS.pitch_sensor;
			yaw_sensor      = GPS.ground_course;
		}
		
#else
		// Read Airspeed
		// -------------
		#if AIRSPEED_SENSOR == 1
		read_airspeed();
		#endif
	
		read_AHRS();

		if (log_bitmask & MASK_LOG_ATTITUDE_FAST)
			Log_Write_Attitude((int)roll_sensor, (int)pitch_sensor, (uint16_t)yaw_sensor);

		if (log_bitmask & MASK_LOG_RAW)
			Log_Write_Raw();
#endif

	// altitude smoothing
	// ------------------
	if (control_mode != FLY_BY_WIRE_B)
		calc_altitude_error();
				
	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_current_flight_mode();

	// apply desired roll, pitch and yaw to the plane
	// ----------------------------------------------
	if (control_mode > MANUAL) 
		stabilize();

	// write out the servo PWM values
	// ------------------------------
	set_servos_4();

#if ENABLE_HIL
	output_HIL();
#endif

#if GCS_PROTOCOL == GCS_PROTOCOL_DEBUGTERMINAL
	readgcsinput();
#endif
}

void medium_loop()
{
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {
	
		// This case deals with the GPS
		//-------------------------------
		case 0:
			medium_loopCounter++;
			update_GPS();
			
			#if MAGNETOMETER == 1
				APM_Compass.Read();     // Read magnetometer
				APM_Compass.Calculate(roll,pitch);  // Calculate heading
			#endif
 
			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;

			if(GPS.new_data){
				dTnav = millis() - medium_loopTimer;
				medium_loopTimer = millis();
			}
			
			// calculate the plane's desired bearing
			// -------------------------------------			
			navigate();
			break;

		// unused? no, jeti gets updated :-)
		//------------------------------
		case 2:
			medium_loopCounter++;

			// perform next command
			// --------------------
			update_commands();
			jeti_update();
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;
			
			if ((log_bitmask & MASK_LOG_ATTITUDE_MED) && !(log_bitmask & MASK_LOG_ATTITUDE_FAST))
				Log_Write_Attitude((int)roll_sensor, (int)pitch_sensor, (uint16_t)yaw_sensor);

			if (log_bitmask & MASK_LOG_CTUN)
				Log_Write_Control_Tuning();

			if (log_bitmask & MASK_LOG_NTUN)
				Log_Write_Nav_Tuning();

			if (log_bitmask & MASK_LOG_GPS)
				Log_Write_GPS(GPS.time, current_loc.lat, current_loc.lng, GPS.altitude, current_loc.alt, (long) GPS.ground_speed, GPS.ground_course, GPS.fix, GPS.num_sats);

			send_message(MSG_ATTITUDE);		// Sends attitude data
			break;

			// This case controls the slow loop
		//---------------------------------
		case 4:
			medium_loopCounter=0;
			slow_loop();
			break;	
	}
}

void slow_loop()
{
	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
		case 0:
			slow_loopCounter++;
			superslow_loopCounter++;
			if(superslow_loopCounter >=15) {
				// keep track of what page is in use in the log
				// *** We need to come up with a better scheme to handle this...
				eeprom_write_word((uint16_t *) EE_LAST_LOG_PAGE, DataFlash.GetWritePage());
				superslow_loopCounter = 0;
			}
			break;
			
		case 1:
			slow_loopCounter++;

			// Read 3-position switch on radio
			// -------------------------------
			read_control_switch();

			// Read Control Surfaces/Mix switches
			// ----------------------------------
			if(reverse_switch){
				update_servo_switches();
			}

			// Read main battery voltage if hooked up - does not read the 5v from radio
			// ------------------------------------------------------------------------
			#if BATTERY_EVENT == 1
				read_battery();
			#endif
			
			// Read Baro pressure
			// ------------------
			read_airpressure();
			break;
			
		case 2:
			slow_loopCounter = 0;
			update_events();
			break;
	}
}


void update_GPS(void)
{
	GPS.update();
	update_GPS_light();
	
	if (GPS.new_data && GPS.fix) {
		GPS_timer = millis();
		send_message(MSG_LOCATION);

		// for performance
		// ---------------
		gps_fix_count++;
		
		if(ground_start_count > 1){
			ground_start_count--;
			ground_start_avg += GPS.ground_speed;
		
		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				SendDebugln("!! bad loc");
				ground_start_count = 5;
				
			} else {
				if(ENABLE_AIR_START == 1 && (ground_start_avg / 5) < SPEEDFILT) {
					startup_ground();

					if (log_bitmask & MASK_LOG_CMD)
						Log_Write_Startup(TYPE_GROUNDSTART_MSG);

					init_home();
				} else if (ENABLE_AIR_START == 0) {
					init_home();
				}

				ground_start_count = 0;
			}
		}  
	
		
		current_loc.lng = GPS.longitude;	// Lon * 10**7
		current_loc.lat = GPS.latitude;		// Lat * 10**7

#if GPS_PROTOCOL == GPS_PROTOCOL_IMU
		current_loc.alt = GPS.altitude;
#else
		current_loc.alt = ((1 - altitude_mix) * GPS.altitude) + (altitude_mix * press_alt);			// alt_MSL centimeters (meters * 100)
#endif
		
		COGX = cos(ToRad(GPS.ground_course/100.0));
		COGY = sin(ToRad(GPS.ground_course/100.0));
	}  
}
			
void update_current_flight_mode(void)
{
	if(control_mode == AUTO){
		crash_checker();				
		
		switch(command_must_ID){			
			case CMD_TAKEOFF:
				#if USE_MAGNETOMETER == ENABLED
					calc_nav_roll();
				#else
					nav_roll = 0;
				#endif
				float scaler;
				#if AIRSPEED_SENSOR == 1
					scaler = (float)airspeed / (float)airspeed_cruise;
					nav_pitch = scaler * takeoff_pitch * 50;
				#else
					scaler = (float)GPS.ground_speed / (float)airspeed_cruise;
					nav_pitch = scaler * takeoff_pitch * 50;
				#endif

				nav_pitch = constrain(nav_pitch, 0l, (long)takeoff_pitch);
				
				servo_out[CH_THROTTLE] = throttle_max; //TODO: Replace with THROTTLE_TAKEOFF or other method of controlling throttle
				// ******************************
				
				// override pitch_sensor to limit porpoising 
				// pitch_sensor 	= constrain(pitch_sensor, -6000, takeoff_pitch * 100);
				// throttle = passthrough
				break;
	
			case CMD_LAND:
				calc_nav_roll();
				
				#if AIRSPEED_SENSOR == 1
					calc_nav_pitch();
					calc_throttle();					
				#else
					calc_nav_pitch();				// calculate nav_pitch just to use for calc_throttle
					calc_throttle();				// throttle basedtest landing_pitch;		// pitch held constant
				#endif
				
				if (land_complete) {
					servo_out[CH_THROTTLE] = 0;
				}
				break;
				
			default:
				hold_course = -1;
				calc_nav_roll();
				calc_nav_pitch();
				calc_throttle();
				break;
		}
	}else{
		switch(control_mode){
			case RTL:
			case LOITER:
				hold_course = -1;
				crash_checker();
				calc_nav_roll();
				calc_nav_pitch();
				calc_throttle();
				break;
			
			case FLY_BY_WIRE_A:
				// fake Navigation output using sticks
				nav_roll	= ((radio_in[CH_ROLL]	- radio_trim[CH_ROLL])	* head_max * reverse_roll) / 350;
				nav_pitch 	= ((radio_in[CH_PITCH]  - radio_trim[CH_PITCH]) * 3500l * 	reverse_pitch) / 350;
		
				nav_roll = constrain(nav_roll, -head_max, head_max); 
				nav_pitch = constrain(nav_pitch, -3000, 3000);	// trying to give more pitch authority
				break;
				
			case FLY_BY_WIRE_B:
				// fake Navigation output using sticks
				// We use pitch_min because its magnitude is normally greater than pitch_max
				nav_roll	= ((radio_in[CH_ROLL]	- radio_trim[CH_ROLL])	* head_max * reverse_roll) / 350;
				altitude_error 	= ((radio_in[CH_PITCH]  - radio_trim[CH_PITCH]) * pitch_min * -reverse_pitch) / 350;
				nav_roll = constrain(nav_roll, -head_max, head_max); 
				
				#if AIRSPEED_SENSOR == 1
					//airspeed_error = ((AIRSPEED_FBW_MAX - AIRSPEED_FBW_MIN) * servo_out[CH_THROTTLE] / 100) + AIRSPEED_FBW_MIN;
					airspeed_error = ((int)(airspeed_fbw_max - airspeed_fbw_min) * servo_out[CH_THROTTLE]) + ((int)airspeed_fbw_min * 100);
					// Intermediate calculation - airspeed_error is just desired airspeed at this point
					airspeed_energy_error = (long)(((long)airspeed_error * (long)airspeed_error) - ((long)airspeed * (long)airspeed))/20000; //Changed 0.00005f * to / 20000 to avoid floating point calculation
					airspeed_error = (airspeed_error - airspeed);

				#endif

				calc_throttle();
				calc_nav_pitch();
				break;
			
			case STABILIZE:
				nav_roll 		= 0;
				nav_pitch 		= 0;
				// throttle is passthrough
				break;
			
			case CIRCLE:
				// we have no GPS installed and have lost radio contact
				// or we just want to fly around in a gentle circle w/o GPS
				// ----------------------------------------------------
				nav_roll = head_max / 3;
				nav_pitch 		= 0;
				
				if (failsafe == true){
					servo_out[CH_THROTTLE] = throttle_cruise;
				}
				break;
			
			case MANUAL:
				// servo_out is for Sim control only
				// ---------------------------------
				servo_out[CH_ROLL]  	= reverse_roll   * (radio_in[CH_ROLL]   - radio_trim[CH_ROLL])   * 9;
				servo_out[CH_PITCH] 	= reverse_pitch  * (radio_in[CH_PITCH]  - radio_trim[CH_PITCH])  * 9;
				servo_out[CH_RUDDER] 	= reverse_rudder * (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 9;
				break;
				//roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000 

		}
	}
}

void read_AHRS(void)
{
	// Get gyro and accel data and perform IMU calculations
	//-----------------------------------------------------
	Read_adc_raw();			// Get current values for IMU sensors
	Matrix_update(); 		// Integrate the DCM matrix
	Normalize();			// Normalize the DCM matrix
	Drift_correction();		// Perform drift correction
	Euler_angles();			// Calculate pitch, roll, yaw for stabilization and navigation
}




