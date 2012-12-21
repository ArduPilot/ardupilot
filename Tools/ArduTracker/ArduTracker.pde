/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
ArduPilotMega (unstable development version)
Authors:	Doug Weibel, Jose Julio, Jordi Munoz, Jason Short
Thanks to:	Chris Anderson, HappyKillMore, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi
Please contribute your ideas!


This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
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
#include <APM_BinComm.h>
#include <APM_RC.h> 		// ArduPilot Mega RC Library
#include <AP_GPS.h>			// ArduPilot GPS library
#include <Wire.h>
#include <DataFlash.h>		// ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>			// ArduPilot Mega Analog to Digital Converter Library
#include <APM_BMP085.h> 	// ArduPilot Mega BMP085 Library
#include <AP_Compass.h>		// ArduPilot Mega Magnetometer Library
#include <AP_Math.h>		// ArduPilot Mega Vector/Matrix math Library
#include <AP_IMU.h>			// ArduPilot Mega IMU Library
#include <AP_DCM.h>			// ArduPilot Mega DCM Library
#include <PID.h>			// PID library

#include <GCS_MAVLink.h>    // MAVLink GCS definitions

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "global_data.h"
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
FastSerialPort0(Serial);		// FTDI/console
FastSerialPort1(Serial1);		// GPS port
FastSerialPort3(Serial3);		// Telemetry port

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

#if HIL_MODE == HIL_MODE_NONE

// real sensors
AP_ADC_ADS7844			adc;
APM_BMP085_Class		pitot; //TODO: 'pitot' is not an appropriate name for a static pressure sensor
AP_Compass_HMC5843		compass;

// real GPS selection
#if GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA		gps(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF		gps(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX	gps(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK		gps(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19		gps(&Serial1);
#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_NONE		gps(NULL);
#else
 #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL				adc;
APM_BMP085_HIL_Class	pitot;
AP_Compass_HIL	compass;
AP_GPS_HIL				gps(NULL);

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_DCM_HIL dcm;
AP_GPS_HIL				gps(NULL);

#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

#if HIL_MODE != HIL_MODE_DISABLED

#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK
	HIL_MAVLINK hil;
#elif HIL_PROTOCOL == HIL_PROTOCOL_XPLANE
	HIL_XPLANE hil;
#endif // HIL PROTOCOL

#endif

#if HIL_MODE != HIL_MODE_ATTITUDE
AP_IMU imu(&adc,getAddress(PARAM_IMU_OFFSET_0));
AP_DCM 	dcm(&imu, &gps, &compass);
#endif

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
#if	  GCS_PROTOCOL == GCS_PROTOCOL_STANDARD
// create an instance of the standard GCS.
BinComm::MessageHandler GCS_MessageHandlers[] = {
	{BinComm::MSG_ANY, receive_message, NULL},
	{BinComm::MSG_NULL,	NULL, NULL}
};
GCS_STANDARD		gcs(GCS_MessageHandlers);

#elif GCS_PROTOCOL == GCS_PROTOCOL_LEGACY
GCS_LEGACY			gcs;
#elif GCS_PROTOCOL == GCS_PROTOCOL_DEBUGTERMINAL
GCS_DEBUGTERMINAL	gcs;
#elif GCS_PROTOCOL == GCS_PROTOCOL_XPLANE
GCS_XPLANE			gcs; 	// Should become a HIL
#elif GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
GCS_MAVLINK			gcs;
#else
// If we are not using a GCS, we need a stub that does nothing.
GCS_Class			gcs;
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

byte 	control_mode		= MANUAL;
boolean failsafe			= false;	// did our throttle dip below the failsafe value?
boolean ch3_failsafe		= false;
byte 	crash_timer;
byte 	oldSwitchPosition;				// for remembering the control mode switch
boolean reverse_switch 		= 1;		// do we read the reversing switches after startup?

byte 	ground_start_count	= 6;		// have we achieved first lock and set Home?
int 	ground_start_avg;				// 5 samples to avg speed for ground start
boolean ground_start		= false;	// have we started on the ground?
const char *comma = ",";

const char* flight_mode_strings[] = {
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

uint16_t  radio_in[8];			// current values from the transmitter - microseconds
uint16_t  radio_out[8];			// Send to the PWM library
int16_t  servo_out[8];			// current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100

uint16_t  elevon1_trim 	= 1500; // TODO: handle in EEProm
uint16_t  elevon2_trim 	= 1500;
uint16_t  ch1_temp      = 1500;		// Used for elevon mixing
uint16_t  ch2_temp 	= 1500;

int reverse_roll 	= 1;
int reverse_pitch 	= 1;
int reverse_rudder  = 1;
byte mix_mode 		= 0; // 0 = normal , 1 = elevons
int reverse_elevons = 1;
int reverse_ch1_elevon = 1;
int reverse_ch2_elevon = 1;

// for elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon


float 		nav_gain_scaler	= 1;		// Gain scaling for headwind/tailwind TODO: why does this variable need to be initialized to 1?

// PID controllers
PID	pidServoRoll(getAddress(PARAM_RLL2SRV_P),PID::STORE_EEPROM_FLOAT);
PID pidServoPitch(getAddress(PARAM_PTCH2SRV_P),PID::STORE_EEPROM_FLOAT);
PID	pidServoRudder(getAddress(PARAM_YW2SRV_P),PID::STORE_EEPROM_FLOAT);
PID	pidNavRoll(getAddress(PARAM_HDNG2RLL_P),PID::STORE_EEPROM_FLOAT);
PID pidNavPitchAirspeed(getAddress(PARAM_ARSPD2PTCH_P),PID::STORE_EEPROM_FLOAT);
PID pidNavPitchAltitude(getAddress(PARAM_ALT2PTCH_P),PID::STORE_EEPROM_FLOAT);
PID pidTeThrottle(getAddress(PARAM_ENRGY2THR_P),PID::STORE_EEPROM_FLOAT);
PID pidAltitudeThrottle(getAddress(PARAM_ALT2THR_P),PID::STORE_EEPROM_FLOAT);

PID *pid_index[] = {
	&pidServoRoll,
	&pidServoPitch,
	&pidServoRudder,
	&pidNavRoll,
	&pidNavPitchAirspeed,
	&pidNavPitchAltitude,
	&pidTeThrottle,
	&pidAltitudeThrottle
};

// GPS variables
// -------------
const 	float t7			= 10000000.0;	// used to scale values for EEPROM and flash memory storage
float 	scaleLongUp;						// used to reverse longtitude scaling
float 	scaleLongDown;						// used to reverse longtitude scaling
boolean GPS_light			= false;		// status of the GPS light

// Location & Navigation
// ---------------------
const   float radius_of_earth =  6378100;   // meters
const   float gravity =  9.81;   // meters/ sec^2
long 	hold_course 		= -1;			// deg * 100 dir of plane
long	nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
long 	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
long 	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
int 	climb_rate;							// m/s * 100  - For future implementation of controlled ascent/descent by rate


byte	command_must_index;					// current command memory location
byte	command_may_index;					// current command memory location
byte	command_must_ID;					// current command ID
byte	command_may_ID;						// current command ID
//byte	EEPROM_command						// 1 = from the list, 0 = generated


// Airspeed
// --------
int 	airspeed;							// m/s * 100
int		airspeed_nudge = 0;					// m/s * 100 : additional airspeed based on throttle stick position in top 1/2 of range
float 	airspeed_error;						// m/s * 100
long 	energy_error;						// energy state error (kinetic + potential) for altitude hold
long	airspeed_energy_error;				// kinetic portion of energy error

// Location Errors
// ---------------
long 	bearing_error;						// deg * 100 : 0 to 36000
long 	altitude_error;						// meters * 100 we are off in altitude
float	crosstrack_error;					// meters we are off trackline

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
boolean takeoff_complete	= true;			// Flag for using gps ground course instead of IMU yaw.  Set false when takeoff command processes.
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

int 	throttle_nudge = 0;					// 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel

// Waypoints
// ---------
long 	wp_distance;						// meters - distance between plane and next waypoint
long 	wp_totalDistance;					// meters - distance between old and next waypoint
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
boolean	home_is_set					= false; // Flag for if we have gps lock and have set the home location

// patch antenna variables
struct Location trackVehicle_loc;				// vehicle location to track with antenna 

// IMU variables
// -------------
float G_Dt							= 0.02;		// Integration time for the gyros (DCM algorithm)

float COGX;								 		// Course overground X axis
float COGY							= 1; 		// Course overground Y axis

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
char	display_PID = -1;					// Flag used by DebugTerminal to indicate that the next PID calculation with this index should be displayed

// System Timers
// --------------
unsigned long fast_loopTimer;				// Time in miliseconds of main control loop
unsigned long fast_loopTimeStamp = 0;			// Time Stamp when fast loop was complete
unsigned long medium_loopTimer;				// Time in miliseconds of navigation control loop
byte medium_loopCounter = 0;				// Counters for branching from main control loop to slower loops
byte slow_loopCounter = 0;
byte superslow_loopCounter = 0;
unsigned long deltaMiliSeconds; 			// Delta Time in miliseconds
unsigned long dTnav;						// Delta Time in milliseconds for navigation computations
int mainLoop_count;
unsigned long elapsedTime;					// for doing custom events
unsigned long GPS_timer;
float load;			                        // % MCU cycles used

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {

	init_ardupilot();
}

void loop()
{
	// We want this to execute at 50Hz if possible
	// -------------------------------------------
	if (millis()-fast_loopTimer > 19) {
		deltaMiliSeconds 	= millis() - fast_loopTimer;
        load				= float(fast_loopTimeStamp - fast_loopTimer)/deltaMiliSeconds;
		G_Dt				= (float)deltaMiliSeconds / 1000.f;
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
				gcs.send_message(MSG_PERF_REPORT);
				if (get(PARAM_LOG_BITMASK) & MASK_LOG_PM)
					Log_Write_Performance();

				resetPerfData();
			}
		}
        fast_loopTimeStamp = millis();
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

	// check for throtle failsafe condition
	// ------------------------------------
	//if (get(PARAM_THR_FAILSAFE))
		//set_failsafe(ch3_failsafe);

		// Read Airspeed
		// -------------
# if AIRSPEED_SENSOR == 1 && HIL_MODE != HIL_MODE_ATTITUDE
		//read_airspeed();
# endif

		//dcm.update_DCM(G_Dt);

# if HIL_MODE == HIL_MODE_DISABLED
		//if (get(PARAM_LOG_BITMASK) & MASK_LOG_ATTITUDE_FAST)
			//Log_Write_Attitude((int)dcm.roll_sensor, (int)dcm.pitch_sensor, (uint16_t)dcm.yaw_sensor);

		//if (get(PARAM_LOG_BITMASK) & MASK_LOG_RAW)
			//Log_Write_Raw();
#endif // HIL_MODE

	// altitude smoothing
	// ------------------
	//if (control_mode != FLY_BY_WIRE_B)
		//calc_altitude_error();

    // inertial navigation
    // ------------------
#if INERTIAL_NAVIGATION == ENABLED
	// TODO: implement inertial nav function
    //inertialNavigation();
#endif

	// custom code/exceptions for flight modes
	// ---------------------------------------
	//update_current_flight_mode();

	// apply desired roll, pitch and yaw to the plane
	// ----------------------------------------------
	//if (control_mode > MANUAL)
		//stabilize();

	// write out the servo PWM values
	// ------------------------------
	set_servos_4();


	// XXX is it appropriate to be doing the comms below on the fast loop?

#if HIL_MODE != HIL_MODE_DISABLED
	// kick the HIL to process incoming sensor packets
	hil.update();
	// send out hil data
	hil.send_message(MSG_SERVO_OUT);
	//hil.send_message(MSG_ATTITUDE);
	//hil.send_message(MSG_LOCATION);
	//hil.send_message(MSG_AIRSPEED);
#endif

	// kick the GCS to process uplink data
	gcs.update();
#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
	gcs.data_stream_send(45,1000);
#endif
	// XXX this should be absorbed into the above,
	// or be a "GCS fast loop" interface
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

			#if HIL_MODE != HIL_MODE_ATTITUDE && MAGNETOMETER == 1
				//compass.read();     // Read magnetometer
				//compass.calculate(dcm.roll,dcm.pitch);  // Calculate heading
			#endif

			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
			medium_loopCounter++;


			if(gps.new_data){
				dTnav = millis() - medium_loopTimer;
				medium_loopTimer = millis();
			}

			// calculate the plane's desired bearing
			// -------------------------------------
			//navigate();
			break;

		// command processing
		//------------------------------
		case 2:
			medium_loopCounter++;

			// perform next command
			// --------------------
			//update_commands();
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;

			//if ((get(PARAM_LOG_BITMASK) & MASK_LOG_ATTITUDE_MED) && !(get(PARAM_LOG_BITMASK) & MASK_LOG_ATTITUDE_FAST))
				//Log_Write_Attitude((int)dcm.roll_sensor, (int)dcm.pitch_sensor, (uint16_t)dcm.yaw_sensor);

#if HIL_MODE != HIL_MODE_ATTITUDE
			//if (get(PARAM_LOG_BITMASK) & MASK_LOG_CTUN)
				//Log_Write_Control_Tuning();
#endif

			//if (get(PARAM_LOG_BITMASK) & MASK_LOG_NTUN)
				//Log_Write_Nav_Tuning();

			//if (get(PARAM_LOG_BITMASK) & MASK_LOG_GPS)
				//Log_Write_GPS(gps.time, current_loc.lat, current_loc.lng, gps.altitude, current_loc.alt, (long) gps.ground_speed, gps.ground_course, gps.fix, gps.num_sats);

// XXX this should be a "GCS medium loop" interface
#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
			gcs.data_stream_send(5,45);
			// send all requested output streams with rates requested
			// between 5 and 45 Hz
#else
			//gcs.send_message(MSG_ATTITUDE);		// Sends attitude data
#endif
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
			//if(superslow_loopCounter >=15) {
				// keep track of what page is in use in the log
				// *** We need to come up with a better scheme to handle this...
				//eeprom_write_word((uint16_t *) EE_LAST_LOG_PAGE, DataFlash.GetWritePage());
				//superslow_loopCounter = 0;
			//}
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

#if HIL_MODE != HIL_MODE_ATTITUDE
			// Read Baro pressure
			// ------------------
			//read_airpressure();
#endif

			break;

		case 2:
			slow_loopCounter = 0;
			//update_events();

// XXX this should be a "GCS slow loop" interface
#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
			gcs.data_stream_send(1,5);
			// send all requested output streams with rates requested
			// between 1 and 5 Hz
#else
			//gcs.send_message(MSG_LOCATION);
#endif
			// send a heartbeat
			gcs.send_message(MSG_HEARTBEAT); // XXX This is running at 3 1/3 Hz
			//but should be at 1 Hz, new loop timer?
			// display load
			gcs.send_message(MSG_CPU_LOAD, load*100);
			break;
	}
}


void update_GPS(void)
{
	if(gps.status() == 0)
	{
		gps.init(); // reinitialize dead connections
		return; // let it warm up while other stuff is running
	}
	gps.update();
	update_GPS_light();

	if (gps.new_data && gps.fix) {
		GPS_timer = millis();
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
			ground_start_avg += gps.ground_speed;

		} else if (ground_start_count == 1) {
			// We countdown N number of good GPS fixes
			// so that the altitude is more accurate
			// -------------------------------------
			if (current_loc.lat == 0) {
				SendDebugln("!! bad loc");
				ground_start_count = 5;

			} else {
				if(ENABLE_AIR_START == 1 && (ground_start_avg / 5) < SPEEDFILT){
					startup_ground();

					if (get(PARAM_LOG_BITMASK) & MASK_LOG_CMD)
						Log_Write_Startup(TYPE_GROUNDSTART_MSG);

					init_home();
				} else if (ENABLE_AIR_START == 0) {
					init_home();
				}

				ground_start_count = 0;
			}
		}


		current_loc.lng = gps.longitude;	// Lon * 10**7
		current_loc.lat = gps.latitude;		// Lat * 10**7

// XXX this is bogus; should just force get(PARAM_ALT_MIX) to zero for GPS_PROTOCOL_IMU
#if HIL_MODE == HIL_MODE_ATTITUDE
		current_loc.alt = gps.altitude;
#else
		current_loc.alt = ((1 - get(PARAM_ALT_MIX)) * gps.altitude) + (get(PARAM_ALT_MIX) * press_alt);			// alt_MSL centimeters (meters * 100)
#endif

		// Calculate new climb rate
		add_altitude_data(millis()/100, gps.altitude/10);

		COGX = cos(ToRad(gps.ground_course/100.0));
		COGY = sin(ToRad(gps.ground_course/100.0));
	}
}

void update_current_flight_mode(void)
{
	if(control_mode == AUTO){
		crash_checker();

		switch(command_must_ID){
			case CMD_TAKEOFF:
				if (hold_course > -1) {
					calc_nav_roll();
				} else {
					nav_roll = 0;
				}

#if AIRSPEED_SENSOR == ENABLED
					calc_nav_pitch();
					if (nav_pitch < (long)takeoff_pitch) nav_pitch = (long)takeoff_pitch;
#else
					nav_pitch = (long)((float)gps.ground_speed / (float)get(PARAM_TRIM_AIRSPEED) * (float)takeoff_pitch * 0.5);
					nav_pitch = constrain(nav_pitch, 500l, (long)takeoff_pitch);
#endif


				servo_out[CH_THROTTLE] = get(PARAM_THR_MAX); //TODO: Replace with THROTTLE_TAKEOFF or other method of controlling throttle
														//  What is the case for doing something else?  Why wouldn't you want max throttle for TO?
				// ******************************

				break;

			case CMD_LAND:
				calc_nav_roll();

#if AIRSPEED_SENSOR == ENABLED
					calc_nav_pitch();
					calc_throttle();
#else
					calc_nav_pitch();				// calculate nav_pitch just to use for calc_throttle
					calc_throttle();				// throttle based on altitude error
					nav_pitch = landing_pitch;		// pitch held constant
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
				nav_roll	= ((radio_in[CH_ROLL]	- radio_trim(CH_ROLL))	*
						get(PARAM_LIM_ROLL) * reverse_roll) / 350;
				nav_pitch 	= ((radio_in[CH_PITCH]  - radio_trim(CH_PITCH)) *
						3500l * 	reverse_pitch) / 350;
				nav_roll = constrain(nav_roll, -get(PARAM_LIM_ROLL), get(PARAM_LIM_ROLL));
				nav_pitch = constrain(nav_pitch, -3000, 3000);	// trying to give more pitch authority
				break;

			case FLY_BY_WIRE_B:
				// fake Navigation output using sticks
				// We use get(PARAM_PITCH_MIN) because its magnitude is
				// normally greater than get(PARAM_get(PARAM_PITCH_MAX))
				nav_roll	= ((radio_in[CH_ROLL]	- radio_trim(CH_ROLL))
						* get(PARAM_LIM_ROLL) * reverse_roll) / 350;
				altitude_error 	= ((radio_in[CH_PITCH]  - radio_trim(CH_PITCH))
						* get(PARAM_LIM_PITCH_MIN) * -reverse_pitch) / 350;
				nav_roll = constrain(nav_roll, -get(PARAM_LIM_ROLL), get(PARAM_LIM_ROLL));

#if AIRSPEED_SENSOR == ENABLED
					airspeed_error = ((int)(get(PARAM_ARSPD_FBW_MAX) -
							get(PARAM_ARSPD_FBW_MIN)) *
							servo_out[CH_THROTTLE]) +
							((int)get(PARAM_ARSPD_FBW_MIN) * 100);
					// Intermediate calculation - airspeed_error is just desired airspeed at this point
					airspeed_energy_error = (long)(((long)airspeed_error *
								(long)airspeed_error) -
							((long)airspeed * (long)airspeed))/20000;
					//Changed 0.00005f * to / 20000 to avoid floating point calculation
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
				nav_roll = get(PARAM_LIM_ROLL) / 3;
				nav_pitch 		= 0;

				if (failsafe == true){
					servo_out[CH_THROTTLE] = get(PARAM_TRIM_THROTTLE);
				}
				break;

			case MANUAL:
				// servo_out is for Sim control only
				// ---------------------------------
				servo_out[CH_ROLL]  	= reverse_roll   * (radio_in[CH_ROLL]   - radio_trim(CH_ROLL))   * 9;
				servo_out[CH_PITCH] 	= reverse_pitch  * (radio_in[CH_PITCH]  - radio_trim(CH_PITCH))  * 9;
				servo_out[CH_RUDDER] 	= reverse_rudder * (radio_in[CH_RUDDER] - radio_trim(CH_RUDDER)) * 9;
				break;
				//roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000

		}
	}
}
