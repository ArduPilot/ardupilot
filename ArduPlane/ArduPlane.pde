/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduPilotMega 2.2.0"
/*
Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short
Thanks to:  Chris Anderson, HappyKillMore, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi
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
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_GPS.h>         // ArduPilot GPS library
#include <Wire.h>			// Arduino I2C lib
#include <DataFlash.h>      // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <APM_BMP085.h>     // ArduPilot Mega BMP085 Library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
#include <AP_DCM.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
//#include <AP_RangeFinder.h>	// Range finder library

#define MAVLINK_COMM_NUM_BUFFERS 2
#include <GCS_MAVLink.h>    // MAVLink GCS definitions

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
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
Parameters      g;


////////////////////////////////////////////////////////////////////////////////
// prototypes
void update_events(void);


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
GPS         *g_gps;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
AP_ADC_ADS7844          adc;
APM_BMP085_Class        barometer;
// MAG PROTOCOL
#if   MAG_PROTOCOL == MAG_PROTOCOL_5843
AP_Compass_HMC5843      compass(Parameters::k_param_compass);

#elif MAG_PROTOCOL == MAG_PROTOCOL_5883L
AP_Compass_HMC5883L      compass(Parameters::k_param_compass);

#else
 #error Unrecognised MAG_PROTOCOL setting.
#endif // MAG PROTOCOL

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
AP_DCM_HIL              dcm;
AP_GPS_HIL              g_gps_driver(NULL);
AP_Compass_HIL          compass; // never used
AP_IMU_Shim             imu; // never used

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

//AP_RangeFinder_MaxsonarXL sonar;

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

byte    control_mode        = MANUAL;
byte    oldSwitchPosition;              // for remembering the control mode switch

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
			1   Ailerons (rudder if no ailerons)
			2   Elevator
			3   Throttle
			4   Rudder (if we have ailerons)
			5   Mode
			6   TBD
			7   TBD
			8   TBD
*/

// Failsafe
// --------
int 	failsafe;					// track which type of failsafe is being processed
bool 	ch3_failsafe;
byte    crash_timer;

// Radio
// -----
uint16_t elevon1_trim  = 1500; 	// TODO: handle in EEProm
uint16_t elevon2_trim  = 1500;
uint16_t ch1_temp      = 1500;     // Used for elevon mixing
uint16_t ch2_temp  	= 1500;
int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
bool rc_override_active = false;
uint32_t rc_override_fs_timer = 0;
uint32_t ch3_failsafe_timer = 0;

bool 	reverse_roll;
bool 	reverse_pitch;
bool 	reverse_rudder;
byte 	mix_mode; 			// 0 = normal , 1 = elevons

// TODO: switch these reverses to true/false, after they are handled by RC_Channel
int 	reverse_elevons = 1;
int 	reverse_ch1_elevon = 1;
int 	reverse_ch2_elevon = 1;
// for elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon

// LED output
// ----------
bool GPS_light;							// status of the GPS light

// GPS variables
// -------------
const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
float 	scaleLongUp			= 1;			// used to reverse longtitude scaling
float 	scaleLongDown 		= 1;			// used to reverse longtitude scaling
byte 	ground_start_count	= 5;			// have we achieved first lock and set Home?
int     ground_start_avg;					// 5 samples to avg speed for ground start
bool ground_start;    					// have we started on the ground?
bool	GPS_enabled 	= false;			// used to quit "looking" for gps with auto-detect if none present

// Location & Navigation
// ---------------------
const	float radius_of_earth 	= 6378100;	// meters
const	float gravity 			= 9.81;		// meters/ sec^2
long	nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
long	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
long	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
int		climb_rate;							// m/s * 100  - For future implementation of controlled ascent/descent by rate
float	nav_gain_scaler 		= 1;		// Gain scaling for headwind/tailwind TODO: why does this variable need to be initialized to 1?
long    hold_course       	 	= -1;		// deg * 100 dir of plane

byte	command_must_index;					// current command memory location
byte	command_may_index;					// current command memory location
byte	command_must_ID;					// current command ID
byte	command_may_ID;						// current command ID

// Airspeed
// --------
int		airspeed;							// m/s * 100
int     airspeed_nudge;  					// m/s * 100 : additional airspeed based on throttle stick position in top 1/2 of range
float   airspeed_error;						// m/s * 100
long    energy_error;                       // energy state error (kinetic + potential) for altitude hold
long    airspeed_energy_error;              // kinetic portion of energy error
bool airspeed_enabled = false;

// Location Errors
// ---------------
long	bearing_error;						// deg * 100 : 0 to 36000
long	altitude_error;						// meters * 100 we are off in altitude
float	crosstrack_error;					// meters we are off trackline

// Battery Sensors
// ---------------
float	battery_voltage		= LOW_VOLTAGE * 1.05;		// Battery Voltage of total battery, initialized above threshold for filter
float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cell 1, initialized above threshold for filter
float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2, initialized above threshold for filter
float 	battery_voltage3 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3, initialized above threshold for filter
float 	battery_voltage4 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3 + 4, initialized above threshold for filter

float	current_amps;
float	current_total;

// Airspeed Sensors
// ----------------
float   airspeed_raw;                       // Airspeed Sensor - is a float to better handle filtering
int     airspeed_offset;					// analog air pressure sensor while still
int     airspeed_pressure;					// airspeed as a pressure value

// Barometer Sensor variables
// --------------------------
unsigned long 	abs_pressure;

// Altitude Sensor variables
// ----------------------
//byte 	altitude_sensor = BARO;				// used to know which sensor is active, BARO or SONAR

// flight mode specific
// --------------------
bool takeoff_complete    = true;         // Flag for using gps ground course instead of IMU yaw.  Set false when takeoff command processes.
bool	land_complete;
long	takeoff_altitude;
int			landing_distance;					// meters;
int			landing_pitch;						// pitch for landing set by commands
int			takeoff_pitch;

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
long	nav_roll;							// deg * 100 : target roll angle
long	nav_pitch;							// deg * 100 : target pitch angle
int     throttle_nudge = 0;                 // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel

// Waypoints
// ---------
long	wp_distance;						// meters - distance between plane and next waypoint
long	wp_totalDistance;					// meters - distance between old and next waypoint
byte	next_wp_index;						// Current active command index

// repeating event control
// -----------------------
byte 		event_id; 							// what to do - see defines
long 		event_timer; 						// when the event was asked for in ms
uint16_t 	event_delay; 						// how long to delay the next firing of event in millis
int 		event_repeat = 0;					// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
int 		event_value; 						// per command value, such as PWM for servos
int 		event_undo_value;					// the value used to cycle events (alternate value to event_value)

// delay command
// --------------
long 	condition_value;						// used in condition commands (eg delay, change alt, etc.)
long 	condition_start;
int 	condition_rate;

// 3D Location vectors
// -------------------
struct 	Location home;						// home location
struct 	Location prev_WP;					// last waypoint
struct 	Location current_loc;				// current location
struct 	Location next_WP;					// next waypoint
struct 	Location next_command;				// command preloaded
long 	target_altitude;					// used for altitude management between waypoints
long 	offset_altitude;					// used for altitude management between waypoints
bool	home_is_set; 						// Flag for if we have g_gps lock and have set the home location


// IMU variables
// -------------
float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)


// Performance monitoring
// ----------------------
long 	perf_mon_timer;					// Metric based on accel gain deweighting
int 	G_Dt_max;							// Max main loop cycle time in milliseconds
int 	gps_fix_count;
byte	gcs_messages_sent;


// GCS
// ---
char GCS_buffer[53];
char display_PID = -1;						// Flag used by DebugTerminal to indicate that the next PID calculation with this index should be displayed

// System Timers
// --------------
unsigned long 	fast_loopTimer;				// Time in miliseconds of main control loop
unsigned long 	fast_loopTimeStamp;			// Time Stamp when fast loop was complete
uint8_t 		delta_ms_fast_loop; 		// Delta Time in miliseconds
int 			mainLoop_count;

unsigned long 	medium_loopTimer;			// Time in miliseconds of medium loop
byte 			medium_loopCounter;			// Counters for branching from main control loop to slower loops
uint8_t			delta_ms_medium_loop;

byte 			slow_loopCounter;
byte 			superslow_loopCounter;
byte			counter_one_herz;

unsigned long 	nav_loopTimer;				// used to track the elapsed ime for GPS nav

unsigned long 	dTnav;						// Delta Time in milliseconds for navigation computations
unsigned long 	elapsedTime;				// for doing custom events
float 			load;						// % MCU cycles used

//Camera tracking and stabilisation stuff
// --------------------------------------
byte			camera_mode	= 1;			//0 is do nothing, 1 is stabilize, 2 is track target
byte			gimbal_mode	= 0;			// 0 - pitch & roll, 1 - pitch and yaw (pan & tilt), 2 - pitch, roll and yaw (to be added)
struct			Location camera_target;			//point of iterest for the camera to track
Vector3<float>	target_vector(0,0,1);				//x, y, z to target before rotating to planes axis, values are in meters
float cam_pitch;
float cam_roll;
float cam_tilt;
float cam_pan;

struct			Location GPS_mark;	// GPS POI for position based triggering
int picture_time	= 0;			// waypoint trigger variable
int thr_pic		= 0;			// timer variable for throttle_pic
int camtrig		= 83;			// PK6 chosen as it not near anything so safer for soldering
pinMode(camtrig, OUTPUT);			// these are free pins PE3(5), PH3(15), PH6(18), PB4(23), PB5(24), PL1(36), PL3(38), PA6(72), PA7(71), PK0(89), PK1(88), PK2(87), PK3(86), PK4(83), PK5(84), PK6(83), PK7(82)


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
				gcs.send_message(MSG_PERF_REPORT);
				if (g.log_bitmask & MASK_LOG_PM)
					Log_Write_Performance();

				resetPerfData();
			}
		}

		fast_loopTimeStamp = millis();
	}
}

// Main loop 50Hz
void fast_loop()
{
	// This is the fast loop - we want it to execute at 50Hz if possible
	// -----------------------------------------------------------------
	if (delta_ms_fast_loop > G_Dt_max)
		G_Dt_max = delta_ms_fast_loop;

	// Read radio
	// ----------
	read_radio();

	// check for loss of control signal failsafe condition
	// ------------------------------------
	check_short_failsafe();
	
		// Read Airspeed
		// -------------
	if (airspeed_enabled == true && HIL_MODE != HIL_MODE_ATTITUDE) {
		read_airspeed();
	} else if (airspeed_enabled == true && HIL_MODE == HIL_MODE_ATTITUDE) {
		calc_airspeed_errors();
	}

	#if HIL_MODE == HIL_MODE_SENSORS
		// update hil before dcm update
		hil.update();
	#endif

	dcm.update_DCM(G_Dt);

	// uses the yaw from the DCM to give more accurate turns
	calc_bearing_error();

	# if HIL_MODE == HIL_MODE_DISABLED
		if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST)
			Log_Write_Attitude((int)dcm.roll_sensor, (int)dcm.pitch_sensor, (uint16_t)dcm.yaw_sensor);

		if (g.log_bitmask & MASK_LOG_RAW)
			Log_Write_Raw();
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
	if (control_mode > MANUAL)
		stabilize();

	// write out the servo PWM values
	// ------------------------------
	set_servos_4();


	// XXX is it appropriate to be doing the comms below on the fast loop?

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
	// XXX this should be absorbed into the above,
	// or be a "GCS fast loop" interface
}

void medium_loop()
{
	camera();
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

		// This case deals with the GPS
		//-------------------------------
		case 0:
			medium_loopCounter++;
			if(GPS_enabled)		update_GPS();

			#if HIL_MODE != HIL_MODE_ATTITUDE
				if(g.compass_enabled){
					compass.read();     // Read magnetometer
					compass.calculate(dcm.get_dcm_matrix());  // Calculate heading
					compass.null_offsets(dcm.get_dcm_matrix());
				}
			#endif
/*{
Serial.print(dcm.roll_sensor, DEC);	Serial.printf_P(PSTR("\t"));
Serial.print(dcm.pitch_sensor, DEC);	Serial.printf_P(PSTR("\t"));
Serial.print(dcm.yaw_sensor, DEC);	Serial.printf_P(PSTR("\t"));
Vector3f tempaccel = imu.get_accel();
Serial.print(tempaccel.x, DEC);	Serial.printf_P(PSTR("\t"));
Serial.print(tempaccel.y, DEC);	Serial.printf_P(PSTR("\t"));
Serial.println(tempaccel.z, DEC);
}*/

			break;

		// This case performs some navigation computations
		//------------------------------------------------
		case 1:
 
			medium_loopCounter++;


			if(g_gps->new_data){
				g_gps->new_data 	= false;
				dTnav 				= millis() - nav_loopTimer;
				nav_loopTimer 		= millis();

				// calculate the plane's desired bearing
				// -------------------------------------
				navigate();
			}

			break;

		// command processing
		//------------------------------
		case 2:
			medium_loopCounter++;

			// Read altitude from sensors
			// ------------------
			update_alt();

			// altitude smoothing
			// ------------------
			if (control_mode != FLY_BY_WIRE_B)
				calc_altitude_error();

			// perform next command
			// --------------------
			update_commands();
			break;

		// This case deals with sending high rate telemetry
		//-------------------------------------------------
		case 3:
			medium_loopCounter++;

			#if HIL_MODE != HIL_MODE_ATTITUDE
				if ((g.log_bitmask & MASK_LOG_ATTITUDE_MED) && !(g.log_bitmask & MASK_LOG_ATTITUDE_FAST))
					Log_Write_Attitude((int)dcm.roll_sensor, (int)dcm.pitch_sensor, (uint16_t)dcm.yaw_sensor);

				if (g.log_bitmask & MASK_LOG_CTUN)
					Log_Write_Control_Tuning();
			#endif

			if (g.log_bitmask & MASK_LOG_NTUN)
				Log_Write_Nav_Tuning();

			if (g.log_bitmask & MASK_LOG_GPS)
				Log_Write_GPS(g_gps->time, current_loc.lat, current_loc.lng, g_gps->altitude, current_loc.alt, (long) g_gps->ground_speed, g_gps->ground_course, g_gps->fix, g_gps->num_sats);

			// XXX this should be a "GCS medium loop" interface
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
			check_long_failsafe();
			superslow_loopCounter++;
			if(superslow_loopCounter >=200) {				//	200 = Execute every minute
				#if HIL_MODE != HIL_MODE_ATTITUDE
					if(g.compass_enabled) {
						compass.save_offsets();
					}
				#endif

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
			if (g.switch_enable) {
				update_servo_switches();
			}

			break;

		case 2:
			slow_loopCounter = 0;
			update_events();

			// XXX this should be a "GCS slow loop" interface
			#if GCS_PROTOCOL == GCS_PROTOCOL_MAVLINK
				mavlink_system.sysid = g.sysid_this_mav;		// This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter
				gcs.data_stream_send(1,5);
				// send all requested output streams with rates requested
				// between 1 and 5 Hz
			#else
				gcs.send_message(MSG_LOCATION);
				gcs.send_message(MSG_CPU_LOAD, load*100);
			#endif

			#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && (HIL_MODE != HIL_MODE_DISABLED || HIL_PORT == 0)
				hil.data_stream_send(1,5);
			#endif


			break;
	}
}

void one_second_loop()
{
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();

	// send a heartbeat
	gcs.send_message(MSG_HEARTBEAT);

	#if HIL_PROTOCOL == HIL_PROTOCOL_MAVLINK && (HIL_MODE != HIL_MODE_DISABLED || HIL_PORT == 0)
		hil.send_message(MSG_HEARTBEAT);
	#endif
}

void update_GPS(void)
{
	g_gps->update();
	update_GPS_light();

	if (g_gps->new_data && g_gps->fix) {

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
			ground_start_avg += g_gps->ground_speed;

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

					if (g.log_bitmask & MASK_LOG_CMD)
						Log_Write_Startup(TYPE_GROUNDSTART_MSG);

					init_home();
				} else if (ENABLE_AIR_START == 0) {
					init_home();
				}

				ground_start_count = 0;
			}
		}


		current_loc.lng = g_gps->longitude;    // Lon * 10**7
		current_loc.lat = g_gps->latitude;     // Lat * 10**7

	}
}

void update_current_flight_mode(void)
{
	if(control_mode == AUTO){
		crash_checker();

		switch(command_must_ID){
			case MAV_CMD_NAV_TAKEOFF:
				if (hold_course > -1) {
					calc_nav_roll();
				} else {
					nav_roll = 0;
				}

				if (airspeed_enabled == true)
                                {
					calc_nav_pitch();
					if (nav_pitch < (long)takeoff_pitch) nav_pitch = (long)takeoff_pitch;
				} else {
					nav_pitch = (long)((float)g_gps->ground_speed / (float)g.airspeed_cruise * (float)takeoff_pitch * 0.5);
					nav_pitch = constrain(nav_pitch, 500l, (long)takeoff_pitch);
				}

				g.channel_throttle.servo_out = g.throttle_max; //TODO: Replace with THROTTLE_TAKEOFF or other method of controlling throttle
														//  What is the case for doing something else?  Why wouldn't you want max throttle for TO?
				// ******************************

				break;

			case MAV_CMD_NAV_LAND:
				calc_nav_roll();

				if (airspeed_enabled == true){
					calc_nav_pitch();
					calc_throttle();
				}else{
					calc_nav_pitch();               // calculate nav_pitch just to use for calc_throttle
					calc_throttle();                // throttle based on altitude error
					nav_pitch = landing_pitch;      // pitch held constant
				}

				if (land_complete){
					g.channel_throttle.servo_out = 0;
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
				nav_roll = g.channel_roll.norm_input() * g.roll_limit;
				nav_pitch = g.channel_pitch.norm_input() * (-1) * g.pitch_limit_min;
				// We use pitch_min above because it is usually greater magnitude then pitch_max.  -1 is to compensate for its sign.
				nav_pitch = constrain(nav_pitch, -3000, 3000);	// trying to give more pitch authority
				break;

			case FLY_BY_WIRE_B:
				// fake Navigation output using sticks
				// We use g.pitch_limit_min because its magnitude is
				// normally greater than g.pitch_limit_max
				nav_roll = g.channel_roll.norm_input() * g.roll_limit;
				altitude_error = g.channel_pitch.norm_input() * g.pitch_limit_min;

				if (airspeed_enabled == true)
									{
					airspeed_error = ((int)(g.flybywire_airspeed_max -
							g.flybywire_airspeed_min) *
							g.channel_throttle.servo_out) +
							((int)g.flybywire_airspeed_min * 100);
					// Intermediate calculation - airspeed_error is just desired airspeed at this point
					airspeed_energy_error = (long)(((long)airspeed_error *
								(long)airspeed_error) -
							((long)airspeed * (long)airspeed))/20000;
					//Changed 0.00005f * to / 20000 to avoid floating point calculation
					airspeed_error = (airspeed_error - airspeed);
									}

				calc_throttle();
				calc_nav_pitch();
				break;

			case STABILIZE:
				nav_roll        = 0;
				nav_pitch       = 0;
				// throttle is passthrough
				break;

			case CIRCLE:
				// we have no GPS installed and have lost radio contact
				// or we just want to fly around in a gentle circle w/o GPS
				// ----------------------------------------------------
				nav_roll = g.roll_limit / 3;
				nav_pitch 		= 0;

				if (failsafe != FAILSAFE_NONE){
					g.channel_throttle.servo_out = g.throttle_cruise;
				}
				break;

			case MANUAL:
				// servo_out is for Sim control only
				// ---------------------------------
				g.channel_roll.servo_out = g.channel_roll.pwm_to_angle();
				g.channel_pitch.servo_out = g.channel_pitch.pwm_to_angle();
				g.channel_rudder.servo_out = g.channel_rudder.pwm_to_angle();
				break;
				//roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000

		}
	}
}

void update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------

	// distance and bearing calcs only
	if(control_mode == AUTO){
		verify_commands();
	}else{

		switch(control_mode){
			case LOITER:
				update_loiter();
				calc_bearing_error();
				break;

			case RTL:
				if(wp_distance <= ( g.loiter_radius + LOITER_RANGE) ) {
					do_RTL();
				}else{
					update_crosstrack();
				}
				break;
		}
	}
}


void update_alt()
{
	#if HIL_MODE == HIL_MODE_ATTITUDE
		current_loc.alt = g_gps->altitude;
	#else
		// this function is in place to potentially add a sonar sensor in the future
		//altitude_sensor = BARO;

		current_loc.alt = (1 - g.altitude_mix) * g_gps->altitude;			// alt_MSL centimeters (meters * 100)
		current_loc.alt += g.altitude_mix * (read_barometer() + home.alt);
	#endif

		// Calculate new climb rate
		if(medium_loopCounter == 0 && slow_loopCounter == 0)
			add_altitude_data(millis() / 100, g_gps->altitude / 10);
}
