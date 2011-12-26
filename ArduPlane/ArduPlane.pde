/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduPlane V2.27"
/*
Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler
Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 
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
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_GPS.h>         // ArduPilot GPS library
#include <Wire.h>			// Arduino I2C lib
#include <SPI.h>			// Arduino SPI lib
#include <DataFlash.h>      // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>// ArduPilot Mega polymorphic analog getter
#include <AP_PeriodicProcess.h> // ArduPilot Mega TimerProcess
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
#include <AP_DCM.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>	// Range finder library
#include <ModeFilter.h>
#include <AP_Relay.h>       // APM relay
#include <AP_Mount.h>		// Camera/Antenna mount
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <memcheck.h>

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
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

////////////////////////////////////////////////////////////////////////////////
// ISR Registry
////////////////////////////////////////////////////////////////////////////////
Arduino_Mega_ISR_Registry isr_registry;


////////////////////////////////////////////////////////////////////////////////
// APM_RC_Class Instance
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
static AP_Int8		*flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
#if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844          adc;
#endif

#ifdef DESKTOP_BUILD
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL          compass;
#else

#if CONFIG_BARO == AP_BARO_BMP085
# if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
static AP_Baro_BMP085          barometer(true);
# else
static AP_Baro_BMP085          barometer(false);
# endif
#elif CONFIG_BARO == AP_BARO_MS5611
static AP_Baro_MS5611          barometer;
#endif

static AP_Compass_HMC5843      compass(Parameters::k_param_compass);
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

# if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
  AP_InertialSensor_MPU6000 ins( CONFIG_MPU6000_CHIP_SELECT_PIN );
# else
  AP_InertialSensor_Oilpan ins( &adc );
#endif // CONFIG_IMU_TYPE
AP_IMU_INS imu( &ins, Parameters::k_param_IMU_calibration );
AP_DCM  dcm(&imu, g_gps);

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL              adc;
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL          compass;
AP_GPS_HIL              g_gps_driver(NULL);
AP_InertialSensor_Oilpan ins( &adc );
AP_IMU_Shim imu;
AP_DCM  dcm(&imu, g_gps);

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_ADC_HIL              adc;
AP_DCM_HIL              dcm;
AP_GPS_HIL              g_gps_driver(NULL);
AP_Compass_HIL          compass; // never used
AP_IMU_Shim             imu; // never used

#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

// we always have a timer scheduler
AP_TimerProcess timer_scheduler;


////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
//
GCS_MAVLINK	gcs0(Parameters::k_param_streamrates_port0);
GCS_MAVLINK	gcs3(Parameters::k_param_streamrates_port3);

////////////////////////////////////////////////////////////////////////////////
// PITOT selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilter sonar_mode_filter;

#if CONFIG_PITOT_SOURCE == PITOT_SOURCE_ADC
AP_AnalogSource_ADC pitot_analog_source( &adc,
                        CONFIG_PITOT_SOURCE_ADC_CHANNEL, 1.0);
#elif CONFIG_PITOT_SOURCE == PITOT_SOURCE_ANALOG_PIN
AP_AnalogSource_Arduino pitot_analog_source(CONFIG_PITOT_SOURCE_ANALOG_PIN, 4.0);
#endif

#if SONAR_TYPE == MAX_SONAR_XL
	AP_RangeFinder_MaxsonarXL sonar(&pitot_analog_source, &sonar_mode_filter);
#elif SONAR_TYPE == MAX_SONAR_LV
	// XXX honestly I think these output the same values
	// If someone knows, can they confirm it?
	AP_RangeFinder_MaxsonarXL sonar(&pitot_analog_source, &sonar_mode_filter);
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

byte    control_mode        = INITIALISING;
byte    oldSwitchPosition;              // for remembering the control mode switch
bool    inverted_flight     = false;

#if USB_MUX_PIN > 0
static bool usb_connected;
#endif

static const char *comma = ",";

static const char* flight_mode_strings[] = {
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
			5   Aux5
			6   Aux6
			7   Aux7
			8   Aux8/Mode
		Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
		See libraries/RC_Channel/RC_Channel_aux.h for more information
*/

// Failsafe
// --------
static int 	failsafe;					// track which type of failsafe is being processed
static bool 	ch3_failsafe;
static byte    crash_timer;

// Radio
// -----
static uint16_t elevon1_trim  = 1500; 	// TODO: handle in EEProm
static uint16_t elevon2_trim  = 1500;
static uint16_t ch1_temp      = 1500;     // Used for elevon mixing
static uint16_t ch2_temp  	= 1500;
static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
static bool rc_override_active = false;
static uint32_t rc_override_fs_timer = 0;
static uint32_t ch3_failsafe_timer = 0;

// for elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are equivalent aileron and elevator, not left and right elevon

// LED output
// ----------
static bool GPS_light;							// status of the GPS light

// GPS variables
// -------------
static const 	float t7			= 10000000.0;	// used to scale GPS values for EEPROM storage
static float 	scaleLongUp			= 1;			// used to reverse longitude scaling
static float 	scaleLongDown 		= 1;			// used to reverse longitude scaling
static byte 	ground_start_count	= 5;			// have we achieved first lock and set Home?
static int     ground_start_avg;					// 5 samples to avg speed for ground start
static bool	GPS_enabled 	= false;			// used to quit "looking" for gps with auto-detect if none present

// Location & Navigation
// ---------------------
const	float radius_of_earth 	= 6378100;	// meters
const	float gravity 			= 9.81;		// meters/ sec^2
static long	nav_bearing;						// deg * 100 : 0 to 360 current desired bearing to navigate
static long	target_bearing;						// deg * 100 : 0 to 360 location of the plane to the target
static long	crosstrack_bearing;					// deg * 100 : 0 to 360 desired angle of plane to target
static float	nav_gain_scaler 		= 1;		// Gain scaling for headwind/tailwind TODO: why does this variable need to be initialized to 1?
static long    hold_course       	 	= -1;		// deg * 100 dir of plane

static byte	nav_command_index;					// active nav command memory location
static byte	non_nav_command_index;				// active non-nav command memory location
static byte	nav_command_ID		= NO_COMMAND;	// active nav command ID
static byte	non_nav_command_ID	= NO_COMMAND;	// active non-nav command ID

// Airspeed
// --------
static int		airspeed;							// m/s * 100
static int		airspeed_nudge;  					// m/s * 100 : additional airspeed based on throttle stick position in top 1/2 of range
static long		target_airspeed;					// m/s * 100 (used for Auto-flap deployment in FBW_B mode)
static float	airspeed_error;						// m/s * 100
static long 	energy_error;                       // energy state error (kinetic + potential) for altitude hold
static long		airspeed_energy_error;              // kinetic portion of energy error (m^2/s^2)

// Ground speed
static long		groundspeed_undershoot = 0;				// m/s * 100  (>=0, where > 0 => amount below min ground speed)


// Location Errors
// ---------------
static long	bearing_error;						// deg * 100 : 0 to 36000
static long	altitude_error;						// meters * 100 we are off in altitude
static float	crosstrack_error;					// meters we are off trackline

// Battery Sensors
// ---------------
static float	battery_voltage		= LOW_VOLTAGE * 1.05;		// Battery Voltage of total battery, initialized above threshold for filter
static float 	battery_voltage1 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cell 1, initialized above threshold for filter
static float 	battery_voltage2 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2, initialized above threshold for filter
static float 	battery_voltage3 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3, initialized above threshold for filter
static float 	battery_voltage4 	= LOW_VOLTAGE * 1.05;		// Battery Voltage of cells 1 + 2+3 + 4, initialized above threshold for filter

static float	current_amps;
static float	current_total;

// Airspeed Sensors
// ----------------
static float   airspeed_raw;                       // Airspeed Sensor - is a float to better handle filtering
static float   airspeed_pressure;					// airspeed as a pressure value

// Barometer Sensor variables
// --------------------------
static unsigned long 	abs_pressure;

// Altitude Sensor variables
// ----------------------
static int		sonar_alt;

// flight mode specific
// --------------------
static bool takeoff_complete    = true;         // Flag for using gps ground course instead of IMU yaw.  Set false when takeoff command processes.
static bool	land_complete;
static long	takeoff_altitude;
// static int			landing_distance;					// meters;
static int			landing_pitch;						// pitch for landing set by commands
static int			takeoff_pitch;

// Loiter management
// -----------------
static long 	old_target_bearing;					// deg * 100
static int		loiter_total; 						// deg : how many times to loiter * 360
static int 	loiter_delta;						// deg : how far we just turned
static int		loiter_sum;							// deg : how far we have turned around a waypoint
static long 	loiter_time;						// millis : when we started LOITER mode
static int 	loiter_time_max;					// millis : how long to stay in LOITER mode

// these are the values for navigation control functions
// ----------------------------------------------------
static long	nav_roll;							// deg * 100 : target roll angle
static long	nav_pitch;							// deg * 100 : target pitch angle
static int     throttle_nudge = 0;                 // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel

// Waypoints
// ---------
static long	wp_distance;						// meters - distance between plane and next waypoint
static long	wp_totalDistance;					// meters - distance between old and next waypoint

// repeating event control
// -----------------------
static byte 		event_id; 							// what to do - see defines
static long 		event_timer; 						// when the event was asked for in ms
static uint16_t 	event_delay; 						// how long to delay the next firing of event in millis
static int 		event_repeat = 0;					// how many times to cycle : -1 (or -2) = forever, 2 = do one cycle, 4 = do two cycles
static int 		event_value; 						// per command value, such as PWM for servos
static int 		event_undo_value;					// the value used to cycle events (alternate value to event_value)

// delay command
// --------------
static long 	condition_value;						// used in condition commands (eg delay, change alt, etc.)
static long 	condition_start;
static int 	condition_rate;

// 3D Location vectors
// -------------------
static struct 	Location home;						// home location
static struct 	Location prev_WP;					// last waypoint
static struct 	Location current_loc;				// current location
static struct 	Location next_WP;					// next waypoint
static struct  	Location guided_WP;					// guided mode waypoint
static struct 	Location next_nav_command;			// command preloaded
static struct 	Location next_nonnav_command;		// command preloaded
static long 	target_altitude;					// used for altitude management between waypoints
static long 	offset_altitude;					// used for altitude management between waypoints
static bool	home_is_set; 						// Flag for if we have g_gps lock and have set the home location


// IMU variables
// -------------
static float G_Dt						= 0.02;		// Integration time for the gyros (DCM algorithm)


// Performance monitoring
// ----------------------
static long 	perf_mon_timer;						// Metric based on accel gain deweighting
static int 	G_Dt_max = 0;						// Max main loop cycle time in milliseconds
static int 	gps_fix_count = 0;
static int		pmTest1 = 0;


// System Timers
// --------------
static unsigned long 	fast_loopTimer;				// Time in miliseconds of main control loop
static unsigned long 	fast_loopTimeStamp;			// Time Stamp when fast loop was complete
static uint8_t 		delta_ms_fast_loop; 		// Delta Time in miliseconds
static uint16_t			mainLoop_count;

static unsigned long 	medium_loopTimer;			// Time in miliseconds of medium loop
static byte 			medium_loopCounter;			// Counters for branching from main control loop to slower loops
static uint8_t			delta_ms_medium_loop;

static byte 			slow_loopCounter;
static byte 			superslow_loopCounter;
static byte			counter_one_herz;

static unsigned long 	nav_loopTimer;				// used to track the elapsed time for GPS nav

static unsigned long 	dTnav;						// Delta Time in milliseconds for navigation computations
static float 			load;						// % MCU cycles used

AP_Relay relay;

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
AP_Mount camera_mount(g_gps, &dcm);
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
				if (g.log_bitmask & MASK_LOG_PM)
					#if HIL_MODE != HIL_MODE_ATTITUDE
					Log_Write_Performance();
					#endif

				resetPerfData();
			}
		}

		fast_loopTimeStamp = millis();
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

	// Read Airspeed
	// -------------
	if (g.airspeed_enabled == true) {
#if HIL_MODE != HIL_MODE_ATTITUDE
		read_airspeed();
#else
		calc_airspeed_errors();
#endif
	}

	#if HIL_MODE == HIL_MODE_SENSORS
		// update hil before dcm update
		gcs_update();
	#endif

	dcm.update_DCM();

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
	set_servos();


	// XXX is it appropriate to be doing the comms below on the fast loop?

    gcs_update();
    gcs_data_stream_send(45,1000);
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
			if(GPS_enabled){
                update_GPS();
                calc_gndspeed_undershoot();
            }

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
			if(g.sonar_enabled) sonar_alt = sonar.read();

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

            // send all requested output streams with rates requested
            // between 5 and 45 Hz
            gcs_data_stream_send(5,45);
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
			update_servo_switches();

			update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8);

#if MOUNT == ENABLED
			camera_mount.update_mount_type();
#endif
			break;

		case 2:
			slow_loopCounter = 0;
			update_events();

            mavlink_system.sysid = g.sysid_this_mav;		// This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter
            gcs_data_stream_send(3,5);

#if USB_MUX_PIN > 0
            check_usb_mux();
#endif

			break;
	}
}

static void one_second_loop()
{
	if (g.log_bitmask & MASK_LOG_CUR)
		Log_Write_Current();

	// send a heartbeat
	gcs_send_message(MSG_HEARTBEAT);
    gcs_data_stream_send(1,3);
}

static void update_GPS(void)
{
	g_gps->update();
	update_GPS_light();

	if (g_gps->new_data && g_gps->fix) {
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

        // see if we've breached the geo-fence
        geofence_check(false);
	}
}

static void update_current_flight_mode(void)
{
	if(control_mode == AUTO){
		crash_checker();

		switch(nav_command_ID){
			case MAV_CMD_NAV_TAKEOFF:
				if (hold_course > -1) {
					calc_nav_roll();
				} else {
					nav_roll = 0;
				}

				if (g.airspeed_enabled == true)
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

				if (g.airspeed_enabled == true){
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
			case GUIDED:
				hold_course = -1;
				crash_checker();
				calc_nav_roll();
				calc_nav_pitch();
				calc_throttle();
				break;

			case FLY_BY_WIRE_A:
				// set nav_roll and nav_pitch using sticks
				nav_roll = g.channel_roll.norm_input() * g.roll_limit;
				nav_pitch = g.channel_pitch.norm_input() * (-1) * g.pitch_limit_min;
				// We use pitch_min above because it is usually greater magnitude then pitch_max.  -1 is to compensate for its sign.
				nav_pitch = constrain(nav_pitch, -3000, 3000);	// trying to give more pitch authority
				if (inverted_flight) nav_pitch = -nav_pitch;
				break;

			case FLY_BY_WIRE_B:
				// Substitute stick inputs for Navigation control output
				// We use g.pitch_limit_min because its magnitude is
				// normally greater than g.pitch_limit_max
				nav_roll = g.channel_roll.norm_input() * g.roll_limit;
				altitude_error = g.channel_pitch.norm_input() * g.pitch_limit_min;

				if ((current_loc.alt>=home.alt+g.FBWB_min_altitude) || (g.FBWB_min_altitude == -1)) {
	 				altitude_error = g.channel_pitch.norm_input() * g.pitch_limit_min;
				} else {
					if (g.channel_pitch.norm_input()<0)
						altitude_error =( (home.alt + g.FBWB_min_altitude) - current_loc.alt) + g.channel_pitch.norm_input() * g.pitch_limit_min ;
					else altitude_error =( (home.alt + g.FBWB_min_altitude) - current_loc.alt) ;
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

static void update_navigation()
{
	// wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
	// ------------------------------------------------------------------------

	// distance and bearing calcs only
	if(control_mode == AUTO){
		verify_commands();
	}else{

		switch(control_mode){
			case LOITER:
			case RTL:
			case GUIDED:
				update_loiter();
				calc_bearing_error();
				break;

		}
	}
}


static void update_alt()
{
	#if HIL_MODE == HIL_MODE_ATTITUDE
		current_loc.alt = g_gps->altitude;
	#else
		// this function is in place to potentially add a sonar sensor in the future
		//altitude_sensor = BARO;

		current_loc.alt = (1 - g.altitude_mix) * g_gps->altitude;			// alt_MSL centimeters (meters * 100)
		current_loc.alt += g.altitude_mix * (read_barometer() + home.alt);
	#endif

        geofence_check(true);

		// Calculate new climb rate
		//if(medium_loopCounter == 0 && slow_loopCounter == 0)
		//	add_altitude_data(millis() / 100, g_gps->altitude / 10);
}
