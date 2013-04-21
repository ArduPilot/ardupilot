/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArdupilotTracker V2.00"
/*
 *  Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher
 *  Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon
 *  Please contribute your ideas!
 *
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
//#include <AP_RangeFinder.h>     // Range finder library
#include <Filter.h>                     // Filter library
#include <AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay.h>       // APM relay
//#include <AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed.h>
#include <memcheck.h>

#include <APM_Control.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash.h>
#include <SITL.h>

#include <MobileDriver.h>

// optional new controller library
#if APM_CONTROL == ENABLED
#include <APM_Control.h>
#endif

// Pre-AP_HAL compatibility
#include "compat.h"

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include "GCS.h"
#include "tracking.h"

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

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

////////////////////////////////////////////////////////////////////////////////
// prototypes
// static void update_events(void);
void gcs_send_text_fmt(const prog_char_t *fmt, ...);

////////////////////////////////////////////////////////////////////////////////
// DataFlash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#else
// no dataflash driver
DataFlash_Empty DataFlash;
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
static AP_Int8          *flight_modes = &g.flight_mode1;

#if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844 adc;
#endif

#if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == AP_BARO_HIL
static AP_Baro_BMP085_HIL barometer;
#elif CONFIG_BARO == AP_BARO_MS5611
 #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
 static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
 #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
 static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
 #else
 #error Unrecognized CONFIG_MS5611_SERIAL setting.
 #endif
#else
 #error Unrecognized CONFIG_BARO setting
#endif

#if CONFIG_COMPASS == AP_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == AP_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == AP_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

// GPS selection
#if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

#elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver;

#elif GPS_PROTOCOL == GPS_PROTOCOL_HIL
AP_GPS_HIL      g_gps_driver;

#else
  #error Unrecognised GPS_PROTOCOL setting.
#endif // GPS PROTOCOL

#if CONFIG_INS_TYPE == CONFIG_INS_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_PX4
AP_InertialSensor_PX4 ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_STUB
AP_InertialSensor_Stub ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_OILPAN
AP_InertialSensor_Oilpan ins( &adc );
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE

#if HIL_MODE == HIL_MODE_ATTITUDE
AP_AHRS_HIL ahrs(&ins, g_gps);
#else
AP_AHRS_DCM ahrs(&ins, g_gps);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

////////////////////////////////////////////////////////////////////////////////
// Datacomms interfaces support MAVLink, ArduTracker and maybe others too.
////////////////////////////////////////////////////////////////////////////////
Datacomm_Class_Multiple datacomm0;
Datacomm_Class_Multiple datacomm1;

GCS_MAVLINK mavlink0;
GCS_MAVLINK mavlink1;

ArduTrackerDataInterpreter ardutracker1;

////////////////////////////////////////////////////////////////////////////////
// Analog Inputs
////////////////////////////////////////////////////////////////////////////////

AP_HAL::AnalogSource *vcc_pin;
AP_HAL::AnalogSource * batt_volt_pin;
AP_HAL::AnalogSource * batt_curr_pin;

////////////////////////////////////////////////////////////////////////////////
// Relay
////////////////////////////////////////////////////////////////////////////////
AP_Relay relay;

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// APM2 only
#if USB_MUX_PIN > 0
static bool usb_connected;
#endif

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
enum TrackingMode control_mode  = INITIALISING;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
uint8_t oldSwitchPosition;
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
// static int16_t failsafe;
// Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
// RC receiver should be set up to output a low throttle value when signal is lost
// static bool ch3_failsafe;

// the time when the last HEARTBEAT message arrived from a GCS
static uint32_t last_heartbeat_ms;

// A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
// static uint32_t ch3_failsafe_timer = 0;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// state of the GPS light (on/off)
static uint8_t GPS_light;

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
// Used to compute a speed estimate from the first valid gps fixes to decide if we are
// on the ground or in the air.  Used to decide if a ground start is appropriate if we
// booted with an air start.
static int16_t ground_start_avg;

// true if we have a position estimate from AHRS
static bool have_position;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Constants
const float radius_of_earth   = 6378100;        // meters

static int32_t neutral_bearing_cd;
static bool tracker_initialized;

static int32_t azimuthRCIntegral;
static int32_t elevationRCIntegral;

// This is the direction to the next waypoint or loiter center
// deg * 100 : 0 to 360
// static int32_t target_bearing_cd;

static struct AzimuthElevation incomingAzimuthElevation = {0,0,0};
static struct AzimuthElevation servoAzimuthElevation = {0,0,0};

////////////////////////////////////////////////////////////////////////////////
// Location Errors
////////////////////////////////////////////////////////////////////////////////
// Difference between current bearing and desired bearing.  Hundredths of a degree
// static int32_t bearing_error_cd;

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery pack 1 voltage.  Initialized above the low voltage threshold to pre-load the filter and prevent low voltage events at startup.
static float battery_voltage1        = LOW_VOLTAGE * 1.05;
// Battery pack 1 instantaneous currrent draw.  Amperes
static float current_amps1;
// Totalized current (Amp-hours) from battery 1
static float current_total1;

// To Do - Add support for second battery pack
//static float  battery_voltage2    = LOW_VOLTAGE * 1.05;		// Battery 2 Voltage, initialized above threshold for filter
//static float	current_amps2;									// Current (Amperes) draw from battery 2
//static float	current_total2;									// Totalized current (Amp-hours) from battery 2

////////////////////////////////////////////////////////////////////////////////
// Loiter management
////////////////////////////////////////////////////////////////////////////////
// Previous target bearing.  Used to calculate loiter rotations.  Hundredths of a degree
// static int32_t old_target_bearing_cd;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
// static int32_t nav_roll_cd;

// The instantaneous desired pitch angle.  Hundredths of a degree
// static int32_t nav_pitch_cd;

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances
////////////////////////////////////////////////////////////////////////////////
// Distance between plane and next waypoint.  Meters
// is not static because AP_Camera uses it
uint32_t wp_distance;

// Distance between previous and next waypoint.  Meters
// static uint32_t wp_totalDistance;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
// The home location used for its altitude only. 
// The location is set when we first get stable GPS lock
static struct   Location home;
// Flag for if we have g_gps lock and have set the home location
static bool home_is_set;

// The home location used for RTL.  The location is set when we first get stable GPS lock
static struct   Location target;
// Flag for if we have received a target location
static bool target_is_set;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
// static struct   Location prev_WP;
// The plane's current location
static struct   Location current_loc;
// The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
static struct   Location next_WP;
// The location of the active waypoint in Guided mode.
// static struct   Location guided_WP;
// The location structure information from the Nav command being processed
// static struct   Location next_nav_command;
// The location structure information from the Non-Nav command being processed
// static struct   Location next_nonnav_command;

////////////////////////////////////////////////////////////////////////////////
// Altitude / Climb rate control
////////////////////////////////////////////////////////////////////////////////
// The current desired altitude.  Altitude is linearly ramped between waypoints.  Centimeters
// static int32_t target_altitude_cm;
// Altitude difference between previous and current waypoint.  Centimeters
// static int32_t offset_altitude_cm;

////////////////////////////////////////////////////////////////////////////////
// INS variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt                                               = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
// static int32_t perf_mon_timer;
// The maximum main loop execution time recorded in the current performance monitoring interval
static int16_t G_Dt_max = 0;
// The number of gps fixes recorded in the current performance monitoring interval
static uint8_t gps_fix_count = 0;
// A variable used by developers to track performanc metrics.
// Currently used to record the number of GCS heartbeat messages received
static int16_t pmTest1 = 0;


////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in miliseconds of start of main control loop.  Milliseconds
static uint32_t fast_loopTimer_ms;

// Time Stamp when fast loop was complete.  Milliseconds
static uint32_t fast_loopTimeStamp_ms;

// Number of milliseconds used in last main loop cycle
static uint8_t delta_ms_fast_loop;

// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;

// Time in miliseconds of start of medium control loop.  Milliseconds
static uint32_t medium_loopTimer_ms;

// Counters for branching from main control loop to slower loops
static uint8_t medium_loopCounter;
// Number of milliseconds used in last medium loop cycle
static uint8_t delta_ms_medium_loop;

// Counters for branching from medium control loop to slower loops
static uint8_t slow_loopCounter;
// Counter to trigger execution of very low rate processes
static uint8_t superslow_loopCounter;
// Counter to trigger execution of 1 Hz processes
static uint8_t counter_one_herz;

// % MCU cycles used
static float load;


// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
AP_Mount camera_mount(&current_loc, g_gps, &ahrs, 0);
#endif

#if MOUNT2 == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
AP_Mount camera_mount2(&current_loc, g_gps, &ahrs, 1);
#endif

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

void setup() {
    // this needs to be the first call, as it fills memory with
    // sentinel values
    memcheck_init();

    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    vcc_pin = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    batt_volt_pin = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_pin = hal.analogin->channel(g.battery_curr_pin);
    
    init_ardupilot();
}

void loop()
{
    // We want this to execute at 50Hz, but synchronised with the gyro/accel
    uint16_t num_samples = ins.num_samples_available();
    if (num_samples >= 1) {
        delta_ms_fast_loop      = millis() - fast_loopTimer_ms;
        load                = (float)(fast_loopTimeStamp_ms - fast_loopTimer_ms)/delta_ms_fast_loop;
        G_Dt                = (float)delta_ms_fast_loop / 1000.f;
        fast_loopTimer_ms   = millis();

        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // Execute the medium loop
        // -----------------------
        medium_loop();

        counter_one_herz++;
        if(counter_one_herz == 50) {
            one_second_loop();
            counter_one_herz = 0;
        }

        fast_loopTimeStamp_ms = millis();
    } else if (millis() - fast_loopTimeStamp_ms < 19) {
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

    // TODO: There must be a better way to capture the first valid bearing than this...
    // but my attempts to do it in initialization always just captured zero.
    if(!tracker_initialized && ahrs.yaw_initialised()) {
        neutral_bearing_cd = ahrs.yaw_sensor;
        // I don't like servos that get initialized to a default value, and shortly
        // after jump to a different quiescent position. Better just leave them off
        // until there first meaningful position for them is ready.
        init_servos();
        tracker_initialized = true;
    }
    
    // Read radio
    // ----------
    read_radio();

    // try to send any deferred messages if the serial port now has
    // some space available
    gcs_send_message(MSG_RETRY_DEFERRED);

    ahrs.update();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    // update_current_flight_mode();
    // apply desired roll, pitch and yaw to the plane
    // ----------------------------------------------
     // if (control_mode > MANUAL)
    track();

    // write out the servo PWM values
    // ------------------------------
    set_servos();
    
    /*
    if ((mainLoop_count % 20) == 0) {
    	char blah[40];
    	hal.util->snprintf_P(blah, 40, PSTR("Yaw: %d"), ahrs.yaw_sensor/100);
    	mavlink_send_text(MAVLINK_COMM_0, SEVERITY_LOW, blah);
    }
    */

    datacomm_update();
    gcs_data_stream_send();
}

static void medium_loop()
{
#if MOUNT == ENABLED
    camera_mount.update_mount_position();
#endif

#if MOUNT2 == ENABLED
    camera_mount2.update_mount_position();
#endif

    // This is the start of the medium (10 Hz) loop pieces
    // -----------------------------------------
    switch(medium_loopCounter) {

    // This case deals with the GPS
    //-------------------------------
    case 0:
        medium_loopCounter++;
        update_GPS();
//        calc_gndspeed_undershoot();

#if HIL_MODE != HIL_MODE_ATTITUDE
        if (g.compass_enabled && compass.read()) {
            ahrs.set_compass(&compass);
            compass.null_offsets();
        } else {
            ahrs.set_compass(NULL);
        }
#endif

        break;

    // This case performs some navigation computations
    //------------------------------------------------
    case 1:
        medium_loopCounter++;

        // Read 6-position switch on radio
        // -------------------------------
        read_control_switch();

        // calculate the plane's desired bearing
        // -------------------------------------
        navigate();

        break;

    // command processing
    //------------------------------
    case 2:
        medium_loopCounter++;

        // Read altitude from sensors
        // ------------------
        update_alt();

        // perform next command
        // --------------------
        // update_commands();
        break;

    // This case deals with sending high rate telemetry
    //-------------------------------------------------
    case 3:
        medium_loopCounter++;
        break;

    // This case controls the slow loop
    //---------------------------------
    case 4:
        medium_loopCounter = 0;
        delta_ms_medium_loop    = millis() - medium_loopTimer_ms;
        medium_loopTimer_ms     = millis();

        if (g.battery_monitoring != 0) {
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
    switch (slow_loopCounter) {
    case 0:
        slow_loopCounter++;
        superslow_loopCounter++;
        if(superslow_loopCounter >=200) {                                               //	200 = Execute every minute
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
        update_aux_servo_function(&g.rc_3, &g.rc_4);
        enable_aux_servos();

#if MOUNT == ENABLED
        camera_mount.update_mount_type();
#endif
#if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
#endif
        break;

    case 2:
        slow_loopCounter = 0;
        // update_events();
        mavlink_system.sysid = g.sysid_this_mav;                // This is just an ugly hack to keep mavlink_system.sysid sync'd with our parameter
        check_usb_mux();
        break;
    }
}

static void one_second_loop()
{

    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);
}

static void update_GPS(void)
{
    g_gps->update();
    update_GPS_light();

    // get position from AHRS
    have_position = ahrs.get_position(&current_loc);

    if (g_gps->new_data && g_gps->fix) {
        g_gps->new_data = false;

        // for performance
        // ---------------
        gps_fix_count++;

        if(ground_start_count > 1) {
            ground_start_count--;
            ground_start_avg += g_gps->ground_speed;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 5;

            } else {
                if(ENABLE_AIR_START == 1 && (ground_start_avg / 5) < SPEEDFILT) {
                    startup_ground();
                    init_home();
                } else if (ENABLE_AIR_START == 0) {
                    init_home();
                }

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                }
                ground_start_count = 0;
            }
        }
    }
}

/*
static void update_navigation()
{
    switch(control_mode) {
    default:
        // nothing to do
        break;
    }
}
*/

static void update_alt()
{
#if HIL_MODE == HIL_MODE_ATTITUDE
    current_loc.alt = g_gps->altitude;
#else
    // this function is in place to potentially add a sonar sensor in the future
    //altitude_sensor = BARO;

    if (barometer.healthy) {
        current_loc.alt = (1 - g.altitude_mix) * g_gps->altitude;                       // alt_MSL centimeters (meters * 100)
        current_loc.alt += g.altitude_mix * (read_barometer() + home.alt);
    } else if (g_gps->fix) {
        current_loc.alt = g_gps->altitude;     // alt_MSL centimeters (meters * 100)
    }
#endif
    // Calculate new climb rate
    //if(medium_loopCounter == 0 && slow_loopCounter == 0)
    //	add_altitude_data(millis() / 100, g_gps->altitude / 10);
}

AP_HAL_MAIN();
