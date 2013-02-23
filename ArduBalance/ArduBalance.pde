/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduBalance V1.a"
/*
ArduBalance Version 1
Lead author:    Jason Short

This firmware is free software; you can redistribute it and / or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

Special Thanks for Contributors:


And much more so PLEASE PM me on DIYDRONES to add your contribution to the List

Requires modified "mrelax" version of Arduino, which can be found here:
http: //code.google.com / p/ardupilot - mega / downloads / list

*/

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <math.h>

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Menu.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>             // ArduPilot Mega RC Library
#include <AP_GPS.h>             // ArduPilot GPS library
#include <I2C.h>                // Arduino I2C lib
#include <SPI.h>                // Arduino SPI lib
#include <SPI3.h>               // SPI3 library
#include <AP_Semaphore.h>       // for removing conflict between optical flow and dataflash on SPI3 bus
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_PeriodicProcess.h> // Parent header of Timer
                                // (only included for makefile libpath to work)
#include <AP_TimerProcess.h>    // TimerProcess is the scheduler for MPU6000 reads.
#include <AP_AHRS.h>
#include <APM_PI.h>            	// PI library
#include <AC_PID.h>            	// PID library
#include <RC_Channel.h>     	// RC Channel Library
#include <AP_RangeFinder.h>    	// Range finder library
#include <AP_OpticalFlow.h> 	// Optical Flow library
#include <Filter.h>        		// Filter library
#include <AP_Buffer.h>     		// APM FIFO Buffer
#include <ModeFilter.h>    		// Mode Filter from Filter library
#include <AverageFilter.h> 		// Mode Filter from Filter library
#include <AP_LeadFilter.h> 		// GPS Lead filter
#include <LowPassFilter.h>      // Low Pass Filter library
#include <AP_Relay.h>      		// APM relay
#include <AP_Camera.h>     		// Photo or video camera
#include <AP_Mount.h>      		// Camera/Antenna mount
#include <AP_Airspeed.h>   		// needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <DigitalWriteFast.h>   // faster digital write for LEDs
#include <memcheck.h>

// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

#include <GCS_MAVLink.h>        // MAVLink GCS definitions

// Local modules
#include "Parameters.h"
#include "GCS.h"

#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library

// Limits library - Puts limits on the vehicle, and takes recovery actions
#include <AP_Limits.h>
#include <AP_Limit_GPSLock.h>   // a limits library module
#include <AP_Limit_Geofence.h>  // a limits library module
#include <AP_Limit_Altitude.h>  // a limits library module


////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI / console
FastSerialPort1(Serial1);       // GPS port
FastSerialPort3(Serial3);       // Telemetry port

// port to use for command line interface
static FastSerial *cliSerial = &Serial;

// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
AP_Param param_loader(var_info, WP_START_BYTE);

Arduino_Mega_ISR_Registry isr_registry;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;


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
AP_Semaphore spi_semaphore;
AP_Semaphore spi3_semaphore;
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
DataFlash_APM2 DataFlash(&spi3_semaphore);
#else
DataFlash_APM1 DataFlash(&spi_semaphore);
#endif


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_200HZ;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
 #if CONFIG_ADC == ENABLED
AP_ADC_ADS7844 adc;
 #endif

 #ifdef DESKTOP_BUILD
//AP_Baro_BMP085_HIL barometer;
AP_Compass_HIL compass;
  #include <SITL.h>
SITL sitl;
 #else

/*  #if CONFIG_BARO == AP_BARO_BMP085
   # if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
//AP_Baro_BMP085 barometer(true);
   # else
//AP_Baro_BMP085 barometer(false);
   # endif
  #elif CONFIG_BARO == AP_BARO_MS5611
//AP_Baro_MS5611 barometer;
  #endif
*/

AP_Compass_HMC5843 compass;
 #endif

 #if OPTFLOW == ENABLED
  #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
AP_OpticalFlow_ADNS3080 optflow(OPTFLOW_CS_PIN);
  #else
AP_OpticalFlow_ADNS3080 optflow(OPTFLOW_CS_PIN);
  #endif
 #else
AP_OpticalFlow optflow;
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
AP_InertialSensor_MPU6000 ins;
 #else
AP_InertialSensor_Oilpan ins(&adc);
 #endif

// we don't want to use gps for yaw correction on ArduCopter, so pass
// a NULL GPS object pointer
static GPS         *g_gps_null;

 #if DMP_ENABLED == ENABLED && CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
AP_AHRS_MPU6000  ahrs(&ins, g_gps);               // only works with APM2
 #else
AP_AHRS_DCM ahrs(&ins, g_gps);
 #endif

// ahrs2 object is the secondary ahrs to allow running DMP in parallel with DCM
  #if SECONDARY_DMP_ENABLED == ENABLED && CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
AP_AHRS_MPU6000  ahrs2(&ins, g_gps);               // only works with APM2
  #endif

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL              adc;
//AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL          compass;
AP_GPS_HIL              g_gps_driver(NULL);
AP_InertialSensor_Stub  ins;
AP_AHRS_DCM             ahrs(&ins, g_gps);


static int32_t gps_base_alt;

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_ADC_HIL              adc;
AP_InertialSensor_Stub  ins;
AP_AHRS_HIL             ahrs(&ins, g_gps);
AP_GPS_HIL              g_gps_driver(NULL);
AP_Compass_HIL          compass;                  // never used
//AP_Baro_BMP085_HIL      barometer;

 #if OPTFLOW == ENABLED
  #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
AP_OpticalFlow_ADNS3080 optflow(OPTFLOW_CS_PIN);
  #else
AP_OpticalFlow_ADNS3080 optflow(OPTFLOW_CS_PIN);
  #endif    // CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
 #endif     // OPTFLOW == ENABLED

 #ifdef DESKTOP_BUILD
  #include <SITL.h>
SITL sitl;
 #endif     // DESKTOP_BUILD
static int32_t gps_base_alt;
#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

// we always have a timer scheduler
AP_TimerProcess timer_scheduler;

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
GCS_MAVLINK gcs0;
GCS_MAVLINK gcs3;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size5 sonar_mode_filter(2);
#if CONFIG_SONAR == ENABLED
 #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
AP_AnalogSource_ADC sonar_analog_source(&adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
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

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries / RC_Channel / RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set        : 1; // 1
        uint8_t simple_mode        : 1; // 2    // This is the state of simple mode
        uint8_t manual_attitude    : 1; // 3
        uint8_t manual_throttle    : 1; // 4

        uint8_t low_battery        : 1; // 5    // Used to track if the battery is low - LED output flashes when the batt is low
        uint8_t loiter_override    : 1; // 6    // Are we navigating while holding a positon? This is set to false once the speed drops below 1m / s
        uint8_t armed              : 1; // 7
        uint8_t auto_armed         : 1; // 8

        uint8_t failsafe           : 1; // 9    // A status flag for the failsafe state
        uint8_t do_flip            : 1; // 10   // Used to enable flip code
        uint8_t takeoff_complete   : 1; // 11
        uint8_t land_complete      : 1; // 12
        uint8_t compass_status     : 1; // 13
        uint8_t gps_status         : 1; // 14
        uint8_t fast_corner        : 1; // 15   // should we take the waypoint quickly or slow down?
    };
    uint16_t value;
} ap;


static struct AP_System{
    uint8_t GPS_light               : 1; // 1   // Solid indicates we have full 3D lock and can navigate, flash = read
    uint8_t motor_light             : 1; // 2   // Solid indicates Armed state
    uint8_t new_radio_frame         : 1; // 3   // Set true if we have new PWM data to act on from the Radio
    uint8_t nav_ok                  : 1; // 4   // deprecated
    uint8_t CH7_flag                : 1; // 5   // manages state of the ch7 toggle switch
    uint8_t usb_connected           : 1; // 6   // true if APM is powered from USB connection
    uint8_t run_50hz_loop           : 1; // 7   // toggles the 100hz loop for 50hz
    uint8_t alt_sensor_flag         : 1; // 8   // used to track when to read sensors vs estimate alt
    uint8_t yaw_stopped             : 1; // 9   // Used to manage the Yaw hold capabilities

} ap_system;


////////////////////////////////////////////////////////////////////////////////
// velocity in lon and lat directions calculated from GPS position and accelerometer data
// updated after GPS read - 5-10hz
//static int16_t lon_speed;       // expressed in cm / s.  positive numbers mean moving east
//static int16_t lat_speed;       // expressed in cm / s.  positive numbers when moving north

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t x_rate_error;
static int16_t y_rate_error;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
static byte oldSwitchPosition;

// receiver RSSI
static uint8_t receiver_rssi;


////////////////////////////////////////////////////////////////////////////////
// Mavlink/HIL control
////////////////////////////////////////////////////////////////////////////////
// Used to track the GCS based control input
// Allow override of RC channel values for HIL
static int16_t rc_override[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// Status flag that tracks whether we are under GCS control
static bool rc_override_active = false;
// Status flag that tracks whether we are under GCS control
static uint32_t rc_override_fs_timer;

////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////
// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;
// This is used to hold radio tuning values for in-flight CH6 tuning
static float tuning_value;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// This is current status for the LED lights state machine
// setting this value changes the output of the LEDs
static byte led_mode = NORMAL_LEDS;
// Blinking indicates GPS status
static byte copter_leds_GPS_blink;
// Blinking indicates battery status
static byte copter_leds_motor_blink;
// Navigation confirmation blinks
static int8_t copter_leds_nav_blink;

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;


////////////////////////////////////////////////////////////////////////////////
// Mavlink specific
////////////////////////////////////////////////////////////////////////////////
// Used by Mavlink for unknow reasons
static const float radius_of_earth = 6378100;   // meters
// Used by Mavlink for unknow reasons
static const float gravity = 9.80665;           // meters/ sec ^ 2

// Unions for getting byte values
union float_int {
    int32_t int_value;
    float float_value;
} float_int;


////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t wp_bearing;
// Status of the Waypoint tracking mode. Options include:
// NO_NAV_MODE, WP_MODE, LOITER_MODE, CIRCLE_MODE
static byte wp_control;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?
static uint8_t wp_verify_byte;                                                  // used for tracking state of navigating waypoints
// used to limit the speed ramp up of WP navigation
// Acceleration is limited to 1m/s/s
static int16_t max_speed_old;
// Used to track how many cm we are from the "next_WP" location
static int32_t long_error, lat_error;
static int16_t control_roll;
static int16_t control_pitch;
//static uint8_t rtl_state;

////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
static float cos_roll_x         = 1;
static float cos_pitch_x        = 1;
static float cos_yaw_x          = 1;
static float sin_yaw_y;
static float sin_roll;
static float sin_pitch;

////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
// Used to control Axis lock
int32_t roll_axis;
int32_t pitch_axis;

// Filters
AP_LeadFilter xLeadFilter;      // Long GPS lag filter
AP_LeadFilter yLeadFilter;      // Lat  GPS lag filter

//AverageFilterInt32_Size3 roll_rate_d_filter;    // filtered acceleration
//AverageFilterInt32_Size3 pitch_rate_d_filter;   // filtered pitch acceleration

// Barometer filter
//AverageFilterInt32_Size5 baro_filter;

////////////////////////////////////////////////////////////////////////////////
// Circle Mode / Loiter control
////////////////////////////////////////////////////////////////////////////////
// used to determin the desired location in Circle mode
// increments at circle_rate / second
static float circle_angle;
// used to control the speed of Circle mode
// units are in radians, default is 5° per second
static const float circle_rate = 0.0872664625;
// used to track the delat in Circle Mode
static int32_t old_wp_bearing;
// deg : how many times to circle * 360 for Loiter/Circle Mission command
static int16_t loiter_total;
// deg : how far we have turned around a waypoint
static int16_t loiter_sum;
// How long we should stay in Loiter Mode for mission scripting
static uint16_t loiter_time_max;
// How long have we been loitering - The start time in millis
static uint32_t loiter_time;
// The synthetic location created to make the copter do circles around a WP
static struct   Location circle_WP;


////////////////////////////////////////////////////////////////////////////////
// CH7 control
////////////////////////////////////////////////////////////////////////////////
// This register tracks the current Mission Command index when writing
// a mission using CH7 in flight
static int8_t CH7_wp_index;


////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery Voltage of battery, initialized above threshold for filter
static float battery_voltage1 = LOW_VOLTAGE * 1.05;
// refers to the instant amp draw – based on an Attopilot Current sensor
static float current_amps1;
// refers to the total amps drawn – based on an Attopilot Current sensor
static float current_total1;


////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// flatland!

////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static uint8_t yaw_mode;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode;
// The current desired control scheme for altitude hold
//static uint8_t throttle_mode;


////////////////////////////////////////////////////////////////////////////////
// flight specific
////////////////////////////////////////////////////////////////////////////////
// Flatland!

////////////////////////////////////////////////////////////////////////////////
// Navigation general
////////////////////////////////////////////////////////////////////////////////
// The location of home in relation to the copter, updated every GPS read
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next_WP in cm
// is not static because AP_Camera uses it
int32_t wp_distance;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// home location is stored when we have a good GPS lock and arm the copter
// Can be reset each the copter is re-armed
static struct   Location home;
// Current location of the copter
static struct   Location current_loc;
// Next WP is the desired location of the copter - the next waypoint or loiter location
static struct   Location next_WP;
// Prev WP is used to get the optimum path from one WP to the next
static struct   Location prev_WP;
// Holds the current loaded command from the EEPROM for navigation
static struct   Location command_nav_queue;
// Holds the current loaded command from the EEPROM for conditional scripts
static struct   Location command_cond_queue;
// Holds the current loaded command from the EEPROM for guided mode
static struct   Location guided_WP;


////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t original_wp_bearing;
// The amount of angle correction applied to wp_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;


////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// all angles are deg * 100 : target yaw angle
// The Commanded ROll from the autopilot.
//static int32_t nav_roll;
// The Commanded pitch from the autopilot. negative Pitch means go forward.
static int32_t nav_pitch;
// The desired bank towards North (Positive) or South (Negative)
//static int32_t auto_roll;
static int32_t auto_pitch;

// Don't be fooled by the fact that Pitch is reversed from Roll in its sign!
static int16_t nav_lat;
// The desired bank towards East (Positive) or West (Negative)
static int16_t nav_lon;
// The Commanded ROll from the autopilot based on optical flow sensor.
//static int32_t of_roll;
// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
//static int32_t of_pitch;


////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Climb rate control
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t nav_yaw;
static uint8_t yaw_timer;
// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
static struct Location yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
static int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;



////////////////////////////////////////////////////////////////////////////////
// Repeat Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
// The type of repeating event - Toggle a servo channel, Toggle the APM1 relay, etc
static byte event_id;
// Used to manage the timimng of repeating events
static uint32_t event_timer;
// How long to delay the next firing of event in millis
static uint16_t event_delay;
// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
static int16_t event_repeat;
// per command value, such as PWM for servos
static int16_t event_value;
// the stored value used to undo commands - such as original PWM command
static int16_t event_undo_value;

////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
AP_InertialNav inertial_nav(&ahrs, &ins, NULL, &g_gps);
#endif

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Used to manage the rate of performance logging messages
static int16_t perf_mon_counter;
// The number of GPS fixes we have had
static int16_t gps_fix_count;


////////////////////////////////////////////////////////////////////////////////
// Balance specific
////////////////////////////////////////////////////////////////////////////////
static int16_t desired_nav_speed;
static int16_t desired_balance_speed;
#define PWM_LUT_SIZE 40

int16_t fail;
static bool tilt_start;
//static int16_t pwm_LUT[PWM_LUT_SIZE];
// This implementation cannot have a value that's lower than the previous index value in the lut:
//                            0  1    2    3    4    5    6    7    8    9   10    11   12   13   14    15    16    17    18    19    20
//static int16_t pwm_LUT_R[PWM_LUT_SIZE]; //= {0, 249, 297, 334, 370, 405, 420, 448, 489, 535, 583, 641, 710, 799, 897,  1029, 1204, 1474, 1905, 2000, 2000};
//static int16_t pwm_LUT_L[PWM_LUT_SIZE];// = {0, 292, 362, 422, 476, 521, 550, 585, 626, 671, 713, 784, 864, 974, 1109, 1293, 1525, 1895, 2000, 2000, 2000};

//                          0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31   32   33   34   35   36    37    38   39
static int16_t pwm_LUT_R[] = {0, 214, 235, 252, 267, 281, 295, 308, 320, 333, 346, 357, 370, 381, 393, 407, 423, 437, 452, 468, 483, 501, 520, 540, 561, 584, 609, 634, 663, 693, 719, 757, 797, 850, 892, 947, 1014, 1097, 1172, 1271};
static int16_t pwm_LUT_L[] = {0, 249, 277, 311, 342, 369, 394, 420, 444, 468, 490, 513, 532, 548, 568, 585, 606, 625, 643, 663, 642, 661, 687, 714, 739, 774, 808, 840, 876, 918, 984, 1021, 1111, 1171, 1192, 1251, 1351, 1578, 1671, 1824};

static int16_t motor_out[2];    // This is the array of PWM values being sent to the motors
static float balance_offset;

static int16_t body_velocity;
static int32_t nav_bearing;
static int16_t ground_speed;
//static int32_t ground_position;

static int16_t pitch_p;
static int16_t pitch_i;
static int16_t pitch_d;

static int16_t pitch_speed;
static int16_t yaw_speed;

static int16_t desired_speed;

static float wheel_ratio;
static float current_speed;
static float current_encoder_y;
static float current_encoder_x;
static uint32_t balance_timer;

static bool gps_available;
//static float throttle_pedal = 1;

////////////////////////////////////////////////////////////////////////////////
// Wheels
////////////////////////////////////////////////////////////////////////////////

// I2C address of wheel encoders
#define ENCODER_ADDRESS       0x29

static struct {
	int16_t left_distance;
	int16_t right_distance;
	int16_t left_speed;
	int16_t right_speed;
    int16_t left_speed_output;
    int16_t right_speed_output;
	int16_t speed;
} wheel;

// Receive buffer
static union {
    int32_t long_value;
    int16_t int_value;
    uint8_t bytes[];
} bytes_union;


// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Time in microseconds of 50hz control loop
static uint32_t fiftyhz_loopTimer;
// Counters for branching from 10 hz control loop
static byte medium_loopCounter;
// Counters for branching from 3 1/3hz control loop
static byte slow_loopCounter;
// Counters for branching at 1 hz
static byte counter_one_herz;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Delta Time in milliseconds for navigation computations, updated with every good GPS read
static float dTnav;
// Counters for branching from 4 minute control loop used to save Compass offsets
static int16_t superslow_loopCounter;
// Loiter timer - Records how long we have been in loiter
//static uint32_t rtl_loiter_start_time;
// disarms the copter while in Acro or Stabilize mode after 30 seconds of no flight
//static uint8_t auto_disarming_counter;
// prevents duplicate GPS messages from entering system
static uint32_t last_gps_time;

// Used to auto exit the roll_pitch_trim saving function
static uint8_t save_trim_counter;

// Reference to the AP relay object - APM1 only
AP_Relay relay;

// a pin for reading the receiver RSSI voltage. The scaling by 0.25
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_AnalogSource_Arduino RSSI_pin(-1, 0.25);

#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

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

#if CAMERA == ENABLED
//pinMode(camtrig, OUTPUT);			// these are free pins PE3(5), PH3(15), PH6(18), PB4(23), PB5(24), PL1(36), PL3(38), PA6(72), PA7(71), PK0(89), PK1(88), PK2(87), PK3(86), PK4(83), PK5(84), PK6(83), PK7(82)
#endif

////////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
#ifdef AP_LIMITS
AP_Limits               limits;
AP_Limit_GPSLock        gpslock_limit(g_gps);
AP_Limit_Geofence       geofence_limit(FENCE_START_BYTE, FENCE_WP_SIZE, MAX_FENCEPOINTS, g_gps, &home, &current_loc);
AP_Limit_Altitude       altitude_limit(&current_loc);
#endif

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup(){
    memcheck_init();
    init_ardupilot();
}

void loop()
{
    uint32_t timer = micros();
    uint16_t num_samples;

    // We want this to execute fast
    // ----------------------------
    num_samples = ins.num_samples_available();
    if(num_samples >= 4){

        #if DEBUG_FAST_LOOP == ENABLED
        Log_Write_Data(DATA_FAST_LOOP, (int32_t)(timer - fast_loopTimer));
        #endif


        //PORTK |= B00010000;
        G_Dt            = (float)(timer - fast_loopTimer) / 1000000.f;                  // used by PI Loops
        fast_loopTimer  = timer;

        //cliSerial->printf_P(PSTR("%1.4f, "),G_Dt);

        // for mainloop failure monitoring
        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // run the 50hz loop 1/2 the time
        //ap_system.run_50hz_loop = !ap_system.run_50hz_loop;

        //if(ap_system.run_50hz_loop) {
        if(true){

            #if DEBUG_MED_LOOP == ENABLED
            Log_Write_Data(DATA_MED_LOOP, (int32_t)(timer - fiftyhz_loopTimer));
            #endif

            // store the micros for the 50 hz timer
            fiftyhz_loopTimer = timer;

            // check for new GPS messages
            // --------------------------
            update_GPS();

            // run navigation routines
            update_navigation();

            // perform 10hz tasks
            // ------------------
            medium_loop();

            // Stuff to run at full 50hz, but after the med loops
            // --------------------------------------------------
            fifty_hz_loop();

            counter_one_herz++;

            // trgger our 1 hz loop
            if(counter_one_herz >= 50){
                super_slow_loop();
                counter_one_herz = 0;
            }
            perf_mon_counter++;

            if(perf_mon_counter > 600){
                if(g.log_bitmask & MASK_LOG_PM)
                    Log_Write_Performance();
                perf_info_reset();
                gps_fix_count           = 0;
                perf_mon_counter        = 0;
            }
        }
    }else{
#ifdef DESKTOP_BUILD
        usleep(1000);
#endif
        if(num_samples < NUM_IMU_SAMPLES_FOR_50HZ - 1){
            // we have some spare cycles available
            // less than 20ms has passed. We have at least one millisecond
            // of free time. The most useful thing to do with that time is
            // to accumulate some sensor readings, specifically the
            // compass, which is often very noisy but is not interrupt
            // driven, so it can't accumulate readings by itself
            if(g.compass_enabled){
                compass.accumulate();
            }
        }
    }

}

// Main loop - 50hz
static void fast_loop()
{
    // try to send any deferred messages if the serial port now has
    // some space available
    gcs_send_message(MSG_RETRY_DEFERRED);

    // read wheel encoders:
    // -------------------
    update_wheel_encoders();

    // Read radio
    // ----------
    read_radio();

    // Read 3-position switch on radio
    // -------------------------------
    read_control_switch();

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

    // Inertial Nav
    // --------------------
    read_inertia();

    // optical flow
    // --------------------
#if OPTFLOW == ENABLED
    if(g.optflow_enabled){
        update_optical_flow();
    }
#endif

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // write out the servo PWM values
    // ------------------------------
    update_servos();

    Log_Write_Attitude();

    // agmatthews - USERHOOKS
#ifdef USERHOOK_FASTLOOP
    USERHOOK_FASTLOOP
#endif

}

static void medium_loop()
{
    // This is the start of the medium (10 Hz) loop pieces
    // -----------------------------------------
    switch(medium_loopCounter){

    // This case deals with the GPS and Compass
    //-----------------------------------------
    case 0:
        medium_loopCounter++;

#if HIL_MODE != HIL_MODE_ATTITUDE                                                               // don't execute in HIL mode
        if(g.compass_enabled){
            if(compass.read()){
                compass.null_offsets();
            }
        }
#endif

        // save_trim - stores roll and pitch radio inputs to ahrs
        save_trim();

        // record throttle output
        // ------------------------------
        //throttle_integrator += g.rc_3.servo_out;
        break;

    // This case performs some navigation computations
    //------------------------------------------------
    case 1:
        medium_loopCounter++;
        read_receiver_rssi();
        break;

    // command processing
    //-------------------
    case 2:
        medium_loopCounter++;
        break;

    // This case deals with sending high rate telemetry
    //-------------------------------------------------
    case 3:
        medium_loopCounter++;

        // perform next command
        // --------------------
        if(control_mode == AUTO){
            if(ap.home_is_set && g.command_total > 1){
                update_commands();
            }
        }

        if(g.log_bitmask & MASK_LOG_ATTITUDE_MED){
        	//Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
            Log_Write_DMP();
#endif
		}

        if(g.log_bitmask & MASK_LOG_MOTORS)
        	Log_Write_Motors();
        break;

    // This case controls the slow loop
    //---------------------------------
    case 4:
        medium_loopCounter = 0;

        if(g.battery_monitoring != 0){
            read_battery();
        }

        // Accel trims      = hold > 2 seconds
        // Throttle cruise  = switch less than 1 second
        // --------------------------------------------
        read_trim_switch();

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
    // Read Sonar
    // ----------
# if CONFIG_SONAR == ENABLED
    if(g.sonar_enabled){
        //sonar_alt = sonar.read();
    }
#endif


#ifdef USERHOOK_50HZLOOP
    USERHOOK_50HZLOOP
#endif


#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update_mount_position();
#endif

#if MOUNT2 == ENABLED
    // update camera mount's position
    camera_mount2.update_mount_position();
#endif

#if CAMERA == ENABLED
    g.camera.trigger_pic_cleanup();
#endif

# if HIL_MODE == HIL_MODE_DISABLED
    if(g.log_bitmask & MASK_LOG_ATTITUDE_FAST){
        Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
        Log_Write_DMP();
#endif
    }

    if(g.log_bitmask & MASK_LOG_RAW)
        Log_Write_Raw();
#endif

    // kick the GCS to process uplink data
    gcs_update();
    gcs_data_stream_send();
}


static void slow_loop()
{

#if AP_LIMITS == ENABLED

    // Run the AP_Limits main loop
    limits_loop();

#endif // AP_LIMITS_ENABLED

    // This is the slow (3 1/3 Hz) loop pieces
    //----------------------------------------
    switch (slow_loopCounter){
    case 0:
        slow_loopCounter++;
        superslow_loopCounter++;

        // record if the compass is healthy
        set_compass_healthy(compass.healthy);

        if(superslow_loopCounter > 1200){
#if HIL_MODE != HIL_MODE_ATTITUDE
            if(g.rc_3.control_in == 0 && control_mode == STABILIZE && g.compass_enabled){
                compass.save_offsets();
                superslow_loopCounter = 0;
            }
#endif
        }

        if(g.log_bitmask & MASK_LOG_ITERM)
            Log_Write_Iterm();
        break;

    case 1:
        slow_loopCounter++;

#if MOUNT == ENABLED
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
        enable_aux_servos();

#if MOUNT == ENABLED
        camera_mount.update_mount_type();
#endif

#if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
#endif

        // agmatthews - USERHOOKS
#ifdef USERHOOK_SLOWLOOP
        USERHOOK_SLOWLOOP
#endif

        break;

    case 2:
        slow_loopCounter = 0;
        update_events();

        // blink if we are armed
        update_lights();

        if(g.radio_tuning > 0)
            tuning();

#if USB_MUX_PIN > 0
        check_usb_mux();
#endif
        break;

        default:
            slow_loopCounter = 0;
            break;
    }
}

#define AUTO_DISARMING_DELAY 25
// 1Hz loop
static void super_slow_loop()
{
    Log_Write_Data(DATA_AP_STATE, ap.value);

    if(g.log_bitmask & MASK_LOG_CUR)
        Log_Write_Current();

    gcs_send_message(MSG_HEARTBEAT);

    // agmatthews - USERHOOKS
#ifdef USERHOOK_SUPERSLOWLOOP
    USERHOOK_SUPERSLOWLOOP
#endif
}

// called at 100hz but data from sensor only arrives at 20 Hz
#if OPTFLOW == ENABLED
static void update_optical_flow(void)
{
    static uint32_t last_of_update = 0;
    static int log_counter = 0;

    // if new data has arrived, process it
    if(optflow.last_update != last_of_update){
        last_of_update = optflow.last_update;
        optflow.update_position(ahrs.roll, ahrs.pitch, cos_yaw_x, sin_yaw_y, current_loc.alt);      // updates internal lon and lat with estimation based on optical flow

        // write to log at 5hz
        log_counter++;
        if(log_counter >= 4){
            log_counter = 0;
            if(g.log_bitmask & MASK_LOG_OPTFLOW){
                Log_Write_Optflow();
            }
        }
    }
}
#endif  // OPTFLOW == ENABLED

// called at 50hz
static void update_GPS(void)
{
    // A counter that is used to grab at least 10 reads before commiting the Home location
    // static byte ground_start_count  = 10;

    g_gps->update();
    update_GPS_light();

    set_gps_healthy(g_gps->status() == g_gps->GPS_OK);

    if(g_gps->new_data && g_gps->fix){
        gps_available = true;

        // clear new data flag
        g_gps->new_data = false;

        // check for duiplicate GPS messages
        if(last_gps_time != g_gps->time){

            // for performance monitoring
            // --------------------------
            gps_fix_count++;

            if(ap.home_is_set == false){
                init_home();
                if(g.compass_enabled){
                    // Set compass declination automatically
                    compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                }
            }

            if(g.log_bitmask & MASK_LOG_GPS){
                Log_Write_GPS();
            }
        }

        // save GPS time so we don't get duplicate reads
        last_gps_time = g_gps->time;
    }
}

void update_yaw_mode(void)
{
    static bool yaw_flag = false;
	if(labs(ahrs.pitch_sensor) > 4000 || labs(ahrs.roll_sensor) > 4000){

        yaw_speed = 0;
        nav_yaw = ahrs.yaw_sensor;

        #if INERTIAL_NAV == ENABLED
        // clear our speed
        //accels_velocity.x     = 0;
        //accels_velocity.y     = 0;
        //accels_velocity.z     = 0;
        #endif

        return;
    }

    switch(yaw_mode){
        case YAW_ACRO:
            yaw_speed = g.rc_1.control_in;
            break;

        case YAW_HOLD:
            if(g.rc_1.control_in != 0){
                yaw_speed   = g.rc_1.control_in;
                yaw_flag    = true;
            }else{
                if(yaw_flag){
                    yaw_flag    = false;
                    nav_yaw     = ahrs.yaw_sensor;
                }else{
                    yaw_speed   = get_stabilize_yaw(nav_yaw);
                }
            }
            break;

        case YAW_LOOK_AT_NEXT_WP:
            nav_yaw = wp_bearing;
            yaw_speed = get_stabilize_yaw(nav_yaw);
            break;
    }
}

void update_roll_pitch_mode(void)
{
    if(labs(ahrs.pitch_sensor) > 4000){
        balance_timer = millis();
        //g.rc_2.servo_out = 0;
        tilt_start          = false;
        current_speed       = 0;
        nav_yaw  = ahrs.yaw_sensor;
        current_encoder_x = 0;
        current_encoder_y = 0;

        current_loc.lng	 = 0;
        current_loc.lat  = 0;

        next_WP.lng	 = 0;
        next_WP.lat  = 0;

        init_home();
        return;
    }

    if((millis() - balance_timer) < 3000){
        tilt_start  = false;
        pitch_speed = 0;
        yaw_speed   = 0;
        return;
    }else{
        tilt_start  = true;
    }



    //if(labs(ahrs.pitch_sensor) < 200 || labs(ahrs.roll_sensor) < 200){
    //    tilt_start = true;
    //}

    // init
    int16_t bal_out = 0;
    int16_t vel_out = 0;
    int16_t nav_out = 0;
    int16_t speed_error, ff_out, distance_error;


    switch(roll_pitch_mode){
        case ROLL_PITCH_STABLE:
            // we always hold position
            if(abs(g.rc_2.control_in) > 0){
                // reset position
                next_WP.lat = current_loc.lat;
                next_WP.lng = current_loc.lng;
                g.pid_nav.reset_I();
            }

            // in this mode we command the target angle
            bal_out = get_stabilize_pitch(g.rc_2.control_in); // neg = pitch forward

            // speed control:
            vel_out = get_velocity_pitch();

            // maintain location:
            nav_out = get_nav_pitch(0, get_dist_err());

            pitch_speed = (bal_out + vel_out + nav_out);

           	/*cliSerial->printf_P(PSTR("a:%d\td:%d\tbal%d, vel%d, nav%d\n"),
           	        (int16_t)ahrs.pitch_sensor,
                   	(int16_t)wp_distance,
                   	bal_out,
                   	vel_out,
                   	nav_out);*/



            break;

        case ROLL_PITCH_FBW:
            // hold position if we let go of sticks
            if(abs(g.rc_2.control_in) > 0){
                // reset position
                next_WP.lat = current_loc.lat;
                next_WP.lng = current_loc.lng;
                g.pid_nav.reset_I();
            }

            distance_error		= (float)long_error * cos_yaw_x + (float)lat_error * sin_yaw_y;

            // defaulting to 500 / 12 = 41cm/s = 1.5r/s = 1200e/s
            if(g.rc_2.control_in == 0){
                desired_speed  = distance_error;
            }else{
                desired_speed   = -g.rc_2.control_in / g.fbw_speed;             // units = cm/s
                desired_speed 	= constrain(desired_speed, -80, 80);            // units = cm/s
            }

            // switching units to ticks
            desired_speed   = convert_distance_to_encoder_speed(desired_speed); // units = ticks/second : 1RPM = 1000ticks/second
            speed_error     = wheel.speed - desired_speed;                      // units = ticks/second : 1RPM = 1000ticks/second

            // 4 components of stability and navigation
            bal_out         = get_stabilize_pitch(0);                           // hold as vertical as possible
            vel_out         = get_velocity_pitch();                             // magic
            ff_out          = (float)desired_speed * g.throttle;                // allows us to roll while vertical
            nav_out      	= g.pid_nav.get_pid(speed_error, G_Dt);             // allows us to accelerate

            cliSerial->printf_P(PSTR("%d, %d, %d, %d, %d, %d, %d, %d\n"),
                (int16_t)ahrs.pitch_sensor,
                (int16_t)balance_offset,
                bal_out,
                vel_out,
                ff_out,
                nav_out,
                desired_speed,
                speed_error);//*/

            // sum the output
            pitch_speed = (bal_out + vel_out + nav_out - ff_out);
        break;

        case ROLL_PITCH_AUTO:
            // in this mode we command the target angle
            pitch_speed = get_stabilize_pitch(0); // neg = pitch forward

            // speed control:
            pitch_speed += get_velocity_pitch();

            // maintain location:
            if(wp_control == LOITER_MODE){
                pitch_speed += get_nav_pitch(0, get_dist_err());
            }else{
                pitch_speed += get_nav_pitch(500, get_dist_err()); // minimum speed for WP nav
            }
            break;


    }
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_update();
#endif

    ahrs.update();
    omega = ins.get_gyro();

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.update();
#endif
}

static void update_trig(void){
    Vector2f yawvector;
    Matrix3f temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x;     // sin
    yawvector.y     = temp.b.x;         // cos
    yawvector.normalize();

    cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
    cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x     = constrain(cos_pitch_x, 0, 1.0);
    // this relies on constrain() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x      = constrain(cos_roll_x, -1.0, 1.0);

    sin_yaw_y       = yawvector.x;                                              // 1y = north
    cos_yaw_x       = yawvector.y;                                              // 0x = north

    // added to convert earth frame to body frame for rate controllers
    sin_pitch       = -temp.c.x;
    sin_roll        = temp.c.y / cos_pitch_x;

    //flat:
    // 0 ° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 90° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 180 = cos_yaw:  0.00, sin_yaw: -1.00,
    // 270 = cos_yaw: -1.00, sin_yaw:  0.00,
}

static void tuning(){
    tuning_value = (float)g.rc_6.control_in / 1000.0;
    g.rc_6.set_range(g.radio_tuning_low, g.radio_tuning_high);

    switch(g.radio_tuning){
        case CH6_RATE_KP:
            g.pid_balance.kP(tuning_value);
            break;

        case CH6_RATE_KI:
            g.pid_balance.kI(tuning_value);
            break;

        case CH6_RATE_KD:
            g.pid_balance.kD(tuning_value);
            break;

        case CH6_YAW_RATE_KP:
            g.pid_yaw.kP(tuning_value);
            break;

        case CH6_YAW_RATE_KI:
            g.pid_yaw.kI(tuning_value);
            break;

        case CH6_YAW_RATE_KD:
            g.pid_yaw.kD(tuning_value);
            break;

        case CH6_NAV_RATE_KP:
            g.pid_nav.kP(tuning_value);
            break;
    }
}



