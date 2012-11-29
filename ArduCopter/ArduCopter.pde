/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V2.8.1a"
/*
 *  ArduCopter Version 2.8
 *  Lead author:	Jason Short
 *  Based on code and ideas from the Arducopter team: Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen, Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 *  Thanks to:	Chris Anderson, Mike Smith, Jordi Munoz, Doug Weibel, James Goppert, Benjamin Pelletier, Robert Lefebvre, Marco Robustini
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  Special Thanks for Contributors:
 *
 *  Hein Hollander      :Octo Support
 *  Dani Saez           :V Ocoto Support
 *  Max Levine			:Tri Support, Graphics
 *  Jose Julio			:Stabilization Control laws
 *  Randy MacKay		:Heli Support
 *  Jani Hiriven		:Testing feedback
 *  Andrew Tridgell		:Mavlink Support
 *  James Goppert		:Mavlink Support
 *  Doug Weibel			:Libraries
 *  Mike Smith			:Libraries, Coding support
 *  HappyKillmore		:Mavlink GCS
 *  Michael Oborne		:Mavlink GCS
 *  Jack Dunkle			:Alpha testing
 *  Christof Schmid		:Alpha testing
 *  Oliver				:Piezo support
 *  Guntars				:Arming safety suggestion
 *  Igor van Airde      :Control Law optimization
 *  Jean-Louis Naudin   :Auto Landing
 *  Sandro Benigno      :Camera support
 *  Olivier Adler       :PPM Encoder
 *  John Arne Birkeland	:PPM Encoder
 *  Adam M Rivera		:Auto Compass Declination
 *  Marco Robustini		:Alpha testing
 *  Angel Fernandez		:Alpha testing
 *  Robert Lefebvre		:Heli Support & LEDs
 *  Amilcar Lucas		:mount and camera configuration
 *  Gregory Fletcher	:mount orientation math
 *	Leonard Hall 		:Flight Dynamics
 *
 *  And much more so PLEASE PM me on DIYDRONES to add your contribution to the List
 *
 *  Requires modified "mrelax" version of Arduino, which can be found here:
 *  http://code.google.com/p/ardupilot-mega/downloads/list
 *
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
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_MotorsQuad.h>      // AP Motors library for Quad
#include <AP_MotorsTri.h>       // AP Motors library for Tri
#include <AP_MotorsHexa.h>      // AP Motors library for Hexa
#include <AP_MotorsY6.h>        // AP Motors library for Y6
#include <AP_MotorsOcta.h>      // AP Motors library for Octa
#include <AP_MotorsOctaQuad.h>  // AP Motors library for OctaQuad
#include <AP_MotorsHeli.h>      // AP Motors library for Heli
#include <AP_MotorsMatrix.h>    // AP Motors library for Heli
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <ModeFilter.h>         // Mode Filter from Filter library
#include <AverageFilter.h>      // Mode Filter from Filter library
#include <AP_LeadFilter.h>      // GPS Lead filter
#include <LowPassFilter.h>      // Low Pass Filter library
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <ThirdOrderCompFilter.h>   // Complementary filter for combining barometer altitude with accelerometers
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
FastSerialPort0(Serial);        // FTDI/console
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
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

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
static AP_Int8                *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

// real sensors
 #if CONFIG_ADC == ENABLED
AP_ADC_ADS7844 adc;
 #endif

 #ifdef DESKTOP_BUILD
AP_Baro_BMP085_HIL barometer;
AP_Compass_HIL compass;
  #include <SITL.h>
SITL sitl;
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
AP_Baro_BMP085_HIL      barometer;
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
AP_Baro_BMP085_HIL      barometer;

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
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set        : 1; // 1
        uint8_t simple_mode        : 1; // 2    // This is the state of simple mode
        uint8_t manual_attitude    : 1; // 3
        uint8_t manual_throttle    : 1; // 4

        uint8_t low_battery        : 1; // 5    // Used to track if the battery is low - LED output flashes when the batt is low
        uint8_t loiter_override    : 1; // 6    // Are we navigating while holding a positon? This is set to false once the speed drops below 1m/s
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
static int16_t lon_speed;       // expressed in cm/s.  positive numbers mean moving east
static int16_t lat_speed;       // expressed in cm/s.  positive numbers when moving north
static int16_t ground_bearing;	// expressed in centidegrees

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
// Motor Output
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsQuad
#endif
#if FRAME_CONFIG == TRI_FRAME
 #define MOTOR_CLASS AP_MotorsTri
#endif
#if FRAME_CONFIG == HEXA_FRAME
 #define MOTOR_CLASS AP_MotorsHexa
#endif
#if FRAME_CONFIG == Y6_FRAME
 #define MOTOR_CLASS AP_MotorsY6
#endif
#if FRAME_CONFIG == OCTA_FRAME
 #define MOTOR_CLASS AP_MotorsOcta
#endif
#if FRAME_CONFIG == OCTA_QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsOctaQuad
#endif
#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#endif

#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
MOTOR_CLASS motors(CONFIG_APM_HARDWARE, &APM_RC, &g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_8, &g.heli_servo_1, &g.heli_servo_2, &g.heli_servo_3, &g.heli_servo_4);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
MOTOR_CLASS motors(CONFIG_APM_HARDWARE, &APM_RC, &g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_7);
#else
MOTOR_CLASS motors(CONFIG_APM_HARDWARE, &APM_RC, &g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);
#endif

////////////////////////////////////////////////////////////////////////////////
// Mavlink/HIL control
////////////////////////////////////////////////////////////////////////////////
// Used to track the GCS based control input
// Allow override of RC channel values for HIL
static int16_t rc_override[8] = {0,0,0,0,0,0,0,0};
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
float tuning_value;

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
static const float gravity = 9.80665;           // meters/ sec^2

// Unions for getting byte values
union float_int {
    int32_t int_value;
    float float_value;
} float_int;


////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t target_bearing;
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
static uint8_t rtl_state;

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
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static int32_t initial_simple_bearing;

////////////////////////////////////////////////////////////////////////////////
// Rate contoller targets
////////////////////////////////////////////////////////////////////////////////
static uint8_t rate_targets_frame = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
static int32_t roll_rate_target_ef = 0;
static int32_t pitch_rate_target_ef = 0;
static int32_t yaw_rate_target_ef = 0;
static int32_t roll_rate_target_bf = 0;     // body frame roll rate target
static int32_t pitch_rate_target_bf = 0;    // body frame pitch rate target
static int32_t yaw_rate_target_bf = 0;      // body frame yaw rate target

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static int16_t throttle_accel_target_ef = 0;    // earth frame throttle acceleration target
static bool throttle_accel_controller_active = false;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
static float z_accel_meas = 0;              // filtered throttle acceleration
static float throttle_avg;                  // g.throttle_cruise as a float
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only


////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
// Used to control Axis lock
int32_t roll_axis;
int32_t pitch_axis;

// Filters
AP_LeadFilter xLeadFilter;      // Long GPS lag filter
AP_LeadFilter yLeadFilter;      // Lat  GPS lag filter
#if FRAME_CONFIG == HELI_FRAME
LowPassFilterFloat rate_roll_filter;    // Rate Roll filter
LowPassFilterFloat rate_pitch_filter;   // Rate Pitch filter
// LowPassFilterFloat rate_yaw_filter;     // Rate Yaw filter
#endif // HELI_FRAME

// Barometer filter
AverageFilterInt32_Size5 baro_filter;

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
static int32_t old_target_bearing;
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
// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
static int32_t altitude_error;
// The cm/s we are moving up or down based on sensor data - Positive = UP
static int16_t climb_rate_actual;
// Used to dither our climb_rate over 50hz
static int16_t climb_rate_error;
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t sonar_alt;
// The climb_rate as reported by sonar in cm/s
static int16_t sonar_rate;
// The altitude as reported by Baro in cm – Values can be quite high
static int32_t baro_alt;
// The climb_rate as reported by Baro in cm/s
static int16_t baro_rate;

static int16_t saved_toy_throttle;


////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static byte yaw_mode;
// The current desired control scheme for roll and pitch / navigation
static byte roll_pitch_mode;
// The current desired control scheme for altitude hold
static byte throttle_mode;


////////////////////////////////////////////////////////////////////////////////
// flight specific
////////////////////////////////////////////////////////////////////////////////
// An additional throttle added to keep the copter at the same altitude when banking
static int16_t angle_boost;
// counter to verify landings
static uint16_t land_detector;


////////////////////////////////////////////////////////////////////////////////
// Navigation general
////////////////////////////////////////////////////////////////////////////////
// The location of the copter in relation to home, updated every GPS read
static int32_t home_to_copter_bearing;
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
static int32_t original_target_bearing;
// The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;


////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// all angles are deg * 100 : target yaw angle
// The Commanded ROll from the autopilot.
static int32_t nav_roll;
// The Commanded pitch from the autopilot. negative Pitch means go forward.
static int32_t nav_pitch;
// The desired bank towards North (Positive) or South (Negative)
static int32_t auto_roll;
static int32_t auto_pitch;

// Don't be fooled by the fact that Pitch is reversed from Roll in its sign!
static int16_t nav_lat;
// The desired bank towards East (Positive) or West (Negative)
static int16_t nav_lon;
// The Commanded ROll from the autopilot based on optical flow sensor.
static int32_t of_roll;
// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
static int32_t of_pitch;


////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Throttle from the autopilot.
static int16_t nav_throttle;    // 0-1000 for throttle control
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;

////////////////////////////////////////////////////////////////////////////////
// Climb rate control
////////////////////////////////////////////////////////////////////////////////
// Time when we intiated command in millis - used for controlling decent rate
// Used to track the altitude offset for climbrate control
static int8_t alt_change_flag;

////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t nav_yaw;
// A speed governer for Yaw control - limits the rate the quad can be turned by the autopilot
static int32_t auto_yaw;
static uint8_t yaw_timer;
// Options include: no tracking, point at next wp, or at a target
static byte auto_yaw_tracking = MAV_ROI_WPNEXT;
// In AP Mission scripting we have a fine level of control for Yaw
// This is our initial angle for relative Yaw movements
static int32_t command_yaw_start;
// Timer values used to control the speed of Yaw movements
static uint32_t command_yaw_start_time;
static uint16_t command_yaw_time;                                       // how long we are turning
static int32_t command_yaw_end;                                         // what angle are we trying to be
// how many degrees will we turn
static int32_t command_yaw_delta;
// Deg/s we should turn
static int16_t command_yaw_speed;
// Direction we will turn –  1 = CW, 0 or -1 = CCW
static byte command_yaw_dir;
// Direction we will turn – 1 = relative, 0 = Absolute
static byte command_yaw_relative;
// Yaw will point at this location if auto_yaw_tracking is set to MAV_ROI_LOCATION
static struct   Location target_WP;



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
AP_InertialNav inertial_nav(&ahrs, &ins, &barometer, &g_gps);
#endif

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Used to manage the rate of performance logging messages
static int16_t perf_mon_counter;
// The number of GPS fixes we have had
static int16_t gps_fix_count;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Time in microseconds of 50hz control loop
static uint32_t fiftyhz_loopTimer = 0;
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
static uint32_t rtl_loiter_start_time;
// disarms the copter while in Acro or Stabilize mode after 30 seconds of no flight
static uint8_t auto_disarming_counter;
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
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
void get_throttle_althold(int32_t target_alt, int16_t max_climb_rate = ALTHOLD_MAX_CLIMB_RATE);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

void setup() {
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
    if (num_samples >= 1) {

        #if DEBUG_FAST_LOOP == ENABLED
        Log_Write_Data(DATA_FAST_LOOP, (int32_t)(timer - fast_loopTimer));
        #endif

        // check loop time
        perf_info_check_loop_time(timer - fast_loopTimer);

        G_Dt                            = (float)(timer - fast_loopTimer) / 1000000.f;                  // used by PI Loops
        fast_loopTimer          = timer;

        // for mainloop failure monitoring
        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // run the 50hz loop 1/2 the time
        ap_system.run_50hz_loop = !ap_system.run_50hz_loop;

        if(ap_system.run_50hz_loop) {

            #if DEBUG_MED_LOOP == ENABLED
            Log_Write_Data(DATA_MED_LOOP, (int32_t)(timer - fiftyhz_loopTimer));
            #endif

            // store the micros for the 50 hz timer
            fiftyhz_loopTimer               = timer;

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
            if(counter_one_herz >= 50) {
                super_slow_loop();
                counter_one_herz = 0;
            }
            perf_mon_counter++;
            if (perf_mon_counter >= 500 ) {     // 500 iterations at 50hz = 10 seconds
                if (g.log_bitmask & MASK_LOG_PM)
                    Log_Write_Performance();
                perf_info_reset();
                gps_fix_count           = 0;
                perf_mon_counter        = 0;
            }
        }
    } else {
#ifdef DESKTOP_BUILD
        usleep(1000);
#endif
        if (timer - fast_loopTimer < 9) {
            // we have some spare cycles available
            // less than 10ms has passed. We have at least one millisecond
            // of free time. The most useful thing to do with that time is
            // to accumulate some sensor readings, specifically the
            // compass, which is often very noisy but is not interrupt
            // driven, so it can't accumulate readings by itself
            if (g.compass_enabled) {
                compass.accumulate();
            }
        }
    }

}

// Main loop - 100hz
static void fast_loop()
{
    // try to send any deferred messages if the serial port now has
    // some space available
    gcs_send_message(MSG_RETRY_DEFERRED);

    // run low level rate controllers that only require IMU data
    run_rate_controllers();

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

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
    if(g.optflow_enabled) {
        update_optical_flow();
    }
#endif  // OPTFLOW == ENABLED

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // update targets to rate controllers
    update_rate_contoller_targets();

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

#if HIL_MODE != HIL_MODE_ATTITUDE                                                               // don't execute in HIL mode
        if(g.compass_enabled) {
            if (compass.read()) {
                compass.null_offsets();
            }
        }
#endif

        // save_trim - stores roll and pitch radio inputs to ahrs
        save_trim();

        // record throttle output
        // ------------------------------
        throttle_integrator += g.rc_3.servo_out;
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

        if(control_mode == TOY_A) {
            update_toy_throttle();

            if(throttle_mode == THROTTLE_AUTO) {
                update_toy_altitude();
            }
        }

        ap_system.alt_sensor_flag = true;
        break;

    // This case deals with sending high rate telemetry
    //-------------------------------------------------
    case 3:
        medium_loopCounter++;

        // perform next command
        // --------------------
        if(control_mode == AUTO) {
            if(ap.home_is_set && g.command_total > 1) {
                update_commands();
            }
        }

        if(motors.armed()) {
            if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) {
                Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
                Log_Write_DMP();
#endif
            }

            if (g.log_bitmask & MASK_LOG_MOTORS)
                Log_Write_Motors();
        }
        break;

    // This case controls the slow loop
    //---------------------------------
    case 4:
        medium_loopCounter = 0;

        if (g.battery_monitoring != 0) {
            read_battery();
        }

        // Accel trims      = hold > 2 seconds
        // Throttle cruise  = switch less than 1 second
        // --------------------------------------------
        read_trim_switch();

        // Check for engine arming
        // -----------------------
        arm_motors();

        // agmatthews - USERHOOKS
#ifdef USERHOOK_MEDIUMLOOP
        USERHOOK_MEDIUMLOOP
#endif

#if COPTER_LEDS == ENABLED
        update_copter_leds();
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
    // read altitude sensors or estimate altitude
    // ------------------------------------------
    update_altitude_est();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

    // Read Sonar
    // ----------
# if CONFIG_SONAR == ENABLED
    if(g.sonar_enabled) {
        sonar_alt = sonar.read();
    }
#endif

#if TOY_EDF == ENABLED
    edf_toy();
#endif

#ifdef USERHOOK_50HZLOOP
    USERHOOK_50HZLOOP
#endif


#if HIL_MODE != HIL_MODE_DISABLED && FRAME_CONFIG != HELI_FRAME
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
    if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST && motors.armed()) {
        Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
        Log_Write_DMP();
#endif
    }

    if (g.log_bitmask & MASK_LOG_RAW && motors.armed())
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
    switch (slow_loopCounter) {
    case 0:
        slow_loopCounter++;
        superslow_loopCounter++;

        // record if the compass is healthy
        set_compass_healthy(compass.healthy);

        if(superslow_loopCounter > 1200) {
#if HIL_MODE != HIL_MODE_ATTITUDE
            if(g.rc_3.control_in == 0 && control_mode == STABILIZE && g.compass_enabled) {
                compass.save_offsets();
                superslow_loopCounter = 0;
            }
#endif
        }


        if(motors.armed()) {
            if (g.log_bitmask & MASK_LOG_ITERM)
                Log_Write_Iterm();
        }else{
            // check the user hasn't updated the frame orientation
            motors.set_frame_orientation(g.frame_orientation);
        }

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

    if (g.log_bitmask & MASK_LOG_CUR && motors.armed())
        Log_Write_Current();

    // this function disarms the copter if it has been sitting on the ground for any moment of time greater than 25 seconds
    // but only of the control mode is manual
    if((control_mode <= ACRO) && (g.rc_3.control_in == 0)) {
        auto_disarming_counter++;

        if(auto_disarming_counter == AUTO_DISARMING_DELAY) {
            init_disarm_motors();
        }else if (auto_disarming_counter > AUTO_DISARMING_DELAY) {
            auto_disarming_counter = AUTO_DISARMING_DELAY + 1;
        }
    }else{
        auto_disarming_counter = 0;
    }

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
    if( optflow.last_update != last_of_update ) {
        last_of_update = optflow.last_update;
        optflow.update_position(ahrs.roll, ahrs.pitch, cos_yaw_x, sin_yaw_y, current_loc.alt);      // updates internal lon and lat with estimation based on optical flow

        // write to log at 5hz
        log_counter++;
        if( log_counter >= 4 ) {
            log_counter = 0;
            if (g.log_bitmask & MASK_LOG_OPTFLOW) {
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
    static byte ground_start_count  = 10;

    g_gps->update();
    update_GPS_light();

    set_gps_healthy(g_gps->status() == g_gps->GPS_OK);

    if (g_gps->new_data && g_gps->fix) {
        // clear new data flag
        g_gps->new_data = false;

        // check for duiplicate GPS messages
        if(last_gps_time != g_gps->time) {

            // for performance monitoring
            // --------------------------
            gps_fix_count++;

            if(ground_start_count > 1) {
                ground_start_count--;

            } else if (ground_start_count == 1) {

                // We countdown N number of good GPS fixes
                // so that the altitude is more accurate
                // -------------------------------------
                if (current_loc.lat == 0) {
                    ground_start_count = 5;

                }else{
                    if (g.compass_enabled) {
                        // Set compass declination automatically
                        compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                    }
                    // save home to eeprom (we must have a good fix to have reached this point)
                    init_home();
                    ground_start_count = 0;
                }
            }

            if (g.log_bitmask & MASK_LOG_GPS && motors.armed()) {
                Log_Write_GPS();
            }

#if HIL_MODE == HIL_MODE_ATTITUDE                                                               // only execute in HIL mode
            //update_altitude();
            ap_system.alt_sensor_flag = true;
#endif
        }

        // save GPS time so we don't get duplicate reads
        last_gps_time = g_gps->time;
    }
}

void update_yaw_mode(void)
{
    switch(yaw_mode) {
    case YAW_ACRO:
        if(g.axis_enabled) {
            get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        }else{
            get_acro_yaw(g.rc_4.control_in);
        }
        return;
        break;

        // update to allow external roll/yaw mixing
#if TOY_LOOKUP == TOY_EXTERNAL_MIXER
    case YAW_TOY:
#endif

    case YAW_HOLD:
    	get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        break;

    case YAW_LOOK_AT_HOME:
        //nav_yaw updated in update_navigation()
        get_stabilize_yaw(nav_yaw);
        break;

    case YAW_AUTO:
        nav_yaw += constrain(wrap_180(auto_yaw - nav_yaw), -60, 60);                 // 40 deg a second
        //cliSerial->printf("nav_yaw %d ", nav_yaw);
        nav_yaw  = wrap_360(nav_yaw);
        get_stabilize_yaw(nav_yaw);
        break;
		
	case YAW_LOOK_AHEAD:
		
		// Commanded Yaw to automatically look ahead.
		if (g_gps->ground_speed > YAW_LOOK_AHEAD_RATE){									// Speed in cm/s.
			nav_yaw += constrain(wrap_180(ground_bearing - nav_yaw), -60, 60);    		// 40 deg a second
			nav_yaw  = wrap_360(nav_yaw);
			get_stabilize_yaw(nav_yaw + g.rc_4.control_in);								// Allow pilot to "skid" around corners up to 45°
        } else { 
			nav_yaw += g.rc_4.control_in * g.acro_p * G_Dt;
			nav_yaw = wrap_360(nav_yaw);
			get_stabilize_yaw(nav_yaw);
		}
        break;		
    }
}

void update_roll_pitch_mode(void)
{
    if (ap.do_flip) {
        if(abs(g.rc_1.control_in) < 4000) {
            roll_flip();
            return;
        }else{
            // force an exit from the loop if we are not hands off sticks.
            ap.do_flip = false;
            Log_Write_Event(DATA_EXIT_FLIP);
        }
    }

    switch(roll_pitch_mode) {
    case ROLL_PITCH_ACRO:    

#if FRAME_CONFIG == HELI_FRAME
		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            if (motors.flybar_mode == 1) {
                g.rc_1.servo_out = g.rc_1.control_in;
                g.rc_2.servo_out = g.rc_2.control_in;
            } else {
                get_acro_roll(g.rc_1.control_in);
                get_acro_pitch(g.rc_2.control_in);
            }
		}
#else  // !HELI_FRAME
		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            get_acro_roll(g.rc_1.control_in);
            get_acro_pitch(g.rc_2.control_in);
		}
#endif  // HELI_FRAME
        break;

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }
        // mix in user control with Nav control
        nav_roll                += constrain(wrap_180(auto_roll  - nav_roll),  -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second
        nav_pitch               += constrain(wrap_180(auto_pitch - nav_pitch), -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second

        control_roll            = g.rc_1.control_mix(nav_roll);
        control_pitch           = g.rc_2.control_mix(nav_pitch);

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_STABLE_OF:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // mix in user control with optical flow
        get_stabilize_roll(get_of_roll(control_roll));
        get_stabilize_pitch(get_of_pitch(control_pitch));
        break;

    // THOR
    // a call out to the main toy logic
    case ROLL_PITCH_TOY:
        roll_pitch_toy();
        break;
    }
	
	#if FRAME_CONFIG != HELI_FRAME
    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
        reset_stability_I();
    }
	#endif //HELI_FRAME

    if(ap_system.new_radio_frame) {
        // clear new radio frame info
        ap_system.new_radio_frame = false;
    }
}

// new radio frame is used to make sure we only call this at 50hz
void update_simple_mode(void)
{
    static byte simple_counter = 0;             // State machine counter for Simple Mode
    static float simple_sin_y=0, simple_cos_x=0;

    // used to manage state machine
    // which improves speed of function
    simple_counter++;

    int16_t delta = wrap_360(ahrs.yaw_sensor - initial_simple_bearing)/100;

    if (simple_counter == 1) {
        // roll
        simple_cos_x = sin(radians(90 - delta));

    }else if (simple_counter > 2) {
        // pitch
        simple_sin_y = cos(radians(90 - delta));
        simple_counter = 0;
    }

    // Rotate input by the initial bearing
    int16_t _roll   = g.rc_1.control_in * simple_cos_x + g.rc_2.control_in * simple_sin_y;
    int16_t _pitch  = -(g.rc_1.control_in * simple_sin_y - g.rc_2.control_in * simple_cos_x);

    g.rc_1.control_in = _roll;
    g.rc_2.control_in = _pitch;
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
            throttle_accel_controller_active = false;   // this controller does not use accel based throttle controller
            throttle_initialised = true;
            break;

        case THROTTLE_ACCELERATION:
        case THROTTLE_RATE:
        case THROTTLE_STABILIZED_RATE:
        case THROTTLE_DIRECT_ALT:
            throttle_accel_controller_active = true;    // this controller uses accel based throttle controller
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            set_new_altitude(current_loc.alt);          // by default hold the current altitude
            if ( throttle_mode < THROTTLE_HOLD ) {      // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
            }
            throttle_accel_controller_active = true;    // this controller uses accel based throttle controller
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            set_land_complete(false);   // mark landing as incomplete
            land_detector = 0;          // A counter that goes up if our climb rate stalls out.
            set_new_altitude(0);        // Set a new target altitude
            throttle_accel_controller_active = true;    // this controller uses accel based throttle controller
            throttle_initialised = true;
            break;

        default:
            cliSerial->printf_P(PSTR("Unsupported throttle mode: %d!!"),new_throttle_mode);
            break;
    }

    // update the throttle mode
    if( throttle_initialised ) {
        throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        desired_climb_rate = 0;
        nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}

// 50 hz update rate
// controls all throttle behavior
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;

    if(ap.do_flip)     // this is pretty bad but needed to flip in AP modes.
        return;

    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
	if (roll_pitch_mode == ROLL_PITCH_STABLE){
		motors.stab_throttle = true;
	} else {
		motors.stab_throttle = false;
	}
#endif // HELI_FRAME

    switch(throttle_mode) {

    case THROTTLE_MANUAL:
        // completely manual throttle
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            // send pilot's output directly to motors
            set_throttle_out(g.rc_3.control_in, false);

            // update estimate of throttle cruise
            update_throttle_cruise(g.rc_3.control_in);

            // check if we've taken off yet
            if (!ap.takeoff_complete && motors.armed()) {
                if (g.rc_3.control_in > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        break;

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            set_throttle_out(g.rc_3.control_in, true);

            // update estimate of throttle cruise
            update_throttle_cruise(g.rc_3.control_in);

            if (!ap.takeoff_complete && motors.armed()) {
                if (g.rc_3.control_in > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        break;

    case THROTTLE_ACCELERATION:
        // pilot inputs the desired acceleration
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            int16_t desired_acceleration = get_pilot_desired_acceleration(g.rc_3.control_in);
            set_throttle_accel_target(desired_acceleration);
        }
        break;

    case THROTTLE_RATE:
        // pilot inputs the desired climb rate.  Note this is the unstabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate(pilot_climb_rate);
        }
        break;

    case THROTTLE_STABILIZED_RATE:
        // pilot inputs the desired climb rate.  Note this is the unstabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate_stabilized(pilot_climb_rate);
        }
        break;

    case THROTTLE_DIRECT_ALT:
        // pilot inputs a desired altitude from 0 ~ 10 meters
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            // To-Do: this should update the global desired altitude variable next_WP.alt
            int32_t desired_alt = get_pilot_desired_direct_alt(g.rc_3.control_in);
            get_throttle_althold(desired_alt);
        }
        break;

    case THROTTLE_HOLD:
        // alt hold plus pilot input of climb rate
        pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
        get_throttle_rate_stabilized(pilot_climb_rate);
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in next_WP.alt
        if(motors.auto_armed() == true) {
            get_throttle_althold(next_WP.alt);
            // TO-DO: need to somehow set nav_throttle
        }
        // TO-DO: what if auto_armed is not true?!  throttle stuck at unknown position?
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
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
    cos_roll_x          = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x = constrain(cos_pitch_x, 0, 1.0);
    // this relies on constrain() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x  = constrain(cos_roll_x, -1.0, 1.0);

    sin_yaw_y               = yawvector.x;                                              // 1y = north
    cos_yaw_x               = yawvector.y;                                              // 0x = north

    // added to convert earth frame to body frame for rate controllers
    sin_pitch = -temp.c.x;
    sin_roll = temp.c.y / cos_pitch_x;

    //flat:
    // 0 ° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 90° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 180 = cos_yaw:  0.00, sin_yaw: -1.00,
    // 270 = cos_yaw: -1.00, sin_yaw:  0.00,
}

// updated at 10hz
static void update_altitude()
{
    static int16_t old_sonar_alt   = 0;
    static int32_t old_baro_alt    = 0;

#if HIL_MODE == HIL_MODE_ATTITUDE
    // we are in the SIM, fake out the baro and Sonar
    int16_t fake_relative_alt = g_gps->altitude - gps_base_alt;
    baro_alt                = fake_relative_alt;
    sonar_alt               = fake_relative_alt;

    baro_rate               = (baro_alt - old_baro_alt) * 5;             // 5hz
    old_baro_alt            = baro_alt;

#else
    // This is real life

    // read in Actual Baro Altitude
    baro_alt                        = read_barometer();

    // calc the vertical accel rate

     // 2.6 way
     int16_t temp		= (baro_alt - old_baro_alt) * 10;
     baro_rate          = (temp + baro_rate) >> 1;
     baro_rate			= constrain(baro_rate, -500, 500);
     old_baro_alt		= baro_alt;

    // Using Tridge's new clamb rate calc
    /*
    int16_t temp                    = barometer.get_climb_rate() * 100;
    baro_rate                       = (temp + baro_rate) >> 1;
    baro_rate                       = constrain(baro_rate, -300, 300);
    */

    // Note: sonar_alt is calculated in a faster loop and filtered with a mode filter
#endif

    if(g.sonar_enabled) {
        // filter out offset
        float scale;

        // calc rate of change for Sonar
#if HIL_MODE == HIL_MODE_ATTITUDE
        // we are in the SIM, fake outthe Sonar rate
        sonar_rate              = baro_rate;
#else
        // This is real life
        // calc the vertical accel rate
        // positive = going up.
        sonar_rate              = (sonar_alt - old_sonar_alt) * 10;
        sonar_rate              = constrain(sonar_rate, -150, 150);
        old_sonar_alt   = sonar_alt;
#endif

        if(baro_alt < 800) {
#if SONAR_TILT_CORRECTION == 1
            // correct alt for angle of the sonar
            float temp = cos_pitch_x * cos_roll_x;
            temp = max(temp, 0.707);
            sonar_alt = (float)sonar_alt * temp;
#endif

            scale = (float)(sonar_alt - 400) / 200.0;
            scale = constrain(scale, 0.0, 1.0);
            // solve for a blended altitude
            current_loc.alt = ((float)sonar_alt * (1.0 - scale)) + ((float)baro_alt * scale);

            // solve for a blended climb_rate
            climb_rate_actual = ((float)sonar_rate * (1.0 - scale)) + (float)baro_rate * scale;

        }else{
            // we must be higher than sonar (>800), don't get tricked by bad sonar reads
            current_loc.alt = baro_alt;
            // dont blend, go straight baro

            climb_rate_actual       = baro_rate;
        }

    }else{
        // NO Sonar case
        current_loc.alt = baro_alt;
        climb_rate_actual = baro_rate;
    }

    // update the target altitude
    verify_altitude();

    // calc error
    climb_rate_error = (climb_rate_actual - climb_rate) / 5;
}

static void update_altitude_est()
{
    if(ap_system.alt_sensor_flag) {
        update_altitude();
        ap_system.alt_sensor_flag = false;

        if(g.log_bitmask & MASK_LOG_CTUN && motors.armed()) {
            Log_Write_Control_Tuning();
        }

    }else{
        // simple dithering of climb rate
        climb_rate += climb_rate_error;
        current_loc.alt += (climb_rate / 50);
    }
}

static void tuning(){
    tuning_value = (float)g.rc_6.control_in / 1000.0;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    case CH6_RATE_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    case CH6_STABILIZE_KP:
        g.pi_stabilize_roll.kP(tuning_value);
        g.pi_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_STABILIZE_KI:
        g.pi_stabilize_roll.kI(tuning_value);
        g.pi_stabilize_pitch.kI(tuning_value);
        break;

    case CH6_ACRO_KP:
        g.acro_p = tuning_value;
        break;

    case CH6_RATE_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_YAW_KP:
        g.pi_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_KI:
        g.pi_stabilize_yaw.kI(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    case CH6_THROTTLE_KP:
        g.pid_throttle.kP(tuning_value);
        break;

    case CH6_THROTTLE_KI:
        g.pid_throttle.kI(tuning_value);
        break;

    case CH6_TOP_BOTTOM_RATIO:
        motors.top_bottom_ratio = tuning_value;
        break;

    case CH6_RELAY:
        if (g.rc_6.control_in > 525) relay.on();
        if (g.rc_6.control_in < 475) relay.off();
        break;

    case CH6_TRAVERSE_SPEED:
        g.waypoint_speed_max = g.rc_6.control_in;
        break;

    case CH6_LOITER_KP:
        g.pi_loiter_lat.kP(tuning_value);
        g.pi_loiter_lon.kP(tuning_value);
        break;

    case CH6_LOITER_KI:
        g.pi_loiter_lat.kI(tuning_value);
        g.pi_loiter_lon.kI(tuning_value);
        break;

    case CH6_NAV_KP:
        g.pid_nav_lat.kP(tuning_value);
        g.pid_nav_lon.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KP:
        g.pid_loiter_rate_lon.kP(tuning_value);
        g.pid_loiter_rate_lat.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KI:
        g.pid_loiter_rate_lon.kI(tuning_value);
        g.pid_loiter_rate_lat.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KD:
        g.pid_loiter_rate_lon.kD(tuning_value);
        g.pid_loiter_rate_lat.kD(tuning_value);
        break;

    case CH6_NAV_I:
        g.pid_nav_lat.kI(tuning_value);
        g.pid_nav_lon.kI(tuning_value);
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain = tuning_value;
        break;
#endif

    case CH6_THR_HOLD_KP:
        g.pi_alt_hold.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KP:
        g.pid_optflow_roll.kP(tuning_value);
        g.pid_optflow_pitch.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KI:
        g.pid_optflow_roll.kI(tuning_value);
        g.pid_optflow_pitch.kI(tuning_value);
        break;

    case CH6_OPTFLOW_KD:
        g.pid_optflow_roll.kD(tuning_value);
        g.pid_optflow_pitch.kD(tuning_value);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE                                       // do not allow modifying _kp or _kp_yaw gains in HIL mode
    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;
#endif

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    case CH6_INAV_TC:
        inertial_nav.set_time_constant_xy(tuning_value);
        inertial_nav.set_time_constant_z(tuning_value);
        break;
#endif

    case CH6_THR_ACCEL_KP:
        g.pid_throttle_accel.kP(tuning_value);
        break;

    case CH6_THR_ACCEL_KI:
        g.pid_throttle_accel.kI(tuning_value);
        break;

    case CH6_THR_ACCEL_KD:
        g.pid_throttle_accel.kD(tuning_value);
        break;
    }
}

// Outputs Nav_Pitch and Nav_Roll
static void update_nav_wp()
{
    if(wp_control == LOITER_MODE) {

        // calc error to target
        calc_location_error(&next_WP);

        // use error as the desired rate towards the target
        calc_loiter(long_error, lat_error);

    }else if(wp_control == CIRCLE_MODE) {

        // check if we have missed the WP
        int16_t loiter_delta = (target_bearing - old_target_bearing)/100;

        // reset the old value
        old_target_bearing = target_bearing;

        // wrap values
        if (loiter_delta > 180) loiter_delta -= 360;
        if (loiter_delta < -180) loiter_delta += 360;

        // sum the angle around the WP
        loiter_sum += loiter_delta;

        circle_angle += (circle_rate * dTnav);
        //1° = 0.0174532925 radians

        // wrap
        if (circle_angle > 6.28318531)
            circle_angle -= 6.28318531;

        next_WP.lng = circle_WP.lng + (g.loiter_radius * 100 * cos(1.57 - circle_angle) * scaleLongUp);
        next_WP.lat = circle_WP.lat + (g.loiter_radius * 100 * sin(1.57 - circle_angle));

        // use error as the desired rate towards the target
        // nav_lon, nav_lat is calculated

        if(wp_distance > 400) {
            calc_nav_rate(get_desired_speed(g.waypoint_speed_max));
        }else{
            // calc the lat and long error to the target
            calc_location_error(&next_WP);

            calc_loiter(long_error, lat_error);
        }

    }else if(wp_control == WP_MODE) {
        // calc error to target
        calc_location_error(&next_WP);

        int16_t speed = get_desired_speed(g.waypoint_speed_max);
        // use error as the desired rate towards the target
        calc_nav_rate(speed);

    }else if(wp_control == NO_NAV_MODE) {
        // clear out our nav so we can do things like land straight down
        // or change Loiter position

        // We bring copy over our Iterms for wind control, but we don't navigate
        nav_lon = g.pid_loiter_rate_lon.get_integrator();
        nav_lat = g.pid_loiter_rate_lon.get_integrator();

        nav_lon                 = constrain(nav_lon, -2000, 2000);                              // 20°
        nav_lat                 = constrain(nav_lat, -2000, 2000);                              // 20°
    }
}

static void update_auto_yaw()
{
    switch( wp_control ) {
        case LOITER_MODE:
            // just hold nav_yaw
            break;
        case WP_MODE:
            if(auto_yaw_tracking == MAV_ROI_LOCATION) {
                auto_yaw = get_bearing_cd(&current_loc, &target_WP);
            }else if(auto_yaw_tracking == MAV_ROI_WPNEXT) {
                // Point towards next WP
                // we don't use target_bearing because we don't want the copter to turn too much during flight
                auto_yaw = original_target_bearing;
            }
            break;
        case CIRCLE_MODE:
            auto_yaw = get_bearing_cd(&current_loc, &circle_WP);
            break;
        case NO_NAV_MODE:
            // just hold nav_yaw
            break;
    }
}
