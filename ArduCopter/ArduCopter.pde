/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V2.9-rc2"
/*
 *  ArduCopter Version 2.9
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

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_LeadFilter.h>      // GPS Lead filter
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AP_Limits.h>
#include <memcheck.h>
#include <SITL.h>

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler. 

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;


////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
DataFlash_SITL DataFlash;
#else
DataFlash_Empty DataFlash;
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

 #if CONFIG_ADC == ENABLED
AP_ADC_ADS7844 adc;
 #endif

 #if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
AP_InertialSensor_Stub ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
AP_InertialSensor_PX4 ins;
 #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
AP_Baro_BMP085_HIL barometer;
AP_Compass_HIL compass;
SITL sitl;
 #else
// Otherwise, instantiate a real barometer and compass driver
  #if CONFIG_BARO == AP_BARO_BMP085
AP_Baro_BMP085 barometer;
  #elif CONFIG_BARO == AP_BARO_PX4
AP_Baro_PX4 barometer;
  #elif CONFIG_BARO == AP_BARO_MS5611
   #if CONFIG_MS5611_SERIAL == SPI
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
   #elif CONFIG_MS5611_SERIAL == I2C
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
   #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
   #endif
  #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
AP_Compass_PX4 compass;
 #else
AP_Compass_HMC5843 compass;
 #endif
 #endif

 #if OPTFLOW == ENABLED
AP_OpticalFlow_ADNS3080 optflow;
 #else
AP_OpticalFlow optflow;
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

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver();

 #else
  #error Unrecognised GPS_PROTOCOL setting.
 #endif // GPS PROTOCOL

 #if DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_AHRS_MPU6000  ahrs(&ins, g_gps);               // only works with APM2
 #else
AP_AHRS_DCM ahrs(&ins, g_gps);
 #endif

// ahrs2 object is the secondary ahrs to allow running DMP in parallel with DCM
  #if SECONDARY_DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_AHRS_MPU6000  ahrs2(&ins, g_gps);               // only works with APM2
  #endif

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
AP_ADC_HIL              adc;
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL          compass;
AP_GPS_HIL              g_gps_driver;
AP_InertialSensor_Stub  ins;
AP_AHRS_DCM             ahrs(&ins, g_gps);


static int32_t gps_base_alt;

#elif HIL_MODE == HIL_MODE_ATTITUDE
AP_ADC_HIL              adc;
AP_InertialSensor_Stub  ins;
AP_AHRS_HIL             ahrs(&ins, g_gps);
AP_GPS_HIL              g_gps_driver;
AP_Compass_HIL          compass;                  // never used
AP_Baro_BMP085_HIL      barometer;

 #if OPTFLOW == ENABLED
  #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_OpticalFlow_ADNS3080 optflow;
  #else
AP_OpticalFlow_ADNS3080 optflow;
  #endif    // CONFIG_HAL_BOARD == HAL_BOARD_APM2
 #endif     // OPTFLOW == ENABLED

static int32_t gps_base_alt;
#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
GCS_MAVLINK gcs0;
GCS_MAVLINK gcs3;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//

#if CONFIG_SONAR == ENABLED
ModeFilterInt16_Size5 sonar_mode_filter(2);
AP_HAL::AnalogSource *sonar_analog_source;
AP_RangeFinder_MaxsonarXL *sonar;
#endif

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
    uint8_t alt_sensor_flag         : 1; // 7   // used to track when to read sensors vs estimate alt
    uint8_t yaw_stopped             : 1; // 8   // Used to manage the Yaw hold capabilities

} ap_system;

/*
  what navigation updated are needed
 */
static struct nav_updates {
    uint8_t need_velpos             : 1;
    uint8_t need_dist_bearing       : 1;
    uint8_t need_nav_controllers    : 1;
    uint8_t need_nav_pitch_roll     : 1;
} nav_updates;


////////////////////////////////////////////////////////////////////////////////
// velocity in lon and lat directions calculated from GPS position and accelerometer data
// updated after GPS read - 5-10hz
static int16_t lon_speed;       // expressed in cm/s.  positive numbers mean moving east
static int16_t lat_speed;       // expressed in cm/s.  positive numbers when moving north

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
static uint8_t oldSwitchPosition;

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
MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_8, &g.heli_servo_1, &g.heli_servo_2, &g.heli_servo_3, &g.heli_servo_4);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_7);
#else
MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);
#endif

////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////
// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;
// This is used to hold radio tuning values for in-flight CH6 tuning
float tuning_value;
// used to limit the rate that the pid controller output is logged so that it doesn't negatively affect performance
static uint8_t pid_log_counter;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// This is current status for the LED lights state machine
// setting this value changes the output of the LEDs
static uint8_t led_mode = NORMAL_LEDS;
// Blinking indicates GPS status
static uint8_t copter_leds_GPS_blink;
// Blinking indicates battery status
static uint8_t copter_leds_motor_blink;
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
static uint8_t wp_control;
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
static int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
static bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
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
static bool sonar_alt_ok;   // true if we can trust the altitude from the sonar
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
static uint8_t yaw_mode;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode;
// The current desired control scheme for altitude hold
static uint8_t throttle_mode;


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
static uint8_t event_id;
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
// Counters for branching from 10 hz control loop
static uint8_t medium_loopCounter;
// Counters for branching from 3 1/3hz control loop
static uint8_t slow_loopCounter;
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

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
AP_Relay relay;

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  AP_Camera camera(&relay);
#endif

// a pin for reading the receiver RSSI voltage. The scaling by 0.25
// is to take the 0 to 1024 range down to an 8 bit range for MAVLink
AP_HAL::AnalogSource* rssi_analog_source;


// Input sources for battery voltage, battery current, board vcc
AP_HAL::AnalogSource* batt_volt_analog_source;
AP_HAL::AnalogSource* batt_curr_analog_source;
AP_HAL::AnalogSource* board_vcc_analog_source;


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


////////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
#if AP_LIMITS == ENABLED
AP_Limits               limits;
AP_Limit_GPSLock        gpslock_limit(g_gps);
AP_Limit_Geofence       geofence_limit(FENCE_START_BYTE, FENCE_WP_SIZE, MAX_FENCEPOINTS, g_gps, &home, &current_loc);
AP_Limit_Altitude       altitude_limit(&current_loc);
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

void setup() {
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

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

    rssi_analog_source      = hal.analogin->channel(g.rssi_pin, 0.25);
    batt_volt_analog_source = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_analog_source = hal.analogin->channel(g.battery_curr_pin);
    board_vcc_analog_source = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    memcheck_init();
    init_ardupilot();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

// enable this to get console logging of scheduler performance
#define SCHEDULER_DEBUG 0

static void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();
#if SCHEDULER_DEBUG
    cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"), 
                        (unsigned)perf_info_get_num_long_running(),
                        (unsigned)perf_info_get_num_loops(),
                        (unsigned long)perf_info_get_max_time());
#endif
    perf_info_reset();
    gps_fix_count = 0;
}

/*
  return number of micros until the main loop will want to run again
 */
static int16_t main_loop_time_available(void)
{
    uint32_t dt = (micros() - fast_loopTimer);
    if (dt > 10000) {
        return 0;
    }
    return 10000 - dt;
}

typedef void (*event_fn_t)(void);

struct timer_event_table {
    event_fn_t func;
    uint16_t time_interval_10ms;
    uint16_t min_time_usec;
};

#define NUM_TIMER_EVENTS 14

/*
  scheduler table - all regular events apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take
 */
static const struct timer_event_table PROGMEM timer_events[NUM_TIMER_EVENTS] = {
    { update_GPS,         2,     900 },
    { update_navigation,     2,     500 },
    { medium_loop,           2,     700 },
    { update_altitude_est,   2,     900 },
    { fifty_hz_loop,         2,     800 },
    { run_nav_updates,       2,     500 },
    { slow_loop,            10,     500 },
    { gcs_check_input,	     2,     700 },
    { gcs_send_heartbeat,  100,     700 },
    { gcs_data_stream_send,  2,    1100 },
    { gcs_send_deferred,     2,     700 },
    { compass_accumulate,    2,     600 },
    { super_slow_loop,     100,    1100 },
    { perf_update,        1000,     500 }
};
static uint16_t timer_counters[NUM_TIMER_EVENTS];

static uint16_t tick_counter;
static uint32_t event_time_started;
static uint16_t event_time_allowed;

/*
  return number of micros until the current event reaches its deadline
 */
static uint16_t event_time_available(void)
{
    uint32_t dt = micros() - event_time_started;
    if (dt > event_time_allowed) {
        return 0;
    }
    return event_time_allowed - dt;
}

/*
  run as many scheduler events as we can
 */
static void run_events(uint16_t time_available_usec)
{
    for (uint8_t i=0; i<NUM_TIMER_EVENTS; i++) {
        if (tick_counter - timer_counters[i] >= pgm_read_word(&timer_events[i].time_interval_10ms)) {
            // this event is due to run. Do we have enough time to run it?
            event_time_allowed = pgm_read_word(&timer_events[i].min_time_usec);
            if (event_time_allowed <= time_available_usec) {
                // run it
                event_time_started = micros();
                event_fn_t func = (event_fn_t)pgm_read_pointer(&timer_events[i].func);
                func();
                
                // record the tick counter when we ran. This drives
                // when we next run the event
                timer_counters[i] = tick_counter;
                
                // work out how long the event actually took
                uint32_t time_taken = micros() - event_time_started;
                
                if (time_taken > event_time_allowed) {
                    // the event overran!
#if SCHEDULER_DEBUG
                    cliSerial->printf_P(PSTR("overrun in event %u (%u/%u)\n"), 
                                        (unsigned)i, 
                                        (unsigned)time_taken,
                                        (unsigned)event_time_allowed);
#endif
                    return;
                }
                time_available_usec -= time_taken;
            }
        }
    }
}

void loop()
{
    uint32_t timer = micros();

    // We want this to execute fast
    // ----------------------------
    if (ins.num_samples_available() >= 2) {

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

        tick_counter++;
    } else {
        uint16_t time_to_next_loop;
        uint16_t dt = timer - fast_loopTimer;
        if (dt > 10000) {
            time_to_next_loop = 0;
        } else {
            time_to_next_loop = 10000 - dt;
        }
        run_events(time_to_next_loop);
    }
}


// Main loop - 100hz
static void fast_loop()
{
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

        // auto_trim - stores roll and pitch radio inputs to ahrs
        auto_trim();

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
    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

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
    camera.trigger_pic_cleanup();
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
    static uint8_t of_log_counter = 0;

    // if new data has arrived, process it
    if( optflow.last_update != last_of_update ) {
        last_of_update = optflow.last_update;
        optflow.update_position(ahrs.roll, ahrs.pitch, cos_yaw_x, sin_yaw_y, current_loc.alt);      // updates internal lon and lat with estimation based on optical flow

        // write to log at 5hz
        of_log_counter++;
        if( of_log_counter >= 4 ) {
            of_log_counter = 0;
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
    static uint8_t ground_start_count  = 10;

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
            ap_system.alt_sensor_flag = true;
#endif
        }

        // save GPS time so we don't get duplicate reads
        last_gps_time = g_gps->time;
    }
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
        case YAW_ACRO:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = get_bearing_cd(&current_loc, &yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_TOY:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    switch(yaw_mode) {

    case YAW_HOLD:
        // heading hold at heading held in nav_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        break;

    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        if(g.axis_enabled) {
            get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        }else{
            get_acro_yaw(g.rc_4.control_in);
        }
        break;

    case YAW_LOOK_AT_NEXT_WP:
        // point towards next waypoint (no pilot input accepted)
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        nav_yaw = get_yaw_slew(nav_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // point towards a location held in yaw_look_at_WP (no pilot input accepted)
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HOME:
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        get_stabilize_yaw(nav_yaw);
        break;

	case YAW_LOOK_AHEAD:
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(g.rc_4.control_in);
        break;

#if TOY_LOOKUP == TOY_EXTERNAL_MIXER
    case YAW_TOY:
        // update to allow external roll/yaw mixing
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);
        break;
#endif
    }
}

// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
        case ROLL_PITCH_ACRO:
        case ROLL_PITCH_AUTO:
        case ROLL_PITCH_STABLE_OF:
        case ROLL_PITCH_TOY:
        case ROLL_PITCH_LOITER_PR:
            roll_pitch_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
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
        nav_roll                += constrain_int32(wrap_180(auto_roll  - nav_roll),  -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second
        nav_pitch               += constrain_int32(wrap_180(auto_pitch - nav_pitch), -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second

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
        
    case ROLL_PITCH_LOITER_PR:
    
        // LOITER does not get SIMPLE mode ability
        
        nav_roll                += constrain(wrap_180(auto_roll  - nav_roll),  -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second
        nav_pitch               += constrain(wrap_180(auto_pitch - nav_pitch), -g.auto_slew_rate.get(), g.auto_slew_rate.get());                 // 40 deg a second

        get_stabilize_roll(nav_roll);
        get_stabilize_pitch(nav_pitch);
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
    static uint8_t simple_counter = 0;             // State machine counter for Simple Mode
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

    // return immediately if no change
    if( new_throttle_mode == throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_ACCELERATION:                     // pilot inputs the desired acceleration
            if( g.throttle_accel_enabled ) {            // this throttle mode requires use of the accel based throttle controller
                altitude_error = 0;                     // clear altitude error reported to GCS
                throttle_initialised = true;
            }
            break;

        case THROTTLE_RATE:
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;
        case THROTTLE_STABILIZED_RATE:
        case THROTTLE_DIRECT_ALT:
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            set_new_altitude(current_loc.alt);          // by default hold the current altitude
            if ( throttle_mode < THROTTLE_HOLD ) {      // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
            }
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            set_land_complete(false);   // mark landing as incomplete
            land_detector = 0;          // A counter that goes up if our climb rate stalls out.
            set_new_altitude(0);        // Set a new target altitude
            throttle_initialised = true;
            break;

        case THROTTLE_SURFACE_TRACKING:
            if( g.sonar_enabled ) {
                set_new_altitude(current_loc.alt);          // by default hold the current altitude
                if ( throttle_mode < THROTTLE_HOLD ) {      // reset the alt hold I terms if previous throttle mode was manual
                    reset_throttle_I();
                }
                throttle_initialised = true;
            }
            // To-Do: handle the case where the sonar is not enabled
            break;

        default:
            // To-Do: log an error message to the dataflash or tlogs instead of printing to the serial port
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

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;

    if(ap.do_flip)     // this is pretty bad but needed to flip in AP modes.
        return;

    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
	if (control_mode == STABILIZE){
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
			#if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.coll_out);
			#else
			update_throttle_cruise(g.rc_3.control_in);
			#endif  //HELI_FRAME


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
            #if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.coll_out);
			#else
			update_throttle_cruise(g.rc_3.control_in);
			#endif  //HELI_FRAME

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
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        }else{
            int16_t desired_acceleration = get_pilot_desired_acceleration(g.rc_3.control_in);
            set_throttle_accel_target(desired_acceleration);
        }
        break;

    case THROTTLE_RATE:
        // pilot inputs the desired climb rate.  Note this is the unstabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate(pilot_climb_rate);
        }
        break;

    case THROTTLE_STABILIZED_RATE:
        // pilot inputs the desired climb rate.  Note this is the stabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
            altitude_error = 0;             // clear altitude error reported to GCS - normally underlying alt hold controller updates altitude error reported to GCS
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate_stabilized(pilot_climb_rate);
        }
        break;

    case THROTTLE_DIRECT_ALT:
        // pilot inputs a desired altitude from 0 ~ 10 meters
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
            altitude_error = 0;             // clear altitude error reported to GCS - normally underlying alt hold controller updates altitude error reported to GCS
        }else{
            int32_t desired_alt = get_pilot_desired_direct_alt(g.rc_3.control_in);
            get_throttle_althold(desired_alt, g.auto_velocity_z_min, g.auto_velocity_z_max);
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
            get_throttle_althold(next_WP.alt, g.auto_velocity_z_min, g.auto_velocity_z_max);
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        break;

    case THROTTLE_SURFACE_TRACKING:
        // surface tracking with sonar or other rangefinder plus pilot input of climb rate
        pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
        if( sonar_alt_ok ) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(pilot_climb_rate);
        }else{
            // if no sonar fall back stabilize rate controller
            get_throttle_rate_stabilized(pilot_climb_rate);
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
    gcs_check_input();
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

// updated at 10hz
static void update_altitude()
{
    int32_t old_baro_alt    = baro_alt;
    int16_t old_sonar_alt   = sonar_alt;

#if HIL_MODE == HIL_MODE_ATTITUDE
    // we are in the SIM, fake out the baro and Sonar
    int16_t fake_relative_alt = g_gps->altitude - gps_base_alt;
    baro_alt                = fake_relative_alt;
    baro_rate               = (baro_alt - old_baro_alt) * 5;             // 5hz
    if(g.sonar_enabled) {
        sonar_alt           = fake_relative_alt;
        sonar_rate          = baro_rate;
    }
    current_loc.alt = baro_alt;
    climb_rate_actual = baro_rate;
#else
    // read in actual baro altitude
    baro_alt            = read_barometer();

    // calc baro based vertical velocity
    int16_t temp		= (baro_alt - old_baro_alt) * 10;
    baro_rate           = (temp + baro_rate) >> 1;
    baro_rate			= constrain(baro_rate, -500, 500);

    // read in sonar altitude and calculate sonar rate
    if(g.sonar_enabled) {
        sonar_alt       = read_sonar();
        sonar_rate      = (sonar_alt - old_sonar_alt) * 10;
        sonar_rate      = constrain(sonar_rate, -150, 150);
    }

    // Note: with inertial nav, alt and rate are pulled from the inav lib at 50hz in update_altitude_est function
    // so none of the below is required
# if INERTIAL_NAV_Z != ENABLED
    // if no sonar set current alt to baro alt
    if(!g.sonar_enabled) {
        // NO Sonar case
        current_loc.alt = baro_alt;
        climb_rate_actual = baro_rate;
    }else{
        // Blend barometer and sonar data together
        float scale;
        if(baro_alt < 800) {
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
    }
    // climb_rate_error is used to spread the change in climb rate across the next 5 samples
    climb_rate_error = (climb_rate_actual - climb_rate) / 5;
# endif // INERTIAL_NAV_Z != ENABLED
#endif  // HIL_MODE == HIL_MODE_ATTITUDE

    // update the target altitude
    verify_altitude();
}

static void update_altitude_est()
{
#if INERTIAL_NAV_Z == ENABLED
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude();
    climb_rate = inertial_nav.get_velocity_z();

    // update baro and sonar alt and climb rate just for logging purposes
    // To-Do: remove alt_sensor_flag and move update_altitude to be called from 10hz loop
    if(ap_system.alt_sensor_flag) {
        ap_system.alt_sensor_flag = false;
        update_altitude();
        if(g.log_bitmask & MASK_LOG_CTUN && motors.armed()) {
            Log_Write_Control_Tuning();
        }
    }
#else
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
#endif
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

    case CH6_THROTTLE_KD:
        g.pid_throttle.kD(tuning_value);
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

    case CH6_NAV_KI:
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

    case CH6_INAV_TC:
#if INERTIAL_NAV_XY == ENABLED
        inertial_nav.set_time_constant_xy(tuning_value);
#endif
#if INERTIAL_NAV_Z == ENABLED
        inertial_nav.set_time_constant_z(tuning_value);
#endif
        break;

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

AP_HAL_MAIN();

