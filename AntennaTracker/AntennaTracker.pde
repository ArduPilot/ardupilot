/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "AntennaTracker V0.2"
/*
   Lead developers: Matthew Ridley and Andrew Tridgell
 
   Please contribute your ideas! See http://dev.ardupilot.com for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <Filter.h>                     // Filter library
#include <AP_Buffer.h>      // APM FIFO Buffer
#include <memcheck.h>

#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash.h>
#include <SITL.h>
#include <PID.h>
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_NavEKF.h>

#include <AP_Vehicle.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <AP_Rally.h>
#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library
#include <AP_Airspeed.h>
#include <RC_Channel.h>
#include <AP_BoardConfig.h>
#include <AP_OpticalFlow.h>

// Configuration
#include "config.h"

// Local modules
#include "defines.h"

#include "Parameters.h"
#include "GCS.h"

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
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

// main loop scheduler
static AP_Scheduler scheduler;
 
// notification object for LEDs, buzzers etc
static AP_Notify notify;

static uint32_t start_time_ms;

static bool usb_connected;

////////////////////////////////////////////////////////////////////////////////
// prototypes
void gcs_send_text_fmt(const prog_char_t *fmt, ...);


////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_GPS gps;

static AP_Baro barometer;

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#elif CONFIG_COMPASS == HAL_COMPASS_AK8963
static AP_Compass_AK8963_MPU9250 compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

AP_InertialSensor ins;

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, barometer, gps);
#else
AP_AHRS_DCM ahrs(ins, barometer, gps);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

/**
   antenna control channels
 */
static RC_Channel channel_yaw(CH_YAW);
static RC_Channel channel_pitch(CH_PITCH);

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

// board specific config
static AP_BoardConfig BoardConfig;

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
// Location structure defined in AP_Common
////////////////////////////////////////////////////////////////////////////////
static struct   Location current_loc;

// This is the state of the antenna control system
// There are multiple states defined such as MANUAL, FBW-A, AUTO
static enum ControlMode control_mode  = INITIALISING;

////////////////////////////////////////////////////////////////////////////////
// Vehicle state
////////////////////////////////////////////////////////////////////////////////
static struct {
    bool location_valid;    // true if we have a valid location for the vehicle
    Location location;      // lat, long in degrees * 10^7; alt in meters * 100
    Location location_estimate; // lat, long in degrees * 10^7; alt in meters * 100
    uint32_t last_update_us;    // last position update in micxroseconds
    uint32_t last_update_ms;    // last position update in milliseconds
    float heading;          // last known direction vehicle is moving
    float ground_speed;     // vehicle's last known ground speed in m/s
} vehicle;

////////////////////////////////////////////////////////////////////////////////
// Navigation controller state
////////////////////////////////////////////////////////////////////////////////
static struct {
    float bearing;                  // bearing to vehicle in centi-degrees
    float distance;                 // distance to vehicle in meters
    float pitch;                    // pitch to vehicle in degrees (positive means vehicle is above tracker, negative means below)
    float altitude_difference;      // altitude difference between tracker and vehicle in meters.  positive value means vehicle is above tracker
    float altitude_offset;          // offset in meters which is added to tracker altitude to align altitude measurements with vehicle's barometer
    bool manual_control_yaw         : 1;// true if tracker yaw is under manual control
    bool manual_control_pitch       : 1;// true if tracker pitch is manually controlled
    bool need_altitude_calibration  : 1;// true if tracker altitude has not been determined (true after startup)
    bool scan_reverse_pitch         : 1;// controls direction of pitch movement in SCAN mode
    bool scan_reverse_yaw           : 1;// controls direction of yaw movement in SCAN mode
} nav_status;

////////////////////////////////////////////////////////////////////////////////
// Servo state
////////////////////////////////////////////////////////////////////////////////
static struct {
    bool yaw_lower      : 1;    // true if yaw servo has been limited from moving to a lower position (i.e. position or rate limited)
    bool yaw_upper      : 1;    // true if yaw servo has been limited from moving to a higher position (i.e. position or rate limited)
    bool pitch_lower    : 1;    // true if pitch servo has been limited from moving to a lower position (i.e. position or rate limited)
    bool pitch_upper    : 1;    // true if pitch servo has been limited from moving to a higher position (i.e. position or rate limited)
} servo_limit;

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 20ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_ahrs,            1,   1000 },
    { read_radio,             1,    200 },
    { update_tracking,        1,   1000 },
    { update_GPS,             5,   4000 },
    { update_compass,         5,   1500 },
    { update_barometer,       5,   1500 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { compass_accumulate,     1,   1500 },
    { barometer_accumulate,   1,    900 },
    { update_notify,          1,    100 },
    { check_usb_mux,          5,    300 },
    { gcs_retry_deferred,     1,   1000 },
    { one_second_loop,       50,   3900 }
};

// setup the var_info table
AP_Param param_loader(var_info);

/**
  setup the sketch - called once on startup
 */
void setup() 
{
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // antenna tracker does not use pre-arm checks or battery failsafe
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);
    init_tracker();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/**
   loop() is called continuously 
 */
void loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    // tell the scheduler one tick has passed
    scheduler.tick();

    scheduler.run(19900UL);
}

static void one_second_loop()
{
    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // updated armed/disarmed status LEDs
    update_armed_disarmed();

    static uint8_t counter;
    counter++;

    if (counter >= 60) {
        if(g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }
}

AP_HAL_MAIN();
