/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "AntennaTracker V0.1"
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

// tracking status for MAVLink
static struct {
    float bearing;
    float distance;
    float pitch;
    float altitude_difference;
    float altitude_offset;
    bool manual_control_yaw:1;
    bool manual_control_pitch:1;
    bool need_altitude_calibration:1;
    bool scan_reverse_pitch:1;
    bool scan_reverse_yaw:1;
} nav_status;

static uint32_t start_time_ms;

static bool usb_connected;

////////////////////////////////////////////////////////////////////////////////
// prototypes
void gcs_send_text_fmt(const prog_char_t *fmt, ...);


////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_GPS gps;

#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO setting
#endif

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

#if CONFIG_INS_TYPE == HAL_INS_OILPAN || CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

#if CONFIG_INS_TYPE == HAL_INS_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_INS_TYPE == HAL_INS_PX4
AP_InertialSensor_PX4 ins;
#elif CONFIG_INS_TYPE == HAL_INS_VRBRAIN
AP_InertialSensor_VRBRAIN ins;
#elif CONFIG_INS_TYPE == HAL_INS_HIL
AP_InertialSensor_HIL ins;
#elif CONFIG_INS_TYPE == HAL_INS_OILPAN
AP_InertialSensor_Oilpan ins( &apm1_adc );
#elif CONFIG_INS_TYPE == HAL_INS_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_INS_TYPE == HAL_INS_L3G4200D
AP_InertialSensor_L3G4200D ins;
#elif CONFIG_INS_TYPE == HAL_INS_MPU9250
AP_InertialSensor_MPU9250 ins;
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE

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
static RC_Channel channel_yaw(CH_1);
static RC_Channel channel_pitch(CH_2);

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

// If proxy_mode is enabled, uartC is connected to a remote vehicle,
// not gcs[1]
static GCS_MAVLINK proxy_vehicle;

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

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 20ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_ahrs,            1,   1000 },
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
AP_Param param_loader(var_info, EEPROM_MAX_ADDR);

/**
  setup the sketch - called once on startup
 */
void setup() 
{
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // arduplane does not use arming nor pre-arm checks
    AP_Notify::flags.armed = true;
    AP_Notify::flags.pre_arm_check = true;
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
    if (!ins.wait_for_sample(1000)) {
        return;
    }

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
