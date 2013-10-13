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

#include <AP_Vehicle.h>
#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library
#include <AP_Airspeed.h>
#include <RC_Channel.h>

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
} nav_status;


////////////////////////////////////////////////////////////////////////////////
// prototypes
void gcs_send_text_fmt(const prog_char_t *fmt, ...);


////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
// All GPS access should be through this pointer.
static GPS         *g_gps;

#if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == AP_BARO_HIL
static AP_Baro_HIL barometer;
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
AP_GPS_Auto     g_gps_driver(&g_gps);

#if CONFIG_INS_TYPE == CONFIG_INS_MPU6000
AP_InertialSensor_MPU6000 ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_PX4
AP_InertialSensor_PX4 ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_HIL
AP_InertialSensor_HIL ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_OILPAN
AP_InertialSensor_Oilpan ins( &apm1_adc );
#elif CONFIG_INS_TYPE == CONFIG_INS_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_INS_TYPE == CONFIG_INS_L3G4200D
AP_InertialSensor_L3G4200D ins;
#else
  #error Unrecognised CONFIG_INS_TYPE setting.
#endif // CONFIG_INS_TYPE

AP_AHRS_DCM ahrs(&ins, g_gps);

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
static GCS_MAVLINK gcs0;
static GCS_MAVLINK gcs3;

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
    { update_tracking,        1,   1000 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { compass_accumulate,     1,   1500 },
    { barometer_accumulate,   1,    900 },
    { update_notify,          1,    100 },
    { one_second_loop,       50,   3900 }
};

// setup the var_info table
AP_Param param_loader(var_info, EEPROM_MAX_ADDR);

/**
  setup the sketch - called once on startup
 */
void setup() 
{
    // this needs to be the first call, as it fills memory with
    // sentinel values
    memcheck_init();

    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // arduplane does not use arming nor pre-arm checks
    AP_Notify::flags.armed = true;
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.failsafe_battery = false;

    notify.init();
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
