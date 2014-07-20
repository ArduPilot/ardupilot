/*
 *  RangeFinder test code
 */
 
#include <AP_RangeFinder.h>
#include <AP_Common.h>
#include <AP_HAL.h>
#include <Filter.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <GCS_MAVLink.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_NavEKF.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Notify.h>
#include <AP_Mission.h>
#include <AP_Scheduler.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static RangeFinder sonar;

void setup()
{
    // print welcome message
    hal.console->println("Range Finder library test");

    // setup for auto-detect with analog pin 13
    AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_AUTO);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", 13);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    sonar.init();
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    
    hal.console->printf("Distance %.2f\n", (float)sonar.distance_cm());
}
AP_HAL_MAIN();
