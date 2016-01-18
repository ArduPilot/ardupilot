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
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Scheduler.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static RangeFinder sonar;

void setup()
{
    // print welcome message
    hal.console->println("Range Finder library test");

    // setup for analog pin 13
    AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_ANALOG);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", 13);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 3.10);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    sonar.init();
    hal.console->printf_P(PSTR("RangeFinder: %d devices detected\n"), sonar.num_sensors());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    sonar.update();

    hal.console->printf_P(PSTR("Primary: health %d distance_cm %d \n"), (int)sonar.healthy(), sonar.distance_cm());
    hal.console->printf_P(PSTR("All: device_0 type %d health %d distance_cm %d, device_1 type %d health %d distance_cm %d\n"),
    (int)sonar._type[0], (int)sonar.healthy(0), sonar.distance_cm(0), (int)sonar._type[1], (int)sonar.healthy(1), sonar.distance_cm(1));

}
AP_HAL_MAIN();
