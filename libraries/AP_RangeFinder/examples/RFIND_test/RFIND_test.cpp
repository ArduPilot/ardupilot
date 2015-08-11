/*
 *  RangeFinder test code
 */
 
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <DataFlash/DataFlash.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Rally/AP_Rally.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static RangeFinder sonar;

void setup()
{
    // print welcome message
    hal.console->println("Range Finder library test");

    // setup for analog pin 13
    AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_PLI2C);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", -1);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 1.0);

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

    hal.console->printf_P(PSTR("Primary: status %d distance_cm %d \n"), (int)sonar.status(), sonar.distance_cm());
    hal.console->printf_P(PSTR("All: device_0 type %d status %d distance_cm %d, device_1 type %d status %d distance_cm %d\n"),
    (int)sonar._type[0], (int)sonar.status(0), sonar.distance_cm(0), (int)sonar._type[1], (int)sonar.status(1), sonar.distance_cm(1));

}
AP_HAL_MAIN();
