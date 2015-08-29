/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <Filter/Filter.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static OpticalFlow optflow;

void setup()
{
    hal.console->println("OpticalFlow library test ver 1.6");

    hal.scheduler->delay(1000);

    // flowSensor initialization
    optflow.init();

    if (!optflow.healthy()) {
        hal.console->print("Failed to initialise PX4Flow ");
    }

    hal.scheduler->delay(1000);
}

void loop()
{
    hal.console->println("this only tests compilation succeeds");

    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();
