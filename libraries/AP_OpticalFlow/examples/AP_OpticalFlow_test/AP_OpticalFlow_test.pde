/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_OpticalFlow.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <AP_BattMonitor.h>

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
