/*
 *  AP_RangeFinder_PX4_test
 *  Code by DIYDrones.com
 */
 
#include <AP_RangeFinder.h>
#include <AP_Common.h>
#include <AP_HAL.h>
#include <Filter.h> // Filter library

#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_HAL_PX4.h>
#include <GCS_MAVLink.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_AVR.h>
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

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

// declare global instances
ModeFilterInt16_Size5 mode_filter(2);
AP_RangeFinder_PX4 lrf(&mode_filter);
uint8_t device_count;

void setup()
{
    // print welcome message
    hal.console->println("PX4 Range Finder library test");

    // initialise sensor
    lrf.init();

    // display range finder device count
    device_count = lrf.get_count();
    hal.console->printf_P(PSTR("Number of range finders:%d\n"), (int)device_count);
    
    // kick off one reading
    lrf.take_reading();

    for (int i = 0; i < device_count; i++) {
        // check health
        if (!lrf.healthy(i)) {
            hal.console->printf_P(PSTR("Initialisation failed for device %d\n"), i);
        }
    }
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    
    // read the distance measurement
    lrf.read();
    
    for (int i = 0; i < device_count; i++) {
        // display output
        hal.console->printf_P(PSTR("Device:%d Distance:%d Health:%d\n"), i, (int)lrf.get_distance(i), (int)lrf.healthy(i));
    }
}

#else // Non-PX4
#warning AP_RangeFinder_PX4_test is PX4 specific
void setup () {}
void loop () {}
#endif

AP_HAL_MAIN();
