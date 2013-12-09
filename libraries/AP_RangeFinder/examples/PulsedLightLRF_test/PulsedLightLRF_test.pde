/*
 *  AP_RangeFinder_test
 *  Code by DIYDrones.com
 */

// includes
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_RangeFinder.h>
#include <Filter.h>
#include <AP_Buffer.h>

////////////////////////////////////////////////////////////////////////////////
// hal.console-> ports
////////////////////////////////////////////////////////////////////////////////
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// declare global instances
ModeFilterInt16_Size5 mode_filter(2);
AP_RangeFinder_PulsedLightLRF lrf(&mode_filter);

void setup()
{
    // print welcome message
    hal.console->println("PulsedLight LRF Test v1.1");

    // ensure i2c is slow
    hal.i2c->setHighSpeed(false);

    // initialise sensor
    lrf.init();

    // kick off one reading
    lrf.take_reading();

    // check health
    if (!lrf.healthy) {
        hal.console->println("Initialisation failed");
    }
}

void loop()
{
    int16_t distance;

    // ask sensor take a reading
    //lrf.take_reading();
    hal.scheduler->delay(50);

    // get distance
    distance = lrf.read();

    // display output
    //hal.console->printf_P(PSTR("dist: %d \thealth:%d \tb0:%d b1:%d\n"),(int)distance,(int)lrf.healthy,(int)lrf._buff[0], (int)lrf._buff[1]);
    //hal.console->printf_P(PSTR("dist: %d\n"),(int)distance);
    hal.console->printf_P(PSTR("dist: %d  %d\n"),(int)distance,(int)lrf.healthy);
}

AP_HAL_MAIN();
