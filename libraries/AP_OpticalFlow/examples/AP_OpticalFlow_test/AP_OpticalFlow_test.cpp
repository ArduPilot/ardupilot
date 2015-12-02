/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
