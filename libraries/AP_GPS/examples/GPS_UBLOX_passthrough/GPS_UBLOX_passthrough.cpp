/*
 *       GPS UBlox passthrough sketch
 *       Code by DIYDrones.com
 */

#include <stdlib.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup()
{
    // initialise console uart to 38400 baud
    hal.console->begin(38400);

    // initialise gps uart to 38400 baud
    hal.serial(3)->begin(38400);
}

void loop()
{
    // send characters received from the console to the GPS
    while (hal.console->available()) {
        hal.serial(3)->write(hal.console->read());
    }
    // send GPS characters to the console
    while (hal.serial(3)->available()) {
        hal.console->write(hal.serial(3)->read());
    }
}

AP_HAL_MAIN();
