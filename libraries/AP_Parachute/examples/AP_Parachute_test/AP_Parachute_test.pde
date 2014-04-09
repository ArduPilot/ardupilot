/*
 *       Example of AP_Parachute library.
 *       DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <RC_Channel.h>
#include <AP_Relay.h>
#include <AP_Parachute.h>
#include <AP_Notify.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Relay
AP_Relay relay;

// Parachute
AP_Parachute parachute(relay);

void setup()
{
    hal.console->println("AP_Parachute library test");
}

void loop()
{
    // print message to user
    hal.console->printf_P(PSTR("this example tests compilation only"));
    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();
