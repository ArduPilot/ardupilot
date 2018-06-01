/*
 *       Example of AC_Notify library .
 *       DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_Notify/AP_BoardLED.h>        // Board LED library

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create board led object
AP_BoardLED board_led;

void setup()
{
    hal.console->println("AP_Notify library test");

    // initialise the board leds
    board_led.init();

    // turn on initialising notification
    AP_Notify::flags.initialising = true;
    AP_Notify::flags.gps_status = 1;
    AP_Notify::flags.armed = 1;
    AP_Notify::flags.pre_arm_check = 1;
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
