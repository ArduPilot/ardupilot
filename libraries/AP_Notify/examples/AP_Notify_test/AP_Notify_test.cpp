/*
 *       Example of AC_Notify library .
 *       DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_Notify/AP_BoardLED.h>        // Board LED library
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <StorageManager/StorageManager.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

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
