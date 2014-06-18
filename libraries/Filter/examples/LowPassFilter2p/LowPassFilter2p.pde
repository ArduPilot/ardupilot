/*
 *       Example sketch to demonstrate use of LowPassFilter2p library.
 *       Code by Randy Mackay and Andrew Tridgell
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_Param.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <Filter.h>                     // Filter library
#include <LowPassFilter2p.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// craete an instance with 800Hz sample rate and 30Hz cutoff
static LowPassFilter2p low_pass_filter(800, 30);

// setup routine
static void setup()
{
    // introduction
    hal.console->printf("ArduPilot LowPassFilter2p test\n\n");
}

void loop()
{
    int16_t i;
    float new_value;
    float filtered_value;

    for( i=0; i<300; i++ ) {

        // new data value
        new_value = sinf((float)i*2*PI*5/50.0f);  // 5hz

        // output to user
        hal.console->printf("applying: %6.4f", new_value);

        // apply new value and retrieved filtered result
        filtered_value = low_pass_filter.apply(new_value);

        // display results
        hal.console->printf("\toutput: %6.4f\n", filtered_value);

        hal.scheduler->delay(10);
    }
    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
