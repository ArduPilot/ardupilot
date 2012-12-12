/*
 *       Example sketch to demonstrate use of LowPassFilter library.
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Param.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <Filter.h>                     // Filter library
#include <LowPassFilter.h>      // LowPassFilter class (inherits from Filter class)
#include <AP_Buffer.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

// create a global instance of the class
LowPassFilterFloat low_pass_filter;

// setup routine
void setup()
{
    // introduction
    hal.console->printf("ArduPilot LowPassFilter test ver 1.0\n\n");

    // set-up filter
    low_pass_filter.set_time_constant(0.02, 0.015);
    //low_pass_filter.set_cutoff_frequency(0.02, 1.0);

    // Wait for the serial connection
    hal.scheduler->delay(500);
}

//Main loop where the action takes place
void loop()
{
    int16_t i;
    float new_value;
    float filtered_value;

    // reset value to 100.  If not reset the filter will start at the first value entered
    low_pass_filter.reset(0);

    for( i=0; i<300; i++ ) {

        // new data value
        new_value = sin((float)i*2*M_PI*5/50.0);  // 5hz

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
