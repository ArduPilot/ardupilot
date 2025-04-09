/*
 *       Example sketch to demonstrate use of LowPassFilter library.
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>                     // Filter library
#include <Filter/LowPassFilter.h>      // LowPassFilter class (inherits from Filter class)

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create a global instance of the class
LowPassFilterFloat low_pass_filter;

// setup routine
void setup()
{
    // introduction
    hal.console->printf("ArduPilot LowPassFilter test ver 1.0\n\n");

    // set-up filter
    low_pass_filter.set_cutoff_frequency(1.0f);

    // Wait for the serial connection
    hal.scheduler->delay(500);
}

//Main loop where the action takes place
void loop()
{
    // reset value to 100.  If not reset the filter will start at the first value entered
    low_pass_filter.reset(0);

    for(int16_t i = 0; i < 300; i++ ) {

        // new data value
        const float new_value = sinf((float)i * 2 * M_PI * 5 / 50.0f);  // 5hz

        // output to user
        hal.console->printf("applying: %6.4f", (double)new_value);

        // apply new value and retrieved filtered result
        const float filtered_value = low_pass_filter.apply(new_value, 0.02f);

        // display results
        hal.console->printf("\toutput: %6.4f\n", (double)filtered_value);

        hal.scheduler->delay(10);
    }
    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
