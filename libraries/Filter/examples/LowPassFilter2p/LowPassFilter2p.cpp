/*
 *       Example sketch to demonstrate use of LowPassFilter2p library.
 *       Code by Randy Mackay and Andrew Tridgell
 */

#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>                     // Filter library
#include <Filter/LowPassFilter2p.h>

void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// craete an instance with 800Hz sample rate and 30Hz cutoff
static LowPassFilter2pFloat low_pass_filter(800, 30);

// setup routine
static void setup()
{
    // introduction
    hal.console->printf("ArduPilot LowPassFilter2p test\n\n");
}

void loop()
{
    for(int16_t i = 0; i < 300; i++ ) {

        // new data value
        const float new_value = 17 + sinf((float)i * 2 * M_PI * 5 / 50.0f);  // 5hz
        // apply new value and retrieved filtered result
        const float filtered_value = low_pass_filter.apply(new_value);

        // output to user
        hal.console->printf("applying: %6.4f -> %6.4f\n", (double)new_value, filtered_value);

        hal.scheduler->delay(10);
    }
    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
