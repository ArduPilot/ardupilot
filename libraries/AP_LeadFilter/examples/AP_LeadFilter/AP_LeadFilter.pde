/*
 *       Example of AP Lead_Filter library.
 *       Code by Jason Short. 2010
 *       DIYDrones.com
 *
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_LeadFilter.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_LeadFilter xLeadFilter;      // GPS lag filter

void setup()
{
    hal.console->println("AP_LeadFilter test ver 1.0");
    hal.scheduler->delay(500);
}

void loop()
{
    int16_t velocity;
    int32_t position;
    int32_t new_position;
    int16_t i;

    position = 0;
    velocity = 0;
    xLeadFilter.clear();

    hal.console->printf("------------------\n");
    hal.console->printf("start position = 0, lag of 1sec.\n");
    for( i = 0; i < 10; i++ ) {
        // get updated position
        new_position = xLeadFilter.get_position(position, velocity);     // new position with velocity of 1 m/s
        hal.console->printf("start pos: %ld, start vel: %d, end pos: %ld\n", (long int)position, (int)velocity, (long int)new_position);
        position = new_position;
        velocity += 100;
    }

    position = 0;
    velocity = 0;
    xLeadFilter.clear();

    hal.console->printf("------------------\n");
    hal.console->printf("start position = 0, lag of 200ms\n");
    for( i = 0; i < 10; i++ ) {
        // get updated position
        new_position = xLeadFilter.get_position(position, velocity, 0.200);     // new position with velocity of 1 m/s
        hal.console->printf("start pos: %ld, start vel: %d, end pos: %ld\n", (long int)position, (int)velocity, (long int)new_position);
        position = new_position;
        velocity += 100;
    }

    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
