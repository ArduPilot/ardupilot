/*
  simple hello world sketch
  Andrew Tridgell September 2011

  Modified for URUS TEST:
    -   Hiroshi Takey December 2016
*/

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint32_t now, last;

void setup()
{
    now = AP_HAL::millis();
    printf("\nhello URUS! time: %u\n", now);
}

void loop()
{
    static uint32_t count = 0;
    printf("counter: %u time: %u\n", count++, last);
    hal.scheduler->delay(1000);
    last = AP_HAL::millis() - now;
    now = AP_HAL::millis();
}

AP_HAL_MAIN();
