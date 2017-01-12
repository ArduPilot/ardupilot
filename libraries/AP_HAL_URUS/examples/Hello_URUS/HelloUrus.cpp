/*
  simple hello world sketch
  Andrew Tridgell September 2011

  Modified for URUS TEST:
    -   Hiroshi Takey December 2016
*/

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

uint32_t time;

void setup()
{
    time = AP_HAL::millis();
    printf("\nhello URUS! time: %u\n", time);
}

void loop()
{
    static uint32_t count = 0;
    if ((AP_HAL::millis() - time) > (1000 - 1)) {
        printf("counter: %u time: %u\n", count++, time);
        time = AP_HAL::millis();
    }
}

AP_HAL_MAIN();
