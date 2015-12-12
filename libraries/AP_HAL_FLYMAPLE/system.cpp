#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>
#include <AP_HAL_FLYMAPLE/Scheduler.h>

#include "FlymapleWirish.h"

extern const AP_HAL::HAL& hal;

namespace AP_HAL {

void init()
{
}

void panic(const char *errormsg, ...)
{
    /* Suspend timer processes. We still want the timer event to go off
     * to run the _failsafe code, however. */
    // REVISIT: not tested on FLYMAPLE
    va_list ap;

    hal.scheduler->suspend_timer_procs();

    va_start(ap, errormsg);
    hal.console->vprintf(errormsg, ap);
    va_end(ap);
    hal.console->printf("\n");

    for(;;);
}

uint32_t micros()
{
    // Use function provided by libmaple.
    return ::micros();
}

uint32_t millis()
{
    // Use function provided by libmaple.
    return ::millis();
}

uint64_t millis64()
{
    return millis();
}

uint64_t micros64()
{
    // this is slow, but solves the problem with logging uint64_t timestamps
    uint64_t ret = millis();
    ret *= 1000ULL;
    ret += micros() % 1000;
    return ret;
}

} // namespace AP_HAL
