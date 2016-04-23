#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include "Scheduler.h"
#include "wirish/wirish.h"

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
    uint32 fms, lms;
    uint32 cycle_cnt;
    uint32 res;
    do {
        // make sure millis() return the same value before and after
        // getting the systick count
        fms = millis();
        cycle_cnt = systick_get_count();
        lms = millis();
    } while (lms != fms);

#define US_PER_MS               1000
    /* SYSTICK_RELOAD_VAL is 1 less than the number of cycles it
       actually takes to complete a SysTick reload */
    res = (fms * US_PER_MS) +
        (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND;

    return res;
#undef US_PER_MS

}

uint32_t millis()
{
    // Use function provided by libmaple.
    return systick_uptime();
}

uint64_t millis64(){
    return systick_uptime();
}

uint64_t micros64(){
    uint64 fms, lms;
    uint32 cycle_cnt;
    uint64 res;
    do {
        // make sure millis() return the same value before and after
        // getting the systick count
        fms = systick_uptime();
        cycle_cnt = systick_get_count();
        lms = systick_uptime();
    } while (lms != fms);

#define US_PER_MS               1000
    //  SYSTICK_RELOAD_VAL is 1 less than the number of cycles it    actually takes to complete a SysTick reload 
    res = (fms * US_PER_MS) +
        (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND;

    return res;
#undef US_PER_MS

}

} // namespace AP_HAL

