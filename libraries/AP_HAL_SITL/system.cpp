#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

using HALSITL::Scheduler;

namespace AP_HAL {

static struct {
    struct timeval start_time;
} state;

void init()
{
    gettimeofday(&state.start_time, nullptr);
}

void panic(const char *errormsg, ...)
{
    va_list ap;

    fflush(stdout);
    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);
    printf("\n");

    for(;;);
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

/*
  we define a millis16() here to avoid an issue with sitl builds in cygwin
 */
uint16_t millis16()
{
    return millis64() & 0xFFFF;
}
    
uint64_t micros64()
{
    const HALSITL::Scheduler* scheduler = HALSITL::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec;
    }

    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e6 * ((tp.tv_sec + (tp.tv_usec * 1.0e-6)) -
                            (state.start_time.tv_sec +
                             (state.start_time.tv_usec * 1.0e-6)));
    return ret;
}

uint64_t millis64()
{
    const HALSITL::Scheduler* scheduler = HALSITL::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec / 1000;
    }

    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) -
                          (state.start_time.tv_sec +
                           (state.start_time.tv_usec*1.0e-6)));
    return ret;
}

} // namespace AP_HAL
