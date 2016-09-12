#include <stdarg.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include <sys/timespec.h>
#include <dspal_time.h>
#include "replace.h"
#include <fenv.h>

extern const AP_HAL::HAL& hal;

namespace AP_HAL {

static struct {
    uint64_t start_time;
} state;

void init()
{
    state.start_time = micros64();
    // we don't want exceptions in flight code. That is the job of SITL
    feclearexcept(FE_OVERFLOW | FE_DIVBYZERO | FE_INVALID);
}

void panic(const char *errormsg, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, errormsg);
    vsnprintf(buf, sizeof(buf), errormsg, ap);
    va_end(ap);
    HAP_PRINTF(buf);
    usleep(2000000);
    hal.rcin->deinit();
    exit(1);
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t ret = ts.tv_sec*1000*1000ULL + ts.tv_nsec/1000U;
    ret -= state.start_time;
    return ret;
}

uint64_t millis64()
{
    return micros64() / 1000;
}

} // namespace AP_HAL
