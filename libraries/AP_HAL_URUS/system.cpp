#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include "CORE_URUS/CORE_URUS.h"

#include <stdio.h>

extern const NSCORE_URUS::CLCORE_URUS& _urus_core;

namespace AP_HAL {

void init()
{
#if 0
    printf("system!\n");
#endif
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

uint64_t micros64()
{
    return _urus_core.timers->get_core_hrdtime();
}

uint64_t millis64()
{
    return _urus_core.timers->get_core_hrdtime() / 1000;
}

} // namespace AP_HAL

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
