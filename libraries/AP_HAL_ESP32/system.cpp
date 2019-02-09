#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/HAL_ESP32_Class.h>
#include <stdint.h>
#include "esp_timer.h"

namespace AP_HAL {

void panic(const char *errormsg, ...)
{
    va_list ap;

    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);

    while (1) {}
}

uint32_t micros()
{
    return micros64();
}

uint32_t millis()
{
    return millis64();
}

uint64_t micros64()
{
    return esp_timer_get_time();
}

uint64_t millis64()
{
    return micros64()/1000;
}

} // namespace AP_HAL

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    static const HAL_ESP32 hal;
    return hal;
}
