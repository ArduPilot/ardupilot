#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_UserInteract_Stream.h"

extern const AP_HAL::HAL& hal;

bool AP_InertialSensor_UserInteractStream::blocking_read() 
{
    /* Wait for input to be available */
    while(!_s->available()) {
        hal.scheduler->delay(20);
    }
    /* Clear all available input */
    while (_s->available()) {
        _s->read();
    }
    return true;
}

void AP_InertialSensor_UserInteractStream::printf(
        const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _s->vprintf(fmt, ap);
    va_end(ap);
}

