#include <stdarg.h>
#include <AP_HAL.h>
#include "AP_InertialSensor_UserInteract_Stream.h"

extern const AP_HAL::HAL& hal;

uint8_t AP_InertialSensor_UserInteractStream::blocking_read() {
    /* Wait for input to be available */
    while(!_s->available()) {
        hal.scheduler->delay(20);
    }
    /* Grab first character */
    uint8_t ret = (uint8_t) _s->read();
    /* Clear all available input */
    while (_s->available()) {
        _s->read();
    }
    return ret;
}

void AP_InertialSensor_UserInteractStream::println_P(const prog_char_t* str) {
    _s->println_P(str);
}

void AP_InertialSensor_UserInteractStream::_printf_P(
        const prog_char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    _s->vprintf_P(fmt, ap);
    va_end(ap);
}

