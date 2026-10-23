#include "system.h"
#include "AP_HAL.h"
#include <AP_InternalError/AP_InternalError.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

uint16_t WEAK AP_HAL::millis16()
{
    return millis() & 0xFFFF;
}

uint16_t WEAK AP_HAL::micros16()
{
    return micros() & 0xFFFF;
}
