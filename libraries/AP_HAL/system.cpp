#include "system.h"

uint16_t WEAK AP_HAL::millis16()
{
    return millis() & 0xFFFF;
}
