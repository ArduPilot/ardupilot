#include "system.h"

uint16_t WEAK AP_HAL::millis16()
{
    return millis() & 0xFFFF;
}

uint16_t WEAK AP_HAL::micros16()
{
    return micros() & 0xFFFF;
}

void WEAK AP_HAL::dump_stack_trace()
{
    // stack dump not available on this platform
}
void WEAK AP_HAL::dump_core_file()
{
    // core dump not available on this platform
}
