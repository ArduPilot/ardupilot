#include "system.h"

extern const AP_HAL::HAL &hal;

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

static uint32_t last_loop_ms;

uint32_t AP_HAL::loop_ms(void)
{
    if (last_loop_ms == 0) {
        return millis();
    }
    return last_loop_ms;
}

void AP_HAL::set_loop_ms(uint32_t ms)
{
    last_loop_ms = ms;
}
