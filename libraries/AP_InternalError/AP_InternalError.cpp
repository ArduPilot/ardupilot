#include "AP_InternalError.h"

#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// actually create the instance:
static AP_InternalError instance;

void AP_InternalError::error(const AP_InternalError::error_t e) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    switch (e) {
    case AP_InternalError::error_t::watchdog_reset:
    case AP_InternalError::error_t::main_loop_stuck:
        // don't panic on these to facilitate watchdog testing
        break;
    default:
        AP_HAL::panic("internal error %u", unsigned(e));
    }
#endif
    internal_errors |= uint32_t(e);
    total_error_count++;

    hal.util->persistent_data.internal_errors = internal_errors;
    hal.util->persistent_data.internal_error_count = total_error_count;
}

namespace AP {

AP_InternalError &internalerror()
{
    return instance;
}

};

// stack overflow hook for low level RTOS code, C binding
void AP_stack_overflow(const char *thread_name)
{
    static bool done_stack_overflow;
    AP::internalerror().error(AP_InternalError::error_t::stack_overflow);
    if (!done_stack_overflow) {
        // we don't want to record the thread name more than once, as
        // first overflow can trigger a 2nd
        strncpy(hal.util->persistent_data.thread_name4, thread_name, 4);
        done_stack_overflow = true;
    }
    hal.util->persistent_data.fault_type = 42; // magic value
    if (!hal.util->get_soft_armed()) {
        AP_HAL::panic("stack overflow %s\n", thread_name);
    }
}
