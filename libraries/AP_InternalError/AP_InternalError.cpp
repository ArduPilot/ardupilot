#include "AP_InternalError.h"

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
    hal.util->persistent_data.internal_errors = internal_errors;
    hal.util->persistent_data.internal_error_count++;
}


namespace AP {

AP_InternalError &internalerror()
{
    return instance;
}

};
