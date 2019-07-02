#include "AP_InternalError.h"

// actually create the instance:
static AP_InternalError instance;

void AP_InternalError::error(const AP_InternalError::error_t e) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_HAL::panic("internal error %u", unsigned(e));
#endif
    internal_errors |= uint32_t(e);
}


namespace AP {

AP_InternalError &internalerror()
{
    return instance;
}

};
