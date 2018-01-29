#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "Semaphores.h"
#include <nuttx/arch.h>

extern const AP_HAL::HAL& hal;

using namespace PX4;

bool Semaphore::take(uint32_t timeout_ms) 
{
    if (up_interrupt_context()) {
        // don't ever wait on a semaphore in interrupt context
        return take_nonblocking();
    }
    return POSIXSemaphore::take(timeout_ms);
}

#endif // CONFIG_HAL_BOARD
