#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "Semaphores.h"

using namespace YUNEEC;

bool YUNEECSemaphore::give() {
    if (_taken) {
        _taken = false;
        return true;
    } else {
        return false;
    }
}

bool YUNEECSemaphore::take(uint32_t timeout_ms) {
    return take_nonblocking();
}

bool YUNEECSemaphore::take_nonblocking() {
    /* No syncronisation primitives to garuntee this is correct */
    if (!_taken) {
        _taken = true;
        return true;
    } else {
        return false;
    }
}

#endif
