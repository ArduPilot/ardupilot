#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include <AP_HAL_YUNEEC.h>
#include "Semaphores.h"
#include "Scheduler.h"
#include <core_cmFunc.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

bool YUNEECSemaphore::give() {
    if (_taken) {
        _taken = false;
        return true;
    } else {
        return false;
    }
}

bool YUNEECSemaphore::take(uint32_t timeout_ms) {
    if (hal.scheduler->in_timerprocess()) {
        hal.scheduler->panic(PSTR("PANIC: YUNEECSemaphore::take used from "
                    "inside timer process"));
        return false; /* Never reached - panic does not return */
    }
    return _take_from_mainloop(timeout_ms);
}

bool YUNEECSemaphore::take_nonblocking() {
    if (hal.scheduler->in_timerprocess()) {
        return _take_nonblocking();
    } else {
        return _take_from_mainloop(0);
    }
}


bool YUNEECSemaphore::_take_from_mainloop(uint32_t timeout_ms) {
    /* Try to take immediately */
    if (_take_nonblocking()) {
        return true;
    } else if (timeout_ms == 0) {
        /* Return immediately if timeout is 0 */
        return false;
    }

    uint16_t timeout_ticks = timeout_ms << 3;
    do {
        /* Delay 1ms until we can successfully take, or we timed out */
        hal.scheduler->delay_microseconds(125);
        timeout_ticks--;
        if (_take_nonblocking()) {
            return true;
        }
    } while (timeout_ticks > 0);

    return false;
}

bool YUNEECSemaphore::_take_nonblocking() {
    bool result = false;
    __disable_irq();
    if (!_taken) {
        _taken = true;
        result = true;
    }
    __enable_irq();
    return result;
}

#endif
