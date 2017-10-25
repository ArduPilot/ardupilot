#include <AP_HAL/AP_HAL.h>

#include "POSIXSemaphores.h"

extern const AP_HAL::HAL& hal;

using namespace AP_HAL;

bool POSIXSemaphore::give()
{
    return pthread_mutex_unlock(&_lock) == 0;
}

bool POSIXSemaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return pthread_mutex_lock(&_lock) == 0;
    }
    if (take_nonblocking()) {
        return true;
    }
    const uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool POSIXSemaphore::take_nonblocking()
{
    return pthread_mutex_trylock(&_lock) == 0;
}
