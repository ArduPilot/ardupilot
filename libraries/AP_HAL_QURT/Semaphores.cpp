#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace QURT;

bool Semaphore::give() 
{
    return pthread_mutex_unlock(&_lock) == 0;
}

bool Semaphore::take(uint32_t timeout_ms) 
{
    if (timeout_ms == 0) {
        return pthread_mutex_lock(&_lock) == 0;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool Semaphore::take_nonblocking() 
{
    return pthread_mutex_trylock(&_lock) == 0;
}

#endif // CONFIG_HAL_BOARD
