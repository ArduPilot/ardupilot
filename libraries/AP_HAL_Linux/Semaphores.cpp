#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

bool LinuxSemaphore::give() 
{
    return pthread_mutex_unlock(&_lock) == 0;
}

bool LinuxSemaphore::take(uint32_t timeout_ms) 
{
    if (timeout_ms == 0) {
        return pthread_mutex_lock(&_lock) == 0;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = hal.scheduler->micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((hal.scheduler->micros64() - start) < timeout_ms*1000);
    return false;
}

bool LinuxSemaphore::take_nonblocking() 
{
    return pthread_mutex_trylock(&_lock) == 0;
}

#endif // CONFIG_HAL_BOARD
