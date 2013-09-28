
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
    while (timeout_ms-- > 0) {
        if (take_nonblocking()) return true;
        hal.scheduler->delay(1);
    }
    return take_nonblocking();
}

bool LinuxSemaphore::take_nonblocking() 
{
    return pthread_mutex_trylock(&_lock) == 0;
}
