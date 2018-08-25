#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "Semaphores.h"
#include <nuttx/arch.h>

extern const AP_HAL::HAL& hal;

using namespace PX4;

// construct a semaphore
Semaphore::Semaphore()
{
    pthread_mutex_init(&_lock, nullptr);
}

// construct a recursive semaphore (allows a thread to take it more than once)
Semaphore_Recursive::Semaphore_Recursive()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
}

bool Semaphore::give() 
{
    return pthread_mutex_unlock(&_lock) == 0;
}

bool Semaphore::take(uint32_t timeout_ms) 
{
    if (up_interrupt_context()) {
        // don't ever wait on a semaphore in interrupt context
        return take_nonblocking();
    }
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
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
