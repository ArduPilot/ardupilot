#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "Semaphores.h"
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// construct a semaphore
Semaphore::Semaphore()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
}


bool Semaphore::give()
{
    if (pthread_mutex_unlock(&_lock) != 0) {
        AP_HAL::panic("Bad semaphore usage");
    }
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return pthread_mutex_lock(&_lock) == 0;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        Scheduler::from(hal.scheduler)->set_in_semaphore_take_wait(true);
        hal.scheduler->delay_microseconds(200);
        Scheduler::from(hal.scheduler)->set_in_semaphore_take_wait(false);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms * 1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    return pthread_mutex_trylock(&_lock) == 0;
}

#endif  // CONFIG_HAL_BOARD
