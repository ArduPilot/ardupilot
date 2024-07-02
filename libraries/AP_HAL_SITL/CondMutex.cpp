#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "CondMutex.h"
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// constructor
CondMutex::CondMutex()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
    pthread_cond_init(&cond, NULL);
}

void CondMutex::lock_and_wait(AP_HAL::CondMutex::Condition proc)
{
    if (hal.scheduler->in_main_thread()) {
        AP_HAL::panic("Bad CondMutex usage");
    }

    if (pthread_mutex_lock(&_lock)!= 0) {
        AP_HAL::panic("Bad CondMutex usage");
    }

    while (!proc()) {   // wait for proc to return true
        if (pthread_cond_wait(&cond, &_lock) != 0) {
            AP_HAL::panic("Bad CondMutex usage");
        }
    }
}

void CondMutex::lock_and_signal()
{
    if (pthread_mutex_lock(&_lock)!= 0) {
        AP_HAL::panic("Bad CondMutex usage");
    }
    pthread_cond_signal(&cond);
}

void CondMutex::unlock()
{
    if (pthread_mutex_unlock(&_lock) != 0) {
        AP_HAL::panic("Bad CondMutex usage");
    }
}

#endif  // CONFIG_HAL_BOARD
