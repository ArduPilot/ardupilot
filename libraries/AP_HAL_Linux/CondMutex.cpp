#include <AP_HAL/AP_HAL.h>

#include "CondMutex.h"
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

// constructor
CondMutex::CondMutex()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
    pthread_cond_init(&cond, NULL);
}

void CondMutex::lock_and_wait(AP_HAL::CondMutex::Condition proc)
{
    pthread_mutex_lock(&_lock);

    while (!proc()) {   // wait for proc to return true
        pthread_cond_wait(&cond, &_lock);
    }
}

void CondMutex::lock_and_signal()
{
    pthread_mutex_lock(&_lock);
    pthread_cond_signal(&cond);
}

void CondMutex::unlock()
{
    pthread_mutex_unlock(&_lock);
}
