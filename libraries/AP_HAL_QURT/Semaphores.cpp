#include <AP_HAL/AP_HAL.h>

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace QURT;

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
    return pthread_mutex_unlock(&_lock) == 0;
}

bool Semaphore::check_owner()
{
    return owner == pthread_self();
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        auto ok = pthread_mutex_lock(&_lock) == 0;
        if (ok) {
            owner = pthread_self();
        }
        return ok;
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
    const auto ok = pthread_mutex_trylock(&_lock) == 0;
    if (ok) {
        owner = pthread_self();
    }
    return ok;
}

/*
  binary semaphore using condition variables
 */

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    pthread_cond_init(&cond, NULL);
    pending = initial_state;
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        struct timespec ts;
        if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
            return false;
        }
        ts.tv_sec += timeout_us/1000000UL;
        ts.tv_nsec += (timeout_us % 1000000U) * 1000UL;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000L;
        }
        if (pthread_cond_timedwait(&cond, &mtx._lock, &ts) != 0) {
            return false;
        }
    }
    pending = false;
    return true;
}

bool BinarySemaphore::wait_blocking(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        if (pthread_cond_wait(&cond, &mtx._lock) != 0) {
            return false;
        }
    }
    pending = false;
    return true;
}

void BinarySemaphore::signal(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        pending = true;
        pthread_cond_signal(&cond);
    }
}
