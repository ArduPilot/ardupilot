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
    take_count--;
    if (pthread_mutex_unlock(&_lock) != 0) {
        AP_HAL::panic("Bad semaphore usage");
    }
    if (take_count == 0) {
        owner = (pthread_t)-1;
    }
    return true;
}

void Semaphore::check_owner() const
{
    // should probably make sure we're holding the semaphore here....
    if (owner != pthread_self()) {
        AP_HAL::panic("Wrong owner");
    }
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        if (pthread_mutex_lock(&_lock) == 0) {
            owner = pthread_self();
            take_count++;
            return true;
        }
        return false;
    }
    if (take_nonblocking()) {
        owner = pthread_self();
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        Scheduler::from(hal.scheduler)->set_in_semaphore_take_wait(true);
        hal.scheduler->delay_microseconds(200);
        Scheduler::from(hal.scheduler)->set_in_semaphore_take_wait(false);
        if (take_nonblocking()) {
            owner = pthread_self();
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms * 1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    if (pthread_mutex_trylock(&_lock) == 0) {
        owner = pthread_self();
        take_count++;
        return true;
    }
    return false;
}


/*
  binary semaphore using pthread condition variables
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
        if (hal.scheduler->in_main_thread() ||
            Scheduler::from(hal.scheduler)->semaphore_wait_hack_required()) {
            /*
              when in the main thread we need to do a busy wait to ensure
              the clock advances
            */
            uint64_t end_us = AP_HAL::micros64() + timeout_us;
            struct timespec ts {};
            do {
                if (pthread_cond_timedwait(&cond, &mtx._lock, &ts) == 0) {
                    pending = false;
                    return true;
                }
                hal.scheduler->delay_microseconds(10);
            } while (AP_HAL::micros64() < end_us);
            return false;
        }

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

#endif  // CONFIG_HAL_BOARD
