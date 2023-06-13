#pragma once

#include "AP_HAL_Namespace.h"

#include <AP_Common/AP_Common.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER 0

#ifndef AP_DEADLOCK_DETECTOR_ENABLED
#define AP_DEADLOCK_DETECTOR_ENABLED 0
#endif

class AP_HAL::Semaphore {
public:

    Semaphore() {
#if AP_DEADLOCK_DETECTOR_ENABLED
        total_sem++;
#endif
    }

    // do not allow copying
    CLASS_NO_COPY(Semaphore);

    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;

    // a variant that blocks forever
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-result"
    virtual void take_blocking() { take(HAL_SEMAPHORE_BLOCK_FOREVER); };
    #pragma GCC diagnostic pop
    
    virtual bool give() = 0;
    virtual ~Semaphore(void) {}

protected:

#if AP_DEADLOCK_DETECTOR_ENABLED
    /*
      the deadlock detector works iteratively, with each use of a
      mutex narrowing the range of the ideal lock order.

      It is based on the principle that for a firmware without lock
      order bugs there is an ideal ordering number for each mutex,
      where you never hold a mutex with a higher number when taking a
      lower numbered mutex. With a system as complex as ArduPilot it
      isn't possible to label mutexes with this ideal lock order
      number at compile time, but we can learn it at run time. Each
      time a mutex is taken we know that the number must be higher
      than the parent mutex in the chain of mutexes already taken in
      this thread. Similarly on release we can constrain the max
      value. We maintain a ofs_min/ofs_max pair which is iteratively
      moved towards the ideal value. We know we have a lock ordering
      bug if the min is greater than the max.

      To use this feature you need to attach a debugger and add a
      hardware watch point on sem_error_count
     */
    virtual AP_HAL::Semaphore *get_sem_list() { return nullptr; }
    virtual void set_sem_list(AP_HAL::Semaphore *sem) {};
    Semaphore *next;
    uint32_t depth;
    static uint32_t total_sem;
    int16_t ofs_min;
    int16_t ofs_max;
    void push_list(void);
    void pop_list(void);
#endif
};

/*
  a method to make semaphores less error prone. The WITH_SEMAPHORE()
  macro will block forever for a semaphore, and will automatically
  release the semaphore when it goes out of scope

  Note that we have two types of semaphores. A normal semaphore can
  only be taken once. A recursive semaphore allows for the thread
  holding the semaphore to take it again. It must be released the same
  number of times it is taken.

  The WITH_SEMAPHORE() macro can be used with either type of semaphore
 */

class WithSemaphore {
public:
    WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line);
    WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line);

    ~WithSemaphore();
private:
    AP_HAL::Semaphore &_mtx;
};

// From: https://stackoverflow.com/questions/19666142/why-is-a-level-of-indirection-needed-for-this-concatenation-macro
#define WITH_SEMAPHORE( sem ) JOIN( sem, __AP_LINE__, __COUNTER__ )

#define JOIN( sem, line, counter ) _DO_JOIN( sem, line, counter )
#define _DO_JOIN( sem, line, counter ) WithSemaphore _getsem ## counter(sem, line)
