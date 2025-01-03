#pragma once

#include "AP_HAL_Namespace.h"

#include <AP_Common/AP_Common.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER 0

class AP_HAL::Semaphore {
public:

    Semaphore() {}

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
};

/*
  a method to make semaphores less error prone. The WITH_SEMAPHORE()
  macro will block forever for a semaphore, and will automatically
  release the semaphore when it goes out of scope

  All semaphores are recursive. This allows for the thread holding
  the semaphore to take it again. It must be released the same number
  of times it is taken.

  The WITH_SEMAPHORE() macro cannot be used with binary semaphores
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

/*
  a binary semaphore
 */
class AP_HAL::BinarySemaphore {
public:
    /*
      create a binary semaphore. initial_state determines if a wait()
      immediately after creation would block. If initial_state is true
      then it won't block, if initial_state is false it will block
     */
    BinarySemaphore(bool initial_state=false) {}

    // do not allow copying
    CLASS_NO_COPY(BinarySemaphore);

    virtual bool wait(uint32_t timeout_us) WARN_IF_UNUSED = 0 ;
    virtual bool wait_blocking() = 0;
    virtual bool wait_nonblocking() { return wait(0); }

    virtual void signal() = 0;
    virtual void signal_ISR() { signal(); }
    
    virtual ~BinarySemaphore(void) {}
};
