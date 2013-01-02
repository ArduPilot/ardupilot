
#ifndef __AP_HAL_SEMAPHORE_H__
#define __AP_HAL_SEMAPHORE_H__

#include <AP_HAL_Namespace.h>

class AP_HAL::Semaphore {
public:
    // get - to claim ownership of the semaphore
    virtual bool get(void* caller) = 0;

    // release - to give up ownership of the semaphore
    virtual bool release(void* caller) = 0;

    // call_on_release - returns true if caller successfully added to the
    // queue to be called back
    virtual bool call_on_release(void* caller, AP_HAL::Proc k) = 0;
};

#endif  // __AP_HAL_SEMAPHORE_H__
