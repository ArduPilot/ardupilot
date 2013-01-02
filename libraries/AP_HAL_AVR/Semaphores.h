
#ifndef __AP_HAL_AVR_SEMAPHORE_H__
#define __AP_HAL_AVR_SEMAPHORE_H__

#include <AP_HAL.h>
#include <AP_HAL_AVR_Namespace.h>

class AP_HAL_AVR::AVRSemaphore : public AP_HAL::Semaphore {
public:
    // Constructor
    AVRSemaphore();

    // get - to claim ownership of the semaphore
    bool get(void* caller);

    // release - to give up ownership of the semaphore
    bool release(void* caller);

    // call_on_release - returns true if caller successfully added to the
    // queue to be called back
    bool call_on_release(void* caller, AP_HAL::Proc k);

protected:
    bool        _taken;
    void*       _owner;
    void*       _waiting_owner;

    // procedure of process waiting for sempahore
    AP_HAL::Proc _waiting_k;
};

#endif  // __AP_HAL_AVR_SEMAPHORE_H__
