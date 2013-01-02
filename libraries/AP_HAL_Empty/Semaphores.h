
#ifndef __AP_HAL_EMPTY_SEMAPHORE_H__
#define __AP_HAL_EMPTY_SEMAPHORE_H__

#include <AP_HAL_Empty.h>

class Empty::EmptySemaphore : public AP_HAL::Semaphore {
public:
    EmptySemaphore();
    // get - to claim ownership of the semaphore
    bool get(void* owner);
    // release - to give up ownership of the semaphore
    bool release(void* owner);
    // call_on_release - returns true if caller successfully added to the
    // queue to be called back
    bool call_on_release(void* caller, AP_HAL::Proc k);
private:
    void* _owner;
    AP_HAL::Proc _k;
};

#endif // __AP_HAL_EMPTY_SEMAPHORE_H__
