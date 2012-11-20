/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_Semaphore.h"

extern const AP_HAL::HAL& hal;

// Constructor
AP_Semaphore::AP_Semaphore()
{
}

// get - to claim ownership of the semaphore 
bool AP_Semaphore::get(void* caller)
{
    bool result = false;
    hal.scheduler->begin_atomic();
    if( !_taken ) {
        _taken = true;
        _owner = caller;
        result = true;
    }
    hal.scheduler->end_atomic();
    return result;
}

// release - to give up ownership of the semaphore
// returns true if successfully released
bool AP_Semaphore::release(void* caller)
{

    // check legitimacy of release call
    if( caller != _owner ) {
        return false;
    }

    // if another process is waiting immediately call the provided kontinuation
    if( _waiting_k != NULL ) {

        // give ownership to waiting process
        _owner = _waiting_owner;
        AP_HAL::Proc k = _waiting_k;

        // clear waiting process
        _waiting_k = NULL;
        _waiting_owner = NULL;

        // callback
        k();
    }

    // give up the semaphore
    _taken = false;
    return true;
}

// call_on_release - returns true if caller successfully added to the queue to
// be called back
bool AP_Semaphore::call_on_release(void* caller, AP_HAL::Proc k)
{
    bool result = false;
    hal.scheduler->begin_atomic();
    if( _waiting_owner == NULL ) {
        _waiting_owner = caller;
        _waiting_k = k;
        result = true;
    }
    hal.scheduler->end_atomic();
    return result;
}
