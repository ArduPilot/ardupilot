/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Semaphore.h"

extern "C" {
#include <inttypes.h>
#include <stdint.h>
#include <avr/interrupt.h>
}
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WConstants.h"
#endif

// include some global constants
AP_Semaphore AP_Semaphore_spi3;

// Constructor
AP_Semaphore::AP_Semaphore()
{
}

// get - to claim ownership of the semaphore 
bool AP_Semaphore::get(void* caller)
{
    bool result = false;
    cli();
    if( !_taken ) {
        _taken = true;
        _owner = caller;
        result = true;
    }
    sei();
    return result;
}

// release - to give up ownership of the semaphore
// returns true if successfully released
bool AP_Semaphore::release(void* caller)
{
    ap_semaphore_callback callback_fn;

    // check legitimacy of release call
    if( caller != _owner ) {
        return false;
    }

    // if another process is waiting immediately call it's call back function
    if( _waiting_callback != NULL ) {

        // give ownership to waiting process
        _owner = _waiting_owner;
        callback_fn = _waiting_callback;

        // clear waiting process
        _waiting_callback = NULL;
        _waiting_owner = NULL;

        // callback
        callback_fn();
    }

    // give up the semaphore
    _taken = false;
    return true;
}

// call_on_release - returns true if caller successfully added to the queue to be called back
bool AP_Semaphore::call_on_release(void* caller, ap_semaphore_callback callback_fn)
{
    bool result = false;
    cli();
    if( _waiting_owner == NULL ) {
        _waiting_owner = caller;
        _waiting_callback = callback_fn;
        result = true;
    }
    sei();
    return result;
}