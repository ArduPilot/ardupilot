// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Semaphore.h
/// @brief	class to ensure conflicts over shared resources are avoided

#ifndef __AP_SEMAPHORE_H__
#define __AP_SEMAPHORE_H__

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library

// the callback type for call_on_release. They are passed in a pointer to the semaphore
typedef void (*ap_semaphore_callback)();

/// @class      AP_Semaphore
class AP_Semaphore {
public:

    // Constructor
    AP_Semaphore();

    // get - to claim ownership of the semaphore
    virtual bool get(void* caller);

    // release - to give up ownership of the semaphore
    virtual bool release(void* caller);

    // call_on_release - returns true if caller successfully added to the queue to be called back
    virtual bool call_on_release(void* caller, ap_semaphore_callback callback_fn);

protected:
    bool        _taken;
    void*       _owner;
    void*       _waiting_owner;
    ap_semaphore_callback     _waiting_callback;       // call back procedure of process waiting for sempahore
};

#endif  // __AP_SEMAPHORE_H__
