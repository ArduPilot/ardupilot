// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Semaphore.h
/// @brief	class to ensure conflicts over shared resources are avoided

#ifndef __AP_SEMAPHORE_H__
#define __AP_SEMAPHORE_H__

#include <AP_HAL.h>

/// @class      AP_Semaphore
class AP_Semaphore {
public:

    // Constructor
    AP_Semaphore();

    // get - to claim ownership of the semaphore
    virtual bool get(void* caller);

    // release - to give up ownership of the semaphore
    virtual bool release(void* caller);

    // call_on_release - returns true if caller successfully added to the
    // queue to be called back
    virtual bool call_on_release(void* caller, AP_HAL::Proc k);

protected:
    bool        _taken;
    void*       _owner;
    void*       _waiting_owner;

    // procedure of process waiting for sempahore
    AP_HAL::Proc _waiting_k;
};

#endif  // __AP_SEMAPHORE_H__
