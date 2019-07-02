#pragma once

#include "AP_HAL_Namespace.h"

#include <AP_Common/AP_Common.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER 0

class AP_HAL::Semaphore {
public:
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;

    // a varient that blocks forever
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-result"
    virtual void take_blocking() { take(HAL_SEMAPHORE_BLOCK_FOREVER); };
    #pragma GCC diagnostic pop
    
    virtual bool give() = 0;
    virtual ~Semaphore(void) {}
};
