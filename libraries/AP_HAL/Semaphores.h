
#ifndef __AP_HAL_SEMAPHORES_H__
#define __AP_HAL_SEMAPHORES_H__

#include <AP_HAL_Namespace.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER ((uint32_t) 0xFFFFFFFF)

class AP_HAL::Semaphore {
public:
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;
    virtual bool give() = 0;
};

#endif  // __AP_HAL_SEMAPHORES_H__
