
#ifndef __AP_HAL_SEMAPHORES_H__
#define __AP_HAL_SEMAPHORES_H__

#include <AP_HAL_Namespace.h>

#include <limits.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER UINT32_MAX

class AP_HAL::Semaphore {
public:
    virtual bool take(uint32_t timeout_ms) = 0;
    virtual bool take_nonblocking() = 0;
    virtual bool give() = 0;
};

#endif  // __AP_HAL_SEMAPHORES_H__
