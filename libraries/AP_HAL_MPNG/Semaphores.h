
#ifndef __AP_HAL_MPNG_SEMAPHORES_H__
#define __AP_HAL_MPNG_SEMAPHORES_H__

#include <AP_HAL.h>
#include <AP_HAL_MPNG_Namespace.h>

class MPNG::AVRSemaphore : public AP_HAL::Semaphore {
public:
    AVRSemaphore();

    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();

protected:
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();

    volatile bool _taken;
};

#endif  // __AP_HAL_MPNG_SEMAPHORES_H__
