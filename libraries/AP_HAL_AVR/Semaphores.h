
#ifndef __AP_HAL_AVR_SEMAPHORES_H__
#define __AP_HAL_AVR_SEMAPHORES_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRSemaphore : public AP_HAL::Semaphore {
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

#endif  // __AP_HAL_AVR_SEMAPHORES_H__
