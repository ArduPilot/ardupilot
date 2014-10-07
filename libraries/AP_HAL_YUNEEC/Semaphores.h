
#ifndef __AP_HAL_YUNEEC_SEMAPHORE_H__
#define __AP_HAL_YUNEEC_SEMAPHORE_H__

#include <AP_HAL_YUNEEC.h>

class YUNEEC::YUNEECSemaphore : public AP_HAL::Semaphore {
public:
    YUNEECSemaphore() : _taken(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();

private:
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();
    bool _taken;
};

#endif // __AP_HAL_YUNEEC_SEMAPHORE_H__
