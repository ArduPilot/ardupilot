
#ifndef __AP_HAL_LINUX_SEMAPHORE_H__
#define __AP_HAL_LINUX_SEMAPHORE_H__

#include <AP_HAL_Linux.h>

class Linux::LinuxSemaphore : public AP_HAL::Semaphore {
public:
    LinuxSemaphore() : _taken(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    bool _taken;
};

#endif // __AP_HAL_LINUX_SEMAPHORE_H__
