#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

class HALSITL::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore() {
        pthread_mutex_init(&_lock, nullptr);
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    pthread_mutex_t _lock;
};


