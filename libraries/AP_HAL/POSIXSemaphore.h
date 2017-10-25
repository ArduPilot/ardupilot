#pragma once

#include <pthread.h>

#include "Semaphores.h"

class AP_HAL::POSIXSemaphore : public AP_HAL::Semaphore {
public:
    POSIXSemaphore() {
        pthread_mutex_init(&_lock, nullptr);
    }
    bool give();
    virtual bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    pthread_mutex_t _lock;
};
