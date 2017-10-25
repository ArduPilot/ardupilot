#pragma once

#include <pthread.h>

#include "Semaphores.h"

class AP_HAL::POSIXSemaphore : public AP_HAL::Semaphore {
public:
    POSIXSemaphore() {
        pthread_mutex_init(&_lock, nullptr);
    }
    bool give() override;
    virtual bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
private:
    pthread_mutex_t _lock;
};
