#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_PX4_Namespace.h"
#include <pthread.h>

class PX4::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
protected:
    pthread_mutex_t _lock;
};

class PX4::Semaphore_Recursive : public PX4::Semaphore {
public:
    Semaphore_Recursive();
};


