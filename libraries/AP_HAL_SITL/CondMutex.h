#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/CondMutex.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

class HALSITL::CondMutex : public AP_HAL::CondMutex {
public:
    CondMutex();
    void lock_and_wait(AP_HAL::CondMutex::Condition condition) override;
    void lock_and_signal() override;
    void unlock() override;

protected:
    pthread_mutex_t _lock;
    pthread_cond_t cond;
};
