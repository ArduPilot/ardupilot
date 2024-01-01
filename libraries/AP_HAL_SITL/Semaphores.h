#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

class HALSITL::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

    void check_owner() const;  // asserts that current thread owns semaphore

protected:
    pthread_mutex_t _lock;
    pthread_t owner;

    // keep track the recursion level to ensure we only disown the
    // semaphore once we're done with it
    uint8_t take_count;

#if AP_DEADLOCK_DETECTOR_ENABLED
    AP_HAL::Semaphore *get_sem_list() override;
    void set_sem_list(AP_HAL::Semaphore *sem) override;
#endif
};
