
#ifndef __AP_HAL_LINUX_SEMAPHORE_H__
#define __AP_HAL_LINUX_SEMAPHORE_H__

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AP_HAL_Linux.h"
#include <pthread.h>

class Linux::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore() {
        pthread_mutex_init(&_lock, NULL);
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    pthread_mutex_t _lock;
};
#endif // CONFIG_HAL_BOARD

#endif // __AP_HAL_LINUX_SEMAPHORE_H__
