#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "HAL_ESP32_Namespace.h"

class HALESP32::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
    void take_blocking();
protected:
    void*  handle;
};


class HALESP32::Semaphore_Recursive : public HALESP32::Semaphore {
public:
    Semaphore_Recursive();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
    void take_blocking();
};
