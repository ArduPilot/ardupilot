#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>

namespace Embox{
    
    class Semaphore : public AP_HAL::Semaphore {
    public:

        bool give() override;
        bool take(uint32_t timeout_ms) override;
        bool take_nonblocking() override;
        bool check_owner() { return true; }
    private:
        bool _taken;
    };
}