#pragma once

#include "AP_HAL_Empty.h"

class Empty::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore() : _taken(false) {}
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
private:
    bool _taken;
};
