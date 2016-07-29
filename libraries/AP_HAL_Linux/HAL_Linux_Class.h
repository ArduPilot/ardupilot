#pragma once

#include <AP_HAL/AP_HAL.h>

class HAL_Linux : public AP_HAL::HAL {
public:
    HAL_Linux();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
