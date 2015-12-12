#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux_Namespace.h"

class HAL_Linux : public AP_HAL::HAL {
public:
    HAL_Linux();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
