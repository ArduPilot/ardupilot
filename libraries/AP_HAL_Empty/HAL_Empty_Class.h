#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Empty_Namespace.h"

class HAL_Empty : public AP_HAL::HAL {
public:
    HAL_Empty();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
