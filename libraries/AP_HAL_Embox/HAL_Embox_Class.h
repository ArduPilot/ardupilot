#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Embox_Namespace.h"

class HAL_Embox : public AP_HAL::HAL {
public:
    HAL_Embox();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
