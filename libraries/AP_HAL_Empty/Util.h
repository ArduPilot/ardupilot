#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Empty_Namespace.h"

class Empty::Util : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return false; }
};
