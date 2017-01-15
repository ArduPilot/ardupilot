#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include "CORE_URUS_NAMESPACE.h"

class NSCORE_URUS::CLCoreUrusScheduler : public AP_HAL::Scheduler {
public:
    CLCoreUrusScheduler() {}
};
