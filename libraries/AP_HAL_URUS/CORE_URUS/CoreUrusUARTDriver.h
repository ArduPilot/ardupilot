#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include "CORE_URUS_NAMESPACE.h"

class NSCORE_URUS::CLCoreUrusUARTDriver : public AP_HAL::UARTDriver {
public:
    CLCoreUrusUARTDriver() {}
    virtual void _timer_tick(void) = 0;
};
