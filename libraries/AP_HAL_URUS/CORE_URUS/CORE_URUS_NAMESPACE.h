#pragma once

#include <stdint.h>

namespace NSCORE_URUS {

    class CLCORE_URUS;
    class CLCoreUrusTimers;
    class CLCoreUrusScheduler;
    class CLCoreUrusUARTDriver;

    const CLCORE_URUS& get_CORE();
    CLCoreUrusScheduler* get_scheduler();
    CLCoreUrusUARTDriver* get_uartDriver();
}
