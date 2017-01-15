#pragma once

namespace NSCORE_URUS {

    class CLCORE_URUS;
    class CLCoreUrusTimers;
    class CLCoreUrusScheduler;

    const CLCORE_URUS& get_CORE();
    CLCoreUrusScheduler* get_scheduler();
}
