#pragma once

#if defined(__CYGWIN__)

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusTimers.h"
#include <errno.h>

class CLCoreUrusTimers_Cygwin : public NSCORE_URUS::CLCoreUrusTimers {
public:
    CLCoreUrusTimers_Cygwin();
    uint64_t get_core_hrdtime () override;

private:

    void _measure_time_proccess();
    void _micro_sleep(uint32_t);
    uint64_t _micros64ts();
    uint64_t _micros64tv();
};

#endif // __CYGWIN__
