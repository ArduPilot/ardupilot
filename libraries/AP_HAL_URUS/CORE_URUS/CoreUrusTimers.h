#pragma once

#include "CORE_URUS_NAMESPACE.h"

#include <stdio.h>
#include <stdint.h>

class NSCORE_URUS::CLCoreUrusTimers {
public:
    CLCoreUrusTimers()
    {}

    virtual uint64_t get_core_hrdtime () = 0;
};
