#pragma once

#include "AP_HAL_Namespace.h"

#include <AP_HAL/utility/functor.h>
#include <AP_Common/AP_Common.h>

class AP_HAL::CondMutex {
public:
    CondMutex() {}

    // do not allow copying
    CLASS_NO_COPY(CondMutex);

    FUNCTOR_TYPEDEF(Condition, bool);

    virtual void lock_and_wait(Condition condition) = 0;
    virtual void lock_and_signal() = 0;
    virtual void unlock() = 0;

    virtual ~CondMutex(void) {}

};
