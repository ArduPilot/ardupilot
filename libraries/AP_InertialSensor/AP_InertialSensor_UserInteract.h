#pragma once

#include <AP_Common/AP_Common.h>

/* Pure virtual interface class */
class AP_InertialSensor_UserInteract {
public:
    virtual bool blocking_read() = 0;
    virtual void printf(const char *, ...) FMT_PRINTF(2, 3) = 0;
};
