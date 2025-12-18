#pragma once

/// @file    AC_Custom_FastTask_Base.h
/// @brief   Base class implementation of custom fast task instance class

#include <AP_Common/AP_Common.h>

class AC_Custom_FastTask_Base
{
public:
    virtual ~AC_Custom_FastTask_Base() {}
    // The initialization function for custom fast task 
    virtual void init() = 0;
    // The loop function for custom fast task
    virtual void update() = 0;
};
