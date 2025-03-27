#pragma once

/// @file    AC_Simulink_Base.h
/// @brief   Base class implementation of Simulink integration

#include <AP_Common/AP_Common.h>

class AC_Simulink_Base {
public:
    virtual ~AC_Simulink_Base() {}
    virtual void init() = 0;
    virtual void update() = 0;
    virtual void reset() = 0;
};
