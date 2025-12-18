#pragma once

/// @file    AC_Custom_FastTask_Factory.h
/// @brief   Factory class for choosing the right instance of the custom fast tast object

#include "AC_Custom_FastTask_Base.h"
#ifdef USE_CUSTOM_FT_SUBFACTORY
#include "AC_CustomFT_SubFactory.h"
#else
#include "AC_Custom_FastTask_Empty.h"
#endif
class AC_Custom_FastTask_Factory
{
public:
    static AC_Custom_FastTask_Base* createInstance();
};