
/// @file    AC_Custom_FastTask_Factory.cpp
/// @brief   Factory class for choosing the right instance of the custom fast tast object

#include "AC_Custom_FastTask_Factory.h"

AC_Custom_FastTask_Base* AC_Custom_FastTask_Factory::createInstance()
{
#ifdef USE_CUSTOM_FT_SUBFACTORY
    return CustomFT_SubFactory::createInstance();
#else
    return new AC_Custom_FastTask_Empty();
#endif
}