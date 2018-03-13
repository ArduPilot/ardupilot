/*
    a helper class to give ability to HAL to have own parameters

*/
#pragma once

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_NEEDS_PARAM_HELPER)
#include <AP_Param/AP_Param.h>

class AP_Param_Helper
{
public:
    AP_Param_Helper(bool f);

    static const AP_Param::GroupInfo var_info[];

// common HAL params
#if defined(F4LIGHT_HAL_PARAMS)
    F4LIGHT_HAL_PARAMS
#endif
    
// per board params - only if defined
#ifdef BOARD_HAL_PARAMS
    BOARD_HAL_PARAMS
#endif

};


extern AP_Param_Helper * hal_param_helper;

#endif
