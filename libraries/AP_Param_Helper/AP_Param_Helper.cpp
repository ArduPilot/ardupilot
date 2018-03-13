#include "AP_Param_Helper.h"

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_NEEDS_PARAM_HELPER)

const AP_Param::GroupInfo AP_Param_Helper::var_info[] = {
#if defined(F4LIGHT_HAL_VARINFO)
    F4LIGHT_HAL_VARINFO
#endif

// only if board defines parameters
#ifdef BOARD_HAL_VARINFO
    BOARD_HAL_VARINFO
#endif
    AP_GROUPEND
};


extern const AP_HAL::HAL& hal;

AP_Param_Helper::AP_Param_Helper(bool f){
    f=!f;

    hal_param_helper=this;
    AP_Param::setup_object_defaults(this, var_info); // setup all params
}


AP_Param_Helper * hal_param_helper;

#endif
