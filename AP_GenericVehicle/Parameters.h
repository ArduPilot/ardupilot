#pragma once

#define AP_PARAM_VEHICLE_NAME genericvehicle

#include <AP_Param/AP_Param.h>

// Global parameter class.
//
class Parameters {
public:

    static const uint16_t        k_format_version = 120;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version =   0,
        k_param_vehicle =   1,
#if HAL_GCS_ENABLED
        k_param_sysid_this_mav =   2,
#endif

#if AP_BOARDCONFIG_SINGLETON_ENABLED
        // BoardConfig object
        k_param_BoardConfig = 3,
#endif
#if HAL_LOGGING_ENABLED
        k_param_log_bitmask = 4,
#endif
    };

    AP_Int16 format_version;
#if HAL_GCS_ENABLED
    AP_Int16 sysid_this_mav;
#endif

#if HAL_LOGGING_ENABLED
    AP_Int32 log_bitmask;
#endif

};
