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
    };

    AP_Int16 format_version;
};
