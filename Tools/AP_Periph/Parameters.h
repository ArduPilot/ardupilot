#pragma once

#include <AP_Common/AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    static const uint16_t k_format_version = 2;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_gps,
        k_param_compass,
        k_param_can_node,
        k_param_can_baudrate,
        k_param_baro,
    };

    AP_Int16 format_version;
    AP_Int16 can_node;
    AP_Int32 can_baudrate;

    Parameters() {}
};

extern const AP_Param::Info var_info[];
