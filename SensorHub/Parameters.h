#pragma once

#include <AP_Common/AP_Common.h>

class Parameters {
public:
    static const uint16_t k_format_version = 1;
    static const uint16_t k_software_type = 5;

    enum {
        k_param_format_version = 0,
        k_param_software_type,
        k_param_scheduler,
        k_param_gps,
        k_param_barometer,
        k_param_compass,
        k_param_ins,
        k_param_serial_manager,
        k_param_BoardConfig,
    };

    AP_Int16 format_version;
    AP_Int16 software_type;
};

extern const AP_Param::Info var_info[];