#pragma once

#include <AP_Common/AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    enum {
        k_param_dummy,
        k_param_barometer,
        k_param_ins,
        k_param_ahrs,
        k_param_airspeed,
        k_param_NavEKF2,
        k_param_compass,
        k_param_logger,
        k_param_NavEKF3,
        k_param_gps,
        k_param_compass_stop_ms,
    };
    AP_Int8 dummy;
    AP_Int32 compass_stop_ms;
};

extern const AP_Param::Info var_info[];
