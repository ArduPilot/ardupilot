// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

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
        k_param_NavEKF,
        k_param_compass
    };
    AP_Int8 dummy;
};

extern const AP_Param::Info var_info[];

#endif // PARAMETERS_H
