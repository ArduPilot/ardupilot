#pragma once

/// @file    AC_CommandModel.h
/// @brief   ArduCopter Command Model Library

#include <AP_Param/AP_Param.h>

/*
  Command model parameters
 */

class AC_CommandModel {
public:
    AC_CommandModel(void)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    float get_rate_tc() { return rate_tc; }
    float get_rate() { return rate; }
    float get_expo() { return expo; }

    void set_rate(float input) { rate = input; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Float rate_tc;
    AP_Float rate;
    AP_Float expo;

};

