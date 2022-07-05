#pragma once

/// @file    AC_CommandModel.h
/// @brief   ArduCopter Command Model Library

#include <AP_Param/AP_Param.h>

/*
  Command model parameters
 */

class AC_CommandModel {
public:
    AC_CommandModel(float initial_rate, float initial_expo, float initial_tc);

    // Accessors for parameters
    float get_rate_tc() const { return rate_tc; }
    float get_rate() const { return rate; }
    float get_expo() const { return expo; }

    // Set the max rate
    void set_rate(float input) { rate.set(input); }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Float rate_tc;       // rate time constant
    AP_Float rate;          // maximum rate
    AP_Float expo;          // expo shaping

};

