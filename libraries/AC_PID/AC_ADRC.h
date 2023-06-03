#pragma once

/// @file	AC_ADRC.h
/// @brief	Generic adaptive control algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AP_PIDInfo.h"

/// @class	AC_ADRC
/// @brief	ADRC control class

class AC_ADRC {
public:
    // Constructor for ADRC
    AC_ADRC(float initial_wc, float initial_wo, float initial_b0, int8_t initial_order, 
            float initial_delta = 0.0f, float initial_limit = 1.0f);
    

    CLASS_NO_COPY(AC_ADRC);

    //  update_all - set target and measured inputs to ADRC controller and calculate outputs
    //  target and error are filtered
    float update_all(float target, float measurement, float dt);

    // reset ESO
    void reset_eso(float measurement);

    const AP_PIDInfo& get_debug_info(void) const { return _debug_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Float _wc;          // Response bandwidth in rad/s
    AP_Float _wo;          // State estimation bandwidth in rad/s
    AP_Float _b0;          // Control gain
    AP_Float _limit;       // Saturation control limit 
    AP_Float _delta;       // Delta
    AP_Int8  _order;       // Control object model order

    // internal variables
    float _z1;
    float _z2;
    float _z3;
    
    AP_PIDInfo _debug_info;

private:
    const float default_wc;         
    const float default_wo;          
    const float default_b0;         
    const float default_limit;      
    const float default_delta;       
    const int8_t default_order;       
  
};