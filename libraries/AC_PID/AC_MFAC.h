#pragma once

/// @file AC_MFAC.h
/// @brief Adaptive controller

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Math/vector2.h>
#include "AP_PIDInfo.h"

class AC_MFAC{
public:

    // Constructor for MFAC
    AC_MFAC(float initial_lamada,float initial_kr,float initial_eplise,float dt);

    CLASS_NO_COPY(AC_MFAC);

    // set time step in seconds
    void set_dt(float dt) { _dt = dt; }

    // update_all  - set target and measured inputs to MFAC controller and calculate output
    float update_all(const float target,const float measurement,const bool wrap2pi);

    // reset controller
    void reset(const float measurement);

    const AP_PIDInfo& get_debug_info(void) const { return _debug_info; }

     // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Float _lamada;
    AP_Float _mu;
    AP_Float _yita;
    AP_Float _eplise;
    AP_Float _kr;
    AP_Float _rou[2];
    AP_Float _fai[2];
    AP_Float _limit;

    // internal variables
    float     _dt;             // timestep in seconds

    // state variables
    Vector2f _vec_fai;
    Vector2f _vec_delu;
    float    _control_cmd{0.0f};
    float    _measurement{0.0f};


    AP_PIDInfo _debug_info;

};