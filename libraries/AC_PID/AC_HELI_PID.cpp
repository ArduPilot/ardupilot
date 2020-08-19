/// @file	AC_HELI_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_HELI_PID.h"

const AP_Param::GroupInfo AC_HELI_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_HELI_PID, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_HELI_PID, _ki, 0),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    2, AC_HELI_PID, _kd, 0),

    // 3 was for uint16 IMAX

    // @Param: VFF
    // @DisplayName: Velocity FF FeedForward Gain
    // @Description: Velocity FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO("VFF",    4, AC_HELI_PID, _kff, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 5, AC_HELI_PID, _kimax, 0),

    // 6 was for float FILT

    // @Param: ILMI
    // @DisplayName: I-term Leak Minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ILMI", 7, AC_HELI_PID, _leak_min, AC_PID_LEAK_MIN),

    // 8 was for float AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTT", 9, AC_HELI_PID, _filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 10, AC_HELI_PID, _filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID D term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 11, AC_HELI_PID, _filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),

    AP_GROUPEND
};

/// Constructor for PID
AC_HELI_PID::AC_HELI_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, float dt) :
    AC_PID(initial_p, initial_i, initial_d, initial_ff, initial_imax, initial_filt_T_hz, initial_filt_E_hz, initial_filt_D_hz, dt)
{
    _last_requested_rate = 0;
}

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

void AC_HELI_PID::update_leaky_i(float leak_rate)
{
    if(!is_zero(_ki) && !is_zero(_dt)){

        // integrator does not leak down below Leak Min
        if (_integrator > _leak_min){
            _integrator -= (float)(_integrator - _leak_min) * leak_rate;
        } else if (_integrator < -_leak_min) {
            _integrator -= (float)(_integrator + _leak_min) * leak_rate;
        }

        _pid_info.I = _integrator;
    }
}
