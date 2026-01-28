/// @file	AC_HELI_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_HELI_PID.h"

const AP_Param::GroupInfo AC_HELI_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, AC_HELI_PID, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 1, AC_HELI_PID, _ki, default_ki),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 2, AC_HELI_PID, _kd, default_kd),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 4, AC_HELI_PID, _kff, default_kff),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 5, AC_HELI_PID, _kimax, default_kimax),

    // 6 was for float FILT

    // @Param: ILMI
    // @DisplayName: I-term Leak Minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ILMI", 7, AC_HELI_PID, _leak_min, default_ilmi),

    // 8 was for float AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the target input (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 9, AC_HELI_PID, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the error (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 10, AC_HELI_PID, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the derivative (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 11, AC_HELI_PID, _filt_D_hz, default_filt_D_hz),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SMAX", 12, AC_HELI_PID, _slew_rate_max, default_slew_rate_max),

    // @Param: PDMX
    // @DisplayName: PD sum maximum
    // @Description: The maximum/minimum value that the sum of the P and D term can output
    // @User: Advanced
    AP_GROUPINFO("PDMX", 13, AC_HELI_PID, _kpdmax, 0),

    // @Param: D_FF
    // @DisplayName: PID Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_FF", 14, AC_HELI_PID, _kdff, default_kdff),

#if AP_FILTER_ENABLED
    // @Param: NTF
    // @DisplayName: PID Target notch filter index
    // @Description: PID Target notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("NTF", 15, AC_HELI_PID, _notch_T_filter, 0),

    // @Param: NEF
    // @DisplayName: PID Error notch filter index
    // @Description: PID Error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("NEF", 16, AC_HELI_PID, _notch_E_filter, 0),
#endif

    AP_GROUPEND
};

// Constructor
AC_HELI_PID::AC_HELI_PID(const AC_PID::Defaults &defaults,  float initial_ilmi) :
    AC_PID{defaults},
    default_ilmi(initial_ilmi)
{

    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _last_requested_rate = 0;
}

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

void AC_HELI_PID::update_leaky_i(float leak_rate)
{
    if (!is_zero(_ki)){

        // integrator does not leak down below Leak Min
        if (_integrator > _leak_min){
            _integrator -= (float)(_integrator - _leak_min) * leak_rate;
        } else if (_integrator < -_leak_min) {
            _integrator -= (float)(_integrator + _leak_min) * leak_rate;
        }

    }
}
