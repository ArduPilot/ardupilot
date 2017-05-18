#include "AC_InputManager_Heli.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_InputManager_Heli::var_info[] = {

    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_InputManager, 0),

    // @Param: STAB_COL_1
    // @DisplayName: Stabilize Mode Collective Point 1
    // @Description: Helicopter's minimum collective pitch setting at zero throttle input in Stabilize mode
    // @Range: 0 500
    // @Units: d%
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_1",    1, AC_InputManager_Heli, _heli_stab_col_min, AC_ATTITUDE_HELI_STAB_COLLECTIVE_MIN_DEFAULT),

    // @Param: STAB_COL_2
    // @DisplayName: Stabilize Mode Collective Point 2
    // @Description: Helicopter's collective pitch setting at mid-low throttle input in Stabilize mode
    // @Range: 0 500
    // @Units: d%
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_2",    2, AC_InputManager_Heli, _heli_stab_col_low, AC_ATTITUDE_HELI_STAB_COLLECTIVE_LOW_DEFAULT),

    // @Param: STAB_COL_3
    // @DisplayName: Stabilize Mode Collective Point 3
    // @Description: Helicopter's collective pitch setting at mid-high throttle input in Stabilize mode
    // @Range: 500 1000
    // @Units: d%
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_3",    3, AC_InputManager_Heli, _heli_stab_col_high, AC_ATTITUDE_HELI_STAB_COLLECTIVE_HIGH_DEFAULT),

    // @Param: STAB_COL_4
    // @DisplayName: Stabilize Mode Collective Point 4
    // @Description: Helicopter's maximum collective pitch setting at full throttle input in Stabilize mode
    // @Range: 500 1000
    // @Units: d%
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_4",    4, AC_InputManager_Heli, _heli_stab_col_max, AC_ATTITUDE_HELI_STAB_COLLECTIVE_MAX_DEFAULT),

    // @Param: ACRO_COL_EXP
    // @DisplayName: Acro Mode Collective Expo
    // @Description: Used to soften collective pitch inputs near center point in Acro mode.
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @User: Advanced
    AP_GROUPINFO("ACRO_COL_EXP",    5, AC_InputManager_Heli, _acro_col_expo, 0),

    AP_GROUPEND
};

// get_pilot_desired_collective - rescale's pilot collective pitch input in Stabilize and Acro modes
float AC_InputManager_Heli::get_pilot_desired_collective(int16_t control_in)
{
    float slope_low, slope_high, slope_range, slope_run, scalar;
    float stab_col_out, acro_col_out;

    // calculate stabilize collective value which scales pilot input to reduced collective range
    // code implements a 3-segment curve with knee points at 40% and 60% throttle input
    if (control_in < 400){
        slope_low = _heli_stab_col_min / 1000.0f;
        slope_high = _heli_stab_col_low / 1000.0f;
        slope_range = 0.4f;
        slope_run = control_in / 1000.0f;
    } else if(control_in <600){
        slope_low = _heli_stab_col_low / 1000.0f;
        slope_high = _heli_stab_col_high / 1000.0f;
        slope_range = 0.2f;
        slope_run = (control_in - 400) / 1000.0f;
    } else {
        slope_low = _heli_stab_col_high / 1000.0f;
        slope_high = _heli_stab_col_max / 1000.0f;
        slope_range = 0.4f;
        slope_run = (control_in - 600) / 1000.0f;
    }    

    scalar = (slope_high - slope_low)/slope_range;
    stab_col_out = slope_low + slope_run * scalar;
    stab_col_out = constrain_float(stab_col_out, 0.0f, 1.0f);

    //
    // calculate expo-scaled acro collective
    // range check expo
    if (_acro_col_expo > 1.0f) {
        _acro_col_expo = 1.0f;
    }

    if (_acro_col_expo <= 0.0f) {
        acro_col_out = control_in / 1000.0f;
    } else {
        // expo variables
        float col_in, col_in3, col_out;
        col_in = (float)(control_in-500)/500.0f;
        col_in3 = col_in*col_in*col_in;
        col_out = (_acro_col_expo * col_in3) + ((1.0f-_acro_col_expo)*col_in);
        acro_col_out = 0.5f + col_out*0.5f;
    }
    acro_col_out = constrain_float(acro_col_out, 0.0f, 1.0f);

    // ramp to and from stab col over 1/2 second
    if (_im_flags_heli.use_stab_col && (_stab_col_ramp < 1.0f)){
        _stab_col_ramp += 2.0f/(float)_loop_rate;
    } else if(!_im_flags_heli.use_stab_col && (_stab_col_ramp > 0.0f)){
        _stab_col_ramp -= 2.0f/(float)_loop_rate;
    }
    _stab_col_ramp = constrain_float(_stab_col_ramp, 0.0f, 1.0f);

    // scale collective output smoothly between acro and stab col
    float collective_out;
    collective_out = (float)((1.0f-_stab_col_ramp)*acro_col_out + _stab_col_ramp*stab_col_out);
    collective_out = constrain_float(collective_out, 0.0f, 1.0f);

    return collective_out;
}


