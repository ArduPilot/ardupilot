#include "AC_InputManager_Heli.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_InputManager_Heli::var_info[] = {

    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_InputManager, 0),

    // Indicies 1-4 (STAB_COL_1 thru STAB_COL_4) have been replaced. 

    // @Param: ACRO_COL_EXP
    // @DisplayName: Acro Mode Collective Expo
    // @Description: Used to soften collective pitch inputs near center point in Acro mode.
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 0.95
    // @User: Advanced
    AP_GROUPINFO("ACRO_COL_EXP",    5, AC_InputManager_Heli, _acro_col_expo, 0),

    // @Param: STB_COL_1
    // @DisplayName: Stabilize Collective Low
    // @Description: Helicopter's minimum collective pitch setting at zero collective stick input in Stabilize mode.  Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STB_COL_1",    6, AC_InputManager_Heli, _heli_stab_col_min, AC_ATTITUDE_HELI_STAB_COLLECTIVE_MIN_DEFAULT),

    // @Param: STB_COL_2
    // @DisplayName: Stabilize Collective Mid-Low
    // @Description: Helicopter's collective pitch setting at mid-low (40%) collective stick input in Stabilize mode. Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STB_COL_2",    7, AC_InputManager_Heli, _heli_stab_col_low, AC_ATTITUDE_HELI_STAB_COLLECTIVE_LOW_DEFAULT),

    // @Param: STB_COL_3
    // @DisplayName: Stabilize Collective Mid-High
    // @Description: Helicopter's collective pitch setting at mid-high (60%) collective stick input in Stabilize mode. Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STB_COL_3",    8, AC_InputManager_Heli, _heli_stab_col_high, AC_ATTITUDE_HELI_STAB_COLLECTIVE_HIGH_DEFAULT),

    // @Param: STB_COL_4
    // @DisplayName: Stabilize Collective High
    // @Description: Helicopter's maximum collective pitch setting at full collective stick input in Stabilize mode. Set this as a percent of collective range given by H_COL_MAX minus H_COL_MIN.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STB_COL_4",    9, AC_InputManager_Heli, _heli_stab_col_max, AC_ATTITUDE_HELI_STAB_COLLECTIVE_MAX_DEFAULT),

    AP_GROUPEND
};

// get_pilot_desired_collective - rescale's pilot collective pitch input in Stabilize and Acro modes
float AC_InputManager_Heli::get_pilot_desired_collective(int16_t control_in)
{
    float slope_low, slope_high, slope_range, slope_run, scalar;
    float stab_col_out, acro_col_out;

    // calculate stabilize collective value which scales pilot input to reduced collective range
    // code implements a 3-segment curve with knee points at 40% and 60% throttle input
    if (control_in < 400){  // control_in ranges from 0 to 1000
        slope_low = _heli_stab_col_min * 0.01f;
        slope_high = _heli_stab_col_low * 0.01f;
        slope_range = 0.4f;
        slope_run = control_in * 0.001f;
    } else if(control_in <600){  // control_in ranges from 0 to 1000
        slope_low = _heli_stab_col_low * 0.01f;
        slope_high = _heli_stab_col_high * 0.01f;
        slope_range = 0.2f;
        slope_run = (control_in - 400) * 0.001f;  // control_in ranges from 0 to 1000
    } else {
        slope_low = _heli_stab_col_high * 0.01f;
        slope_high = _heli_stab_col_max * 0.01f;
        slope_range = 0.4f;
        slope_run = (control_in - 600) * 0.001f;  // control_in ranges from 0 to 1000
    }    

    scalar = (slope_high - slope_low)/slope_range;
    stab_col_out = slope_low + slope_run * scalar;
    stab_col_out = constrain_float(stab_col_out, 0.0f, 1.0f);

    //
    // calculate expo-scaled acro collective
    // range check expo
    if (_acro_col_expo > 1.0f) {
        _acro_col_expo.set(1.0f);
    }

    if (_acro_col_expo <= 0.0f) {
        acro_col_out = control_in * 0.001f;  // control_in ranges from 0 to 1000
    } else {
        // expo variables
        float col_in, col_in3, col_out;
        col_in = (float)(control_in-500)/500.0f;  // control_in ranges from 0 to 1000
        col_in3 = col_in*col_in*col_in;
        col_out = (_acro_col_expo * col_in3) + ((1.0f-_acro_col_expo)*col_in);
        acro_col_out = 0.5f + col_out*0.5f;
    }
    acro_col_out = constrain_float(acro_col_out, 0.0f, 1.0f);

    // ramp function
    if (is_positive(_ramp)) {
        float dt = 1/(float)_loop_rate;
        // factor 2 to transition over a time span of 0.5s
        _ramp -= 2*dt;
        _ramp = constrain_float(_ramp, 0.0f, 1.0f);
    }

    //set Stabilize or Acro collective output
    float new_flightmode_col_output;
    if (_im_flags_heli.use_stab_col) {
        new_flightmode_col_output = stab_col_out;
    } else {
        new_flightmode_col_output = acro_col_out;
    }

    // scale collective output smoothly between previous and current mode output
    float collective_out;
    collective_out = new_flightmode_col_output * (1.0 - _ramp) + _ramp * _old_flightmode_col_output;
    collective_out = constrain_float(collective_out, 0.0f, 1.0f);

    return collective_out;
}

// parameter_check - check if input manager specific parameters are sensible
bool AC_InputManager_Heli::parameter_check(char* fail_msg, uint8_t fail_msg_len) const
{

    const struct StabCheck {
        const char *name;
        int16_t value;
    } stab_checks[] = {
        {"IM_STB_COL_1", _heli_stab_col_min },
        {"IM_STB_COL_2", _heli_stab_col_low },
        {"IM_STB_COL_3", _heli_stab_col_high },
        {"IM_STB_COL_4", _heli_stab_col_max },
    };

    // check values are within valid range
    for (uint8_t i=0; i<ARRAY_SIZE(stab_checks); i++) {
        const StabCheck check = stab_checks[i];
        if ((check.value < 0) || (check.value > 100)){
            hal.util->snprintf(fail_msg, fail_msg_len, "%s out of range", check.name);
            return false;
        }
    }
    // check values are in correct order
    for (uint8_t i=1; i<ARRAY_SIZE(stab_checks); i++) {
        if ((stab_checks[i-1].value >= stab_checks[i].value)){
            hal.util->snprintf(fail_msg, fail_msg_len, "%s must be < %s", stab_checks[i-1].name, stab_checks[i].name);
            return false;
        }
    }
    // all other cases parameters are OK
    return true;
}

