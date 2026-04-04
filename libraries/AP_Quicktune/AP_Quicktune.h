#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED 1 // NOTE: may be disabled by vehicle header
#endif

#if AP_QUICKTUNE_ENABLED

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

class AP_Quicktune {
public:
    AP_Quicktune()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_Quicktune() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Quicktune);

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void update(bool mode_supports_quicktune);
    void update_switch_pos(const RC_Channel::AuxSwitchPos ch_flag);

private:

    // Parameters
    AP_Int8 enable;
    AP_Int8 axes_enabled;
    AP_Float double_time;
    AP_Float gain_margin;
    AP_Float osc_smax;
    AP_Float yaw_p_max;
    AP_Float yaw_d_max;
    AP_Float rp_pi_ratio;
    AP_Float y_pi_ratio;
    AP_Int8 auto_filter;
    AP_Float auto_save;
    AP_Float reduce_max;
    AP_Int16 options;
    AP_Int8 angle_max;

    // Low, Mid and High must be in the same positions as they are in RC_Channel::AuxSwitchPos
    enum class SwitchPos : uint8_t {
        LOW,
        MID,
        HIGH,
        NONE,
    };


    enum class AxisName : uint8_t {
        RLL = 0,
        PIT,
        YAW,
        DONE,
        END,
    };

    /*
      note! we rely on the enum being in the same order between axes
     */
    enum class Param : uint8_t {
        RLL_P = 0,
        RLL_I,
        RLL_D,
        RLL_SMAX,
        RLL_FLTT,
        RLL_FLTD,
        RLL_FLTE,
        RLL_FF,

        PIT_P,
        PIT_I,
        PIT_D,
        PIT_SMAX,
        PIT_FLTT,
        PIT_FLTD,
        PIT_FLTE,
        PIT_FF,

        YAW_P,
        YAW_I,
        YAW_D,
        YAW_SMAX,
        YAW_FLTT,
        YAW_FLTD,
        YAW_FLTE,
        YAW_FF,
        END,
    };

    static const uint8_t param_per_axis = uint8_t(Param::PIT_P) - uint8_t(Param::RLL_P);
    static_assert(uint8_t(Param::END) == 3*param_per_axis, "AP_Quicktune Param error");

    // Also the gains
    enum class Stage : uint8_t {
        D,
        P,
        DONE,
        I,
        FF,
        SMAX,
        FLTT,
        FLTD,
        FLTE,
        END,
    };

    // Time keeping
    uint32_t last_stage_change_ms;
    uint32_t last_gain_report_ms;
    uint32_t last_pilot_input_ms;
    uint32_t last_warning_ms;
    uint32_t tune_done_time_ms;

    // Bitmasks
    uint32_t axes_done;
    uint32_t filters_done;
    uint32_t param_changed; //Bitmask of changed parameters

    Stage current_stage = Stage::D;
    Param slew_parm = Param::END;
    uint8_t slew_steps;
    float slew_delta;
    SwitchPos sw_pos; //Switch pos to be set by aux func
    bool need_restore;
    float param_saved[uint8_t(Param::END)]; //Saved values of the parameters

    void reset_axes_done();
    void setup_filters(AxisName axis);
    bool have_pilot_input() const;
    AxisName get_current_axis() const;
    float get_slew_rate(AxisName axis) const;
    void advance_stage(AxisName axis);
    void adjust_gain(Param param, float value);
    void adjust_gain_limited(Param param, float value);
    float get_gain_mul() const;
    void restore_all_params();
    void save_all_params();
    Param get_pname(AxisName axis, Stage stage) const;
    AP_Float *get_param_pointer(Param param) const;
    float get_param_value(Param param) const;
    void set_param_value(Param param, float value);
    void set_and_save_param_value(Param param, float value);
    float gain_limit(Param param) const;
    AxisName get_axis(Param param) const;
    float limit_gain(Param param, float value);
    const char* get_param_name(Param param) const;
    Stage get_stage(Param param) const;
    const char* get_axis_name(AxisName axis) const;
    AC_PID *get_axis_pid(AxisName axis) const;
    void Write_QWIK(float SRate, float Gain, Param param);

    void abort_tune(void);
};

#endif  // AP_QUICKTUNE_ENABLED
