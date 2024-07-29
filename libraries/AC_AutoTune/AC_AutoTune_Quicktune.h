#pragma once

#include "AutoTune_config.h"

#if AP_QUICKTUNE_ENABLED

#include "AutoTune.h"

class AP_Quicktune : public AutoTune_Backend {
public:
    // constructor
    AP_Quicktune(
        AutoTune& _frontend,
        AC_AttitudeControl& _attitude_control,
        AC_PosControl& _pos_control,
        AP_AHRS_View& _ahrs_view,
        AP_InertialNav& _inertial_nav,
        AP_Motors& _motors
    );

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void run() override;
    void stop() override;
    bool run_previous_mode() override { return true; };

    void update_switch_pos(const RC_Channel::AuxSwitchPos ch_flag) override;

private:

    // Parameters
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

    // Low, Mid and High must be in the same positions as they are in RC_Channel::AuxSwitchPos
    enum class SwitchPos : uint8_t {
        LOW,
        MID,
        HIGH,
        NONE,
    };


    enum class AxisName : uint8_t {
        RLL,
        PIT,
        YAW,
        DONE,
        END,
    };

    enum class Param : uint8_t {
        RLL_P,
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
    uint32_t last_stage_change;
    uint32_t last_gain_report;
    uint32_t last_pilot_input;
    uint32_t last_warning;
    uint32_t tune_done_time;

    // Bitmasks
    uint32_t axes_done;
    uint32_t filters_done;
    uint32_t param_changed; //Bitmask of changed parameters

    Stage current_stage = Stage::D;
    Param slew_parm = Param::END;
    float slew_target;
    uint8_t slew_steps;
    float slew_delta;
    SwitchPos sw_pos; //Switch pos to be set by aux func
    bool need_restore;
    float param_saved[uint8_t(Param::END)]; //Saved values of the parameters

    void reset_axes_done();
    void setup_filters(AxisName axis);
    bool have_pilot_input();
    AxisName get_current_axis();
    float get_slew_rate(AxisName axis);
    void advance_stage(AxisName axis);
    void adjust_gain(Param param, float value);
    void adjust_gain_limited(Param param, float value);
    float get_gain_mul();
    void restore_all_params();
    void save_all_params();
    Param get_pname(AxisName axis, Stage stage);
    AP_Float *get_param_pointer(Param param);
    float get_param_value(Param param);
    void set_param_value(Param param, float value);
    void set_and_save_param_value(Param param, float value);
    float gain_limit(Param param);
    AxisName get_axis(Param param);
    float limit_gain(Param param, float value);
    const char* get_param_name(Param param);
    Stage get_stage(Param param);
    const char* get_axis_name(AxisName axis);
    void Write_QUIK(float SRate, float Gain, Param param);

    void abort_tune(void);
};

#endif  // AP_QUICKTUNE_ENABLED
