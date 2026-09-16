#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

class AC_ADRC
{
public:
    struct Defaults {
        uint8_t adrc_type;
        float b0;
        float td_r;
        float td_h0;
        float eso_beta1;
        float eso_beta2;
        float eso_beta3;
        float eso_delta;
        float eso_h_gain;
        float nlsef_alpha1;
        float nlsef_alpha2;
        float nlsef_delta;
        float nlsef_kp;
        float nlsef_kd;
        float limit_u_max;
        float filt_target_hz;
        float filt_measure_hz;
        float wc;
        float wo;
    };

    AC_ADRC(const Defaults &defaults);

    float update_all(const float& target,const float& measure,float dt,bool limit,uint8_t aix);
    void reset_filter(float target,float measure);

    static const AP_Param::GroupInfo var_info[];

private:
    // state struct
    struct ADRC_State;

    // core update subfunctions
    float update_axis(float target, float measure, float dt, ADRC_State &state,bool limit,uint8_t aix);
    float _update_ladrc_1st(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix);
    float _update_ladrc_2nd(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix);
    float _update_ladrc_2nd_exp(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix);
    float _update_nl_adrc(float original_target, float original_measure,float target, float measure, float dt, ADRC_State &state, uint8_t aix);

    // logging helper
    void _write_log(uint8_t aix, const char* log_name, float t1, float t3, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15);
    void _write_log_nl(uint8_t aix, const char* log_name, float t1, float t3, float t5, float t6, float t7, float t8, float t9, float t10, float t11, float t12, float t13, float t14, float t15);

    bool _is_state_valid(const ADRC_State &state) const;
    void _reset_state(ADRC_State &state, float measure, float target);

    bool is_valid_data(float v) const;
    float fal(float e, float alpha, float delta) const;
    float sign(float x);
    float fhan(float x1, float x2, float r, float h) const;
    float get_filt_target_alpha(float dt) const;
    float get_filt_measure_alpha(float dt) const;

    // Parameters
    AP_Int8 _adrc_type;
    AP_Float _b0;
    AP_Float _td_r;
    AP_Float _td_h0;
    AP_Float _eso_beta1;
    AP_Float _eso_beta2;
    AP_Float _eso_beta3;
    AP_Float _eso_delta;
    AP_Float _eso_h_gain;
    AP_Float _nlsef_alpha1;
    AP_Float _nlsef_alpha2;
    AP_Float _nlsef_delta;
    AP_Float _nlsef_kp;
    AP_Float _nlsef_kd;
    AP_Float _limit_u_max;
    AP_Float _filt_target_hz;
    AP_Float _filt_measure_hz;
    AP_Float _wc;
    AP_Float _wo;

    // Internal states
    float _v1;
    float _v2;
    float _z1;
    float _z2;
    float _z3;
    float _last_u_out;
    float _target;
    float _last_target;
    float _measure;
    float _last_measure;

    uint32_t _update_last_log_loop_time1;
    uint32_t _update_last_log_loop_time2;
    uint32_t _update_last_log_loop_time3;

    // default value storage
    uint8_t default_adrc_type;
    float default_b0;
    float default_td_r;
    float default_td_h0;
    float default_eso_beta1;
    float default_eso_beta2;
    float default_eso_beta3;
    float default_eso_delta;
    float default_eso_h_gain;
    float default_nlsef_alpha1;
    float default_nlsef_alpha2;
    float default_nlsef_delta;
    float default_nlsef_kp;
    float default_nlsef_kd;
    float default_limit_u_max;
    float default_filt_target_hz;
    float default_filt_measure_hz;
    float default_wc;
    float default_wo;
};




