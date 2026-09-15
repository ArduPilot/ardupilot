#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>



/// @class AC_ADRC

class AC_ADRC{
public:
    struct Defaults {
    	uint8_t adrc_type;
        float b0;
        //TD
        float td_r;
        float td_h0;
        //ESO
        float eso_beta1;
        float eso_beta2;
        float eso_beta3;
        float eso_delta;
        float eso_h_gain;
        //NLSEF
        float nlsef_alpha1;
        float nlsef_alpha2;
        float nlsef_delta;
        float nlsef_kp;
        float nlsef_kd;

        //output limit  +-1rad/S
        float limit_u_max;
        //filter
        float fil_hz;
        float wc;
        float wo;
    };

    AC_ADRC(const AC_ADRC::Defaults &defaults);
    CLASS_NO_COPY(AC_ADRC);

    void set_b0(float v)   { _b0.set(v); }
    void set_r(float v)    { _td_r.set(v); }
    void set_beta1(float v){ _eso_beta1.set(v); }
    void set_beta2(float v){ _eso_beta2.set(v); }
    void set_beta3(float v){ _eso_beta3.set(v); }
    void set_delta_eso(float v){ _eso_delta.set(v); }

    void set_alpha1(float v){ _nlsef_alpha1.set(v); }
    void set_alpha2(float v){ _nlsef_alpha2.set(v); }
    void set_delta_nlsef(float v) { _nlsef_delta.set(v); }
    void set_kp_nlsef(float v) { _nlsef_kp.set(v); }
    void set_kd_nlsef(float v) { _nlsef_kd.set(v); }
    void set_limit_umax(float v)  { _limit_u_max.set(v); }

    float get_b0()        const { return _b0.get(); }
    float get_nlsef_kp()        const { return _nlsef_kp; }
    float get_nlsef_kd()        const { return _nlsef_kd; }
    float get_eso_beta1()    const { return _eso_beta1; }
    float get_eso_beta2()    const { return _eso_beta2; }
    float get_eso_beta3()    const { return _eso_beta3; }
    float get_nlsef_alpha1()    const { return _nlsef_alpha1; }
    float get_nlsef_alpha2()    const { return _nlsef_alpha2; }

    float get_z1() const { return _z1; }
    float get_z2() const { return _z2; }
    float get_z3() const { return _z3; }

    float get_v1() const { return _v1; }
    float get_v2() const { return _v2; }

    float get_u() const { return _last_u_out; }
    float get_error() const { return _error; }
    float update_all(const float& target, const float& measure, float dt,bool limit,uint8_t aix);
    void reset_filter(float target,float measure);
    bool is_valid_data(float v);
    static const struct AP_Param::GroupInfo var_info[];

protected:

    static float fal(float e, float alpha, float delta);
    static float fhan(float x1, float x2, float r, float h);
    float get_filt_alpha(float dt) ;
    float update_axis(float target, float measure, float dt,
                      float& v1, float& v2,
                      float& z1, float& z2, float& z3,
                      float& last_u,bool limit,uint8_t aix);



    // ---- param -----------------------------------------------
    AP_Int8 _adrc_type;
    AP_Float _b0;
    AP_Float _td_r;
    AP_Float _td_h0;
    AP_Float _eso_beta1;
    AP_Float _eso_beta2;
    AP_Float _eso_beta3;
    AP_Float _eso_delta;
    AP_Float _nlsef_alpha1;
    AP_Float _nlsef_alpha2;
    AP_Float _nlsef_delta;
    AP_Float _nlsef_kp;          // NLSEF kp
    AP_Float _nlsef_kd;          // NLSEF kd
    AP_Float  _eso_h_gain;
    AP_Float _limit_u_max;
    AP_Float _filt_hz;
    AP_Float _wc;
    AP_Float _wo;
    // ---- --------------------------------------------------


    const int8_t default_adrc_type;
    const float default_b0;
    //TD
    const float default_tdr;
    const float default_td_h0;
    //ESO
    const float default_eso_beta1;
    const float default_eso_beta2;
    const float default_eso_beta3;
    const float default_eso_delta;
    const float default_eso_h_gain;
    //NLSEF
    const float default_nlsef_alpha1;
    const float default_nlsef_alpha2;
    const float default_nlsef_delta;
    const float default_nlsef_kp;
    const float default_nlsef_kd;

    const float default_limit_u_max;
    const float default_filt_hz;

    const float default_wc;
    const float default_wo;
    // ---- TD  ----
    float _v1;
    float _v2;

    // ---- ESO  ----
    float _z1;
    float _z2;
    float _z3;

    float _last_u_out;
    //error
    float  _error;
    //target and measure
    float _last_target;
    float _target;
    float _measure;
    float _last_measure;
};


