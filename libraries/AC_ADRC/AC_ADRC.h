#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>



/// @class AC_ADRC
/// @brief 角速度环二阶 ADRC 控制器
class AC_ADRC{
public:
    struct Defaults {
    	//adrc类型
    	uint8_t adrc_type;
    	//缩放输出系数
        float b0;
        //TD
        float td_r;        // TD 速度因子
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

        //输出范围+-1rad/S
        float limit_u_max;
        //数据滤波处理
        float fil_hz;

        //数据滤波处理
        float wc;
        float wo;
    };
    /// 构造函数
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

    /// 获取当前参数
    float get_b0()        const { return _b0.get(); }
    float get_nlsef_kp()        const { return _nlsef_kp; }
    float get_nlsef_kd()        const { return _nlsef_kd; }
    float get_eso_beta1()    const { return _eso_beta1; }
    float get_eso_beta2()    const { return _eso_beta2; }
    float get_eso_beta3()    const { return _eso_beta3; }
    float get_nlsef_alpha1()    const { return _nlsef_alpha1; }
    float get_nlsef_alpha2()    const { return _nlsef_alpha2; }


    /// 获取 X 轴（北）观测器状态
    float get_z1() const { return _z1; }
    float get_z2() const { return _z2; }
    float get_z3() const { return _z3; }


    /// 获取 TD 状态
    float get_v1() const { return _v1; }  // 目标位置跟踪
    float get_v2() const { return _v2; }  // 目标速度微分

    /// 获取最后一次输出
    float get_u() const { return _last_u_out; }
    ///位置误差信息
    float get_error() const { return _error; }


    ///
    /// 核心更新函数（二维同步更新）
    /// target_pos: 位置目标 (cm)，x=北, y=东
    /// actual_pos: 实际位置 (cm)
    /// dt: 采样周期 (s)
    /// 返回: 加速度指令 (cm/s²)，已做合加速度限幅
    ///
    float update_all(const float& target, const float& measure, float dt,bool limit,uint8_t aix);

    /// 重置两个轴的控制器状态
    void reset_filter(float target,float measure);
    //数据是有效的
    bool is_valid_data(float v);
    /// 参数表
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// fal 非线性函数
    static float fal(float e, float alpha, float delta);

    /// fhan 最速控制综合函数（用于 TD）
    static float fhan(float x1, float x2, float r, float h);


    float get_filt_alpha(float dt) ;
    /// 单轴更新（内部调用，x 或 y）
    /// 返回该轴的加速度（未限幅）
    float update_axis(float target, float measure, float dt,
                      float& v1, float& v2,
                      float& z1, float& z2, float& z3,
                      float& last_u,bool limit,uint8_t aix);



    // ---- 参数 -----------------------------------------------
    AP_Int8 _adrc_type;   // 控制模式类型
    AP_Float _b0;                 //系数b0

    AP_Float _td_r;        // TD 速度因子
    AP_Float _td_h0;       // TD 滤波因子

    AP_Float _eso_beta1;
    AP_Float _eso_beta2;
    AP_Float _eso_beta3;
    AP_Float _eso_delta;             // fal 线性段阈值


    AP_Float _nlsef_alpha1;      // NLSEF 位置误差非线性指数
    AP_Float _nlsef_alpha2;      // NLSEF 速度误差非线性指数
    AP_Float _nlsef_delta;       // NLSEF fal 线性段阈值
    AP_Float _nlsef_kp;          // NLSEF kp
    AP_Float _nlsef_kd;          // NLSEF kd
    AP_Float  _eso_h_gain;
    AP_Float _limit_u_max;   // 输出加速度上限 (cm/s²)
    AP_Float _filt_hz;       // 数据滤波处理
    AP_Float _wc;            // 控制带宽
    AP_Float _wo;            // 观测带宽
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
    //数据滤波处理
    const float default_filt_hz;

    const float default_wc;
    const float default_wo;
    // ---- TD 状态 ----
    float _v1;             // 跟踪位置
    float _v2;             // 跟踪速度（微分）

    // ---- ESO 状态 ----
    float _z1;             // 位置估计
    float _z2;             // 速度估计
    float _z3;             // 总扰动估计

    // ---- 输出 ----
    float _last_u_out;          // 最后一次加速度输出
    //误差
    float  _error;    //误差信息

    float _last_target;
    float _target;
    float _measure;
    float _last_measure;
};


