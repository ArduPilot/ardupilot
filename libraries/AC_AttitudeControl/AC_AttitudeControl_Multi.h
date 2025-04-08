#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PDNN Controller~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <AC_PID/AC_PDNN_SO3.h>  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RPY_FILT_HZ
 # define AC_ATC_MULTI_RATE_RPY_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~添加pdnn~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define ATTCONTROL_PDNN_XY_kR                  70.0f    // horizontal pdnn controller P gain default
#define ATTCONTROL_PDNN_XY_KOmega              70.0f    // horizontal pdnn controller D gain default
#define ATTCONTROL_PDNN_Z_kR                   50.0f    // veritical pdnn controller P gain default
#define ATTCONTROL_PDNN_Z_KOmega               50.0f    // veritical pdnn controller D gain default
//在头文件中定义pdnn控制器构造函数的初始化默认值，是因为cpp文件中if编译需要工作空间先build copter
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle//设置油门输出，传入值是从AC_PosControl.cpp文件垂直控制中thr_out传入
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override; //override是一个标记符，用于表示继承

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Rc旋转矩阵传入~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void set_Rc(const Matrix3f& Rc, bool Rc_active) override;  //接受从位置控制Ac_PosControl中传入的Rc，并存储到_Rc内部变量
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~getter纯虚函数~~~~~~~~~~~~~~~~~~~~用于AC_PosControl中获取姿态控制数据
    Vector3f get_phi() const override; //获取神经网络输出
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run_dt(const Vector3f& gyro, float dt) override;
    void rate_controller_target_reset() override;
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // boost angle_p/pd each cycle on high throttle slew
    void update_throttle_gain_boost();

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll { //聚合初始化，c++11的语法。可以直接在类声明时对成员变量进行初始化的方式。
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_PID                _pid_rate_pitch{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_RP_P,
            .i         = AC_ATC_MULTI_RATE_RP_I,
            .d         = AC_ATC_MULTI_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AC_PID                _pid_rate_yaw{
        AC_PID::Defaults{
            .p         = AC_ATC_MULTI_RATE_YAW_P,
            .i         = AC_ATC_MULTI_RATE_YAW_I,
            .d         = AC_ATC_MULTI_RATE_YAW_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_MULTI_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .filt_E_hz = AC_ATC_MULTI_RATE_YAW_FILT_HZ,
            .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };


 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~创建PDNN控制器对象~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    AC_PDNN_SO3      _pdnn_att;  //没有用到聚合初始化，所以需要在cpp的构造函数中初始化

 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    // angle_p/pd boost multiplier
    AP_Float              _throttle_gain_boost;

     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~声明内部成员变量~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    Matrix3f    _Rc;      //解算后的期望旋转矩阵
    bool        _Rc_active;
    Matrix3f    _R_body_to_ned_meas; //旋转矩阵测量值 body to NED
    Matrix3f    _R_body_to_neu_meas; //旋转矩阵测量值 body to NEU

    //声明神经网络输出
    float _phi_x,_phi_y,_phi_z;

    //声明自适应参数
    float _m_x, _m_y, _m_z;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
};

 ///~~~~~~~~~~~~~~~~~~~~~~~~~~~DIY New DDS Topic output get_log~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Wrench struct

struct DIYLog
{
Vector3f force; // 力的三维向量
Vector3f torque; // 力矩的三维向量
// 默认构造函数，初始化为 0
DIYLog() : force(), torque() {}
// 参数化构造函数
DIYLog(const Vector3f& f, const Vector3f& t) : force(f), torque(t) {}
};

extern DIYLog get_log_out_1(float test_msg_3, float test_msg_4); //注意这里全局变量，所以传入的参数名也要区分
extern DIYLog current_log_out_1;
extern DIYLog get_current_log_out_1();
///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~DIY end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
