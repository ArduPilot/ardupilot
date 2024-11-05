#pragma once //这是一个编译器指令，确保头文件在编译过程中只被包含一次，避免重复包含带来的编译错误。这是 #ifndef/#define 方式的替代写法，更加简洁且更符合现代C++的头文件保护习惯。

/// @file	AC_PID_2D.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>  //AP_Common 主要用于提供一些基础的定义和工具函数，例如处理内存、字符串以及与硬件相关的低级操作。
#include <AP_Param/AP_Param.h>
#include <stdlib.h>               //C 标准库中的头文件，提供了一些通用的工具函数，例如内存分配、随机数生成、进程控制等。
#include <cmath>
#include <AC_PID/AP_PIDInfo.h>    //是一个PID相关的信息类，可能用于提供调试和监控的信息，帮助在飞行控制过程中实时查看PID输出和各项指标。
#include <Filter/SlewCalculator2D.h> //是一种用于限制输出变化率的计算器，避免在控制器中出现输出过于激烈的变化。这在飞控中非常重要，以保证输出控制的平滑性，防止电机或舵机因为突变的输入而受损。

/// @class	AC_PID_2D
/// @brief	Copter PID control class
class AC_PID_2D {
public:

    // Constructor for PID //构造函数
    AC_PID_2D(float initial_kP, float initial_kI, float initial_kD, float initial_kFF, float initial_imax, float initial_filt_hz, float initial_filt_d_hz);

    CLASS_NO_COPY(AC_PID_2D); //CLASS_NO_COPY 是一个宏，常用于禁止类的复制行为

    // update_all - set target and measured inputs to PID controller and calculate outputs
    // target and error are filtered
    // the derivative is then calculated and filtered
    // the integral is then updated if it does not increase in the direction of the limit vector
    //计算和更新 PID 控制器的输出
    Vector2f update_all(const Vector2f &target, const Vector2f &measurement, float dt, const Vector2f &limit);
    Vector2f update_all(const Vector3f &target, const Vector3f &measurement, float dt, const Vector3f &limit);

    // update the integral
    // if the limit flag is set the integral is only allowed to shrink
    void update_i(float dt, const Vector2f &limit); //更新 PID 控制器的积分项

    // get results from pid controller //获取 PID 控制器各部分结果的成员函数。
    Vector2f get_p() const; //获取比例项输出
    const Vector2f& get_i() const; //获取积分项输出
    Vector2f get_d() const; //获取微分项输出
    Vector2f get_ff(); //获取前馈项输出
    const Vector2f& get_error() const { return _error; } //获取当前误差值

    // reset the integrator
    void reset_I(); //重置积分项

    // reset_filter - input and D term filter will be reset to the next value provided to set_input()
    void reset_filter() { _reset_filter = true; } //重置滤波器

    // save gain to eeprom 保存增益到eeprom
    void save_gains();

    // get accessors
    AP_Float &kP() { return _kp; }
    AP_Float &kI() { return _ki; }
    AP_Float &kD() { return _kd; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_E_hz() { return _filt_E_hz; } //输入误差的滤波器截止频率 
    AP_Float &filt_D_hz() { return _filt_D_hz; } //微分项的滤波器截止频率 
    float imax() const { return _kimax.get(); }  //积分项的最大值限制 Imax
    float get_filt_E_alpha(float dt) const;      //用于输入误差滤波的滤波系数
    float get_filt_D_alpha(float dt) const;      //用于微分项滤波的滤波系数

    // set accessors
    void set_kP(float v) { _kp.set(v); }
    void set_kI(float v) { _ki.set(v); }
    void set_kD(float v) { _kd.set(v); }
    void set_ff(float v) { _kff.set(v); }
    void set_imax(float v) { _kimax.set(fabsf(v)); }  //设置积分项最大值
    void set_filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }  //设置误差滤波截止频率
    void set_filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }  //设置微分项滤波截止频率

    // integrator setting functions //设置积分器的初始状态
    void set_integrator(const Vector2f& target, const Vector2f& measurement, const Vector2f& i);
    void set_integrator(const Vector2f& error, const Vector2f& i);
    void set_integrator(const Vector3f& i) { set_integrator(Vector2f{i.x, i.y}); }
    void set_integrator(const Vector2f& i);

    // return current slew rate of slew limiter. Will return 0 if SMAX is zero 当前变化率限制器的变化率
    float get_slew_rate(void) const { return _slew_calc.get_slew_rate(); }

    const AP_PIDInfo& get_pid_info_x(void) const { return _pid_info_x; }
    const AP_PIDInfo& get_pid_info_y(void) const { return _pid_info_y; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _kff;
    AP_Float _kimax;
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz

    // internal variables
    Vector2f    _target;        // target value to enable filtering
    Vector2f    _error;         // error value to enable filtering
    Vector2f    _derivative;    // last derivative from low-pass filter
    Vector2f    _integrator;    // integrator value
    bool        _reset_filter;  // true when input filter should be reset during next call to update_all

    AP_PIDInfo _pid_info_x;
    AP_PIDInfo _pid_info_y;

    SlewCalculator2D _slew_calc;    // 2D slew rate calculator

private:
    const float default_kp;
    const float default_ki;
    const float default_kd;
    const float default_kff;
    const float default_kimax;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
};
