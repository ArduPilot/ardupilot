#pragma once //这是一个编译器指令，确保头文件在编译过程中只被包含一次，避免重复包含带来的编译错误。这是 #ifndef/#define 方式的替代写法，更加简洁且更符合现代C++的头文件保护习惯。

/// @file	AC_PDNN_3D.h
/// @brief	PD + Neural Networks algorithm.
#include <AP_Common/AP_Common.h>  //AP_Common 主要用于提供一些基础的定义和工具函数，例如处理内存、字符串以及与硬件相关的低级操作。
#include <AP_Param/AP_Param.h>
#include <stdlib.h>               //C 标准库中的头文件，提供了一些通用的工具函数，例如内存分配、随机数生成、进程控制等。
#include <cmath>
#include <AC_PID/AP_PDNNInfo.h>    //自创建的PDNN相关的信息类，可能用于提供调试和监控的信息，帮助在飞行控制过程中实时查看PID输出和各项指标。
#include <Filter/SlewCalculator2D.h> //是一种用于限制输出变化率的计算器，避免在控制器中出现输出过于激烈的变化。这在飞控中非常重要，以保证输出控制的平滑性，防止电机或舵机因为突变的输入而受损。
/// @class	AC_PDNN_3D
/// @brief	Copter PDNN control class

class AC_PDNN_3D {
public:

// Constructor for PDNN //构造函数
    AC_PDNN_3D(float initial_kP, float initial_kD, float initial_kP_z, float initial_kD_z, float initial_kFF, float initial_filt_hz, float initial_filt_d_hz);

    CLASS_NO_COPY(AC_PDNN_3D); //CLASS_NO_COPY 是一个宏，常用于禁止类的复制行为

    // update_all - set target and measured inputs to PDNN controller and calculate outputs
    // target and error are filtered
    // the derivative is then calculated and filtered
    //计算和更新 PDNN 控制器的输出
    Vector3f update_all(const Vector3f &target, const Vector3f &measurement, float dt);

    // get results from pdnn controller //获取 PDNN 控制器各部分结果的成员函数。
    Vector3f get_p() const; //获取比例项输出
    Vector3f get_d() const; //获取微分项输出
    Vector3f get_ff(); //获取前馈项输出
    const Vector3f& get_error() const { return _error; } //获取当前误差值

    // reset_filter - input and D term filter will be reset to the next value provided to set_input()
    void reset_filter() { _reset_filter = true; } //重置滤波器

    // save gain to eeprom 保存增益到eeprom
    void save_gains();

    // get accessors
    AP_Float &kP() { return _kp; }
    AP_Float &kD() { return _kd; }
    AP_Float &kP_z() { return _kp_z; }
    AP_Float &kD_z() { return _kd_z; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_E_hz() { return _filt_E_hz; } //输入误差的滤波器截止频率 
    AP_Float &filt_D_hz() { return _filt_D_hz; } //微分项的滤波器截止频率 
    float get_filt_E_alpha(float dt) const;      //用于输入误差滤波的滤波系数
    float get_filt_D_alpha(float dt) const;      //用于微分项滤波的滤波系数

    // set accessors
    void set_kP(float v) { _kp.set(v); }
    void set_kD(float v) { _kd.set(v); }
    void set_kP_z(float v) { _kp_z.set(v); }
    void set_kD_z(float v) { _kd_z.set(v); }
    void set_ff(float v) { _kff.set(v); }
    void set_filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }  //设置误差滤波截止频率
    void set_filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }  //设置微分项滤波截止频率

    // return current slew rate of slew limiter. Will return 0 if SMAX is zero 当前变化率限制器的变化率 
    float get_slew_rate(void) const { return _slew_calc.get_slew_rate(); } //后续考虑删除

    const AP_PDNNInfo& get_pdnn_info_x(void) const { return _pdnn_info_x; }
    const AP_PDNNInfo& get_pdnn_info_y(void) const { return _pdnn_info_y; }
    const AP_PDNNInfo& get_pdnn_info_z(void) const { return _pdnn_info_z; }

     // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

    protected:

    // parameters
    AP_Float _kp;
    AP_Float _kd;
    AP_Float _kp_z;
    AP_Float _kd_z;
    AP_Float _kff;
    AP_Float _filt_E_hz;         // PDNN error filter frequency in Hz
    AP_Float _filt_D_hz;         // PDNN derivative filter frequency in Hz

    // internal variables
    Vector3f    _target;        // target value to enable filtering
    Vector3f    _error;         // error value to enable filtering
    Vector3f    _derivative;    // last derivative from low-pass filter
    Vector3f    _integrator;    // integrator value
    Vector3f    _pdnn_output;    //pdnn控制器输出
    Vector3f    _pdnn_output_P;  //pdnn控制器输出P
    Vector3f    _pdnn_output_D;   //pdnn控制器输出P
    bool        _reset_filter;  // true when input filter should be reset during next call to update_all

    AP_PDNNInfo _pdnn_info_x;
    AP_PDNNInfo _pdnn_info_y;
    AP_PDNNInfo _pdnn_info_z;

    SlewCalculator2D _slew_calc;    //暂时不使用 2D slew rate calculator

    private:
    const float default_kp;
    const float default_kd;
    const float default_kp_z;
    const float default_kd_z;
    const float default_kff;
    const float default_filt_E_hz;
    const float default_filt_D_hz;

};