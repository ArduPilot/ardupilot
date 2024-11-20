#pragma once //这是一个编译器指令，确保头文件在编译过程中只被包含一次，避免重复包含带来的编译错误。这是 #ifndef/#define 方式的替代写法，更加简洁且更符合现代C++的头文件保护习惯。

/// @file	AC_PDNN_SO3.h
/// @brief	PD + Neural Networks algorithm.
#include <AP_Common/AP_Common.h>  //AP_Common 主要用于提供一些基础的定义和工具函数，例如处理内存、字符串以及与硬件相关的低级操作。
#include <AP_Param/AP_Param.h>
#include <stdlib.h>               //C 标准库中的头文件，提供了一些通用的工具函数，例如内存分配、随机数生成、进程控制等。
#include <cmath>
#include <AC_PID/AP_PDNNSO3Info.h>    //自创建的PDNN相关的信息类，可能用于提供调试和监控的信息，帮助在飞行控制过程中实时查看输出和各项指标。
#include <Filter/SlewCalculator2D.h> //是一种用于限制输出变化率的计算器，避免在控制器中出现输出过于激烈的变化。这在飞控中非常重要，以保证输出控制的平滑性，防止电机或舵机因为突变的输入而受损。
/// @class	AC_PDNN_SO3
/// @brief	Copter PDNN control class

class AC_PDNN_SO3 {
public:

// Constructor for PDNN //构造函数
    AC_PDNN_SO3(float initial_kR, float initial_kOmega, float initial_kR_z, float initial_kOmega_z);

    CLASS_NO_COPY(AC_PDNN_SO3); //CLASS_NO_COPY 是一个宏，常用于禁止类的复制行为

    //计算和更新 PDNN 控制器的输出
    Vector3f update_all(const Matrix3f &R_c, const Matrix3f &R, const Vector3f &Omega, float dt);

    // get results from pdnn controller //获取 PDNN 控制器各部分结果的成员函数。
    Vector3f get_R() const; //获取R比例项输出
    Vector3f get_Omega() const; //获取Omega微分项输出
    
    const Vector3f& get_e_R() const { return _e_R; } //获取当前误差值

    void reset() { _reset = true; } //重置检查
  
    // save gain to eeprom 保存增益到eeprom
    void save_gains();

    // get accessors
    AP_Float &kR() { return _kR; }
    AP_Float &kOmega() { return _kOmega; }
    AP_Float &kR_z() { return _kR_z; }
    AP_Float &kOmega_z() { return _kOmega_z; }
    

    // set accessors
    void set_kR(float v) { _kR.set(v); }
    void set_kOmega(float v) { _kOmega.set(v); }
    void set_kR_z(float v) { _kR_z.set(v); }
    void set_kOmega_z(float v) { _kOmega_z.set(v); }

    const AP_PDNNSO3Info& get_pdnn_info_x(void) const { return _pdnn_info_x; }
    const AP_PDNNSO3Info& get_pdnn_info_y(void) const { return _pdnn_info_y; }
    const AP_PDNNSO3Info& get_pdnn_info_z(void) const { return _pdnn_info_z; }

     // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

    protected:

    // parameters
    AP_Float _kR;
    AP_Float _kOmega;
    AP_Float _kR_z;
    AP_Float _kOmega_z;

    // internal variables
    Matrix3f    _R_c;        // 期望旋转矩阵
    Matrix3f    _R;          // 测量的当前旋转矩阵
    Vector3f    _Omega;      //测量的当前角速度
    Matrix3f    _Omega_hat;      //测量的当前角速度的斜对称矩阵
    Vector3f    _Omega_c;      //期望角速度
    Matrix3f    _Omega_c_hat;      //期望角速度的斜对称矩阵
    Vector3f    _e_R;         // 旋转矩阵误差
    Matrix3f    _e_R_hat;     // 旋转矩阵误差的斜对称矩阵           
    Vector3f    _e_Omega;         //角加速度误差
    Matrix3f    _dot_R_c;     // 期望旋转矩阵的导数
    Vector3f    _dot_Omega_c;     // 期望角速度的导数
    Vector3f    _derivative;    // last derivative from low-pass filter
    Vector3f    _integrator;    // integrator value
    Vector3f    _pdnn_output;    //pdnn控制器输出
    Vector3f    _pdnn_output_R;  //pdnn控制器输出P
    Vector3f    _pdnn_output_Omega;   //pdnn控制器输出P
    Vector3f    _geomrtry_output;  //几何项输出
    bool        _reset; 

    AP_PDNNSO3Info _pdnn_info_x;
    AP_PDNNSO3Info _pdnn_info_y;
    AP_PDNNSO3Info _pdnn_info_z;


    private:
    const float default_kR;
    const float default_kOmega;
    const float default_kR_z;
    const float default_kOmega_z;

};