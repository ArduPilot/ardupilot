#pragma once

/// @file   AC_P_2D.h
/// @brief  2-axis P controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h> //包含一些在 ArduPilot 项目中常用的常见定义和工具函数。例如，与其他代码库的接口、通用数据类型、常量等。
#include <AP_Param/AP_Param.h>   //AP_Param 提供对参数存储和管理的支持，使控制器的常量可以持久化存储在 EEPROM 中。通过 AP_Param，可以轻松将变量存储并使其在重启飞控后得到保留。

/// @class  AC_P_2D
/// @brief  2-axis P controller
class AC_P_2D {
public:

    // constructor
    AC_P_2D(float initial_p); //参数 float initial_p 用于初始化比例控制器的初始值 p

    CLASS_NO_COPY(AC_P_2D); //CLASS_NO_COPY 是 ArduPilot 项目中的一个宏，用于禁用类的复制构造和复制赋值。这样做的目的是防止无意的拷贝行为，保持类对象的唯一性。

    // set target and measured inputs to P controller and calculate outputs
    Vector2f update_all(postype_t &target_x, postype_t &target_y, const Vector2f &measurement) WARN_IF_UNUSED; //设置比例控制器的目标和测量输入值，并计算输出。

    // set target and measured inputs to P controller and calculate outputs
    // measurement is provided as 3-axis vector but only x and y are used //接受的是三维的测量值 (Vector3f)，但实际上只提取了 x 和 y 分量，并将它们封装为一个 Vector2f，然后调用了第一个版本的 update_all()。
    Vector2f update_all(postype_t &target_x, postype_t &target_y, const Vector3f &measurement) WARN_IF_UNUSED {
        return update_all(target_x, target_y, Vector2f{measurement.x, measurement.y});
    }

    // set_limits - sets the maximum error to limit output and first and second derivative of output
    void set_limits(float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f); //设置控制器输出限制的函数。这个函数可以设置控制器的输出上限，以及输出的一阶导数（通常是速率）和二阶导数（通常是加速度）的最大值。

    // set_error_max - reduce maximum position error to error_max
    // to be called after setting limits
    void set_error_max(float error_max); //将位置误差的最大值限制为 error_max

    // get_error_max - return maximum position error
    float get_error_max() { return _error_max; } //getter 函数，用于返回控制器中存储的最大位置误差

    // save gain to eeprom
    void save_gains() { _kp.save(); } //将控制器的增益值保存到 EEPROM 的函数。EEPROM（Electrically Erasable Programmable Read-Only Memory）是一种可擦写的存储器，用于在设备掉电后仍能保持数据。

    // get accessors 定义了几个访问器（accessor）函数，用于获取类中的私有成员变量。
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    const Vector2f& get_error() const { return _error; } //获取控制器的误差值

    // set accessors 定义“写入”访问器接口
    void set_kP(float v) { _kp.set(v); } //setter 函数，用于设置控制器的比例增益（kP）。这个函数为私有成员变量 _kp 提供了一个公共的写入接口，允许外部代码安全地修改比例增益的值。

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private: //私有成员，外部无法修改，除非用访问器

    // parameters
    AP_Float    _kp;

    // internal variables
    Vector2f _error;    // error between target and measured
    float _error_max;   // error limit in positive direction
    float _D1_max;      // maximum first derivative of output

    const float default_kp;
};
