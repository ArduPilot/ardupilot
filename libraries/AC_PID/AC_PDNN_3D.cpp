/// @file	AC_PDNN_3D.cpp
/// @brief	Generic PDNN algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PDNN_3D.h"

#define AC_PDNN_3D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency 作为提醒定义的宏，在别处好像没有被调用


const AP_Param::GroupInfo AC_PDNN_3D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PDNN_3D, _kp, default_kp),

    // @Param: FLTE
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 1, AC_PDNN_3D, _filt_E_hz, default_filt_E_hz),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D",    2, AC_PDNN_3D, _kd, default_kd),

    // @Param: FLTD
    // @DisplayName: D term filter frequency in Hz
    // @Description: D term filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 3, AC_PDNN_3D, _filt_D_hz, default_filt_D_hz),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF",    4, AC_PDNN_3D, _kff, default_kff),

    AP_GROUPEND
};

// Constructor 构造函数
AC_PDNN_3D::AC_PDNN_3D(float initial_kP, float initial_kD, float initial_kFF, float initial_filt_E_hz, float initial_filt_D_hz) :
    default_kp(initial_kP),
    default_kd(initial_kD),
    default_kff(initial_kFF),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info); //读取eeprom存储参数值，也可以不用eeprom，选择在代码中直接定义硬编码参数值，坏处是调试后每次都会重置，不会保存。

    // reset input filter to first value received 重置滤波器
    _reset_filter = true;
}

//  update_all - set target and measured inputs to PDNN controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated if it does not increase in the direction of the limit vector
Vector3f AC_PDNN_3D::update_all(const Vector3f &target, const Vector3f &measurement, float dt)
{
    // don't process inf or NaN //检查输入的有效性，避免处理空值NaN与无穷大inf的值
    if (target.is_nan() || target.is_inf() ||
        measurement.is_nan() || measurement.is_inf()) {
        return Vector3f{};
    }

    _target = target; //_target 是一个私有成员变量，作为内部使用的值

    // reset input filter to value received //无人机重启pid时的初始化
    if (_reset_filter) {
        _reset_filter = false;
        _error = _target - measurement;
        _derivative.zero();  //.zero是vector3f自带的成员函数，可以便捷地初始化归零一个三维向量
    } else {
        Vector3f error_last{_error}; //将上一个循环计算出的误差 _error 存储到一个临时变量 error_last 中，用于后续的微分项计算。
        _error += ((_target - measurement) - _error) * get_filt_E_alpha(dt); //低通滤波：误差变化量*滤波系数

        // calculate and filter derivative
        if (is_positive(dt)) { //检查时间步长是否有效
            const Vector3f derivative{(_error - error_last) / dt}; //_error - error_last：计算当前误差与上一时刻误差之间的差，表示误差的变化量。计算误差变化量除以时间步长 dt，得到误差变化的速率，即微分项。
            _derivative += (derivative - _derivative) * get_filt_D_alpha(dt); //低通滤波：微分项变化量*滤波系数。使用低通滤波来对微分项进行平滑处理
        }
    }

    // calculate slew limit //后续考虑删除
    _slew_calc.update(Vector2f{_pdnn_info_x.P + _pdnn_info_x.D, _pdnn_info_y.P + _pdnn_info_y.D}, dt);
    _pdnn_info_x.slew_rate = _pdnn_info_y.slew_rate = _slew_calc.get_slew_rate();
 
    _pdnn_info_x.target = _target.x;
    _pdnn_info_x.actual = measurement.x;
    _pdnn_info_x.error = _error.x;
    _pdnn_info_x.P = _error.x * _kp;
    _pdnn_info_x.D = _derivative.x * _kd;
    _pdnn_info_x.FF = _target.x * _kff;//这里应该改成期望加速度前馈ddotXd

    _pdnn_info_y.target = _target.y;
    _pdnn_info_y.actual = measurement.y;
    _pdnn_info_y.error = _error.y;
    _pdnn_info_y.P = _error.y * _kp;
    _pdnn_info_y.D = _derivative.y * _kd;
    _pdnn_info_y.FF = _target.y * _kff;//这里应该改成期望加速度前馈ddotYd

    _pdnn_info_z.target = _target.z;
    _pdnn_info_z.actual = measurement.z;
    _pdnn_info_z.error = _error.z;
    _pdnn_info_z.P = _error.z * _kp;
    _pdnn_info_z.D = _derivative.z * _kd;
    _pdnn_info_z.FF = _target.z * _kff;//这里应该改成期望加速度前馈ddotYd

    return _error * _kp + _derivative * _kd + _target * _kff; //返回pdnn控制器输出
}

Vector3f AC_PDNN_3D::get_p() const
{
    return _error * _kp;
}

Vector3f AC_PDNN_3D::get_d() const
{
    return _derivative * _kd;
}

Vector3f AC_PDNN_3D::get_ff()
{
    _pdnn_info_x.FF = _target.x * _kff;
    _pdnn_info_y.FF = _target.y * _kff;
    _pdnn_info_z.FF = _target.z * _kff;
    return _target * _kff;
}

// save_gains - save gains to eeprom
void AC_PDNN_3D::save_gains()
{
    _kp.save();
    _kd.save();
    _kff.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

// get the target filter alpha
float AC_PDNN_3D::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get the derivative filter alpha
float AC_PDNN_3D::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}



  
