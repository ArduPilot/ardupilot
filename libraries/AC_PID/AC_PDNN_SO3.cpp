/// @file	AC_PDNN_SO3.cpp
/// @brief	Generic PDNN algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PDNN_SO3.h"


const AP_Param::GroupInfo AC_PDNN_SO3::var_info[] = {
    // @Param: kR
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("kR_xy",    0, AC_PDNN_SO3, _kR, default_kR),
    // @Param: KOmega
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("kOmega_xy",    1, AC_PDNN_SO3, _kOmega, default_kOmega),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("kR_z",    2, AC_PDNN_SO3, _kR_z, default_kR_z),
    
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("kOmega_z",    3, AC_PDNN_SO3, _kOmega_z, default_kOmega_z),

    AP_GROUPEND
};

// Constructor 构造函数
AC_PDNN_SO3::AC_PDNN_SO3(float initial_kR, float initial_kOmega, float initial_kR_z, float initial_kOmega_z) :
    default_kR(initial_kR),
    default_kOmega(initial_kOmega),
    default_kR_z(initial_kR_z),
    default_kOmega_z(initial_kOmega_z)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info); //读取eeprom存储参数值，也可以不用eeprom，选择在代码中直接定义硬编码参数值，坏处是调试后每次都会重置，不会保存。
    
    // reset input filter to first value received 重置控制器
    _reset = true; //每次调用重置为true
}

//  update_all - set target and measured inputs to PDNN controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated if it does not increase in the direction of the limit vector
Vector3f AC_PDNN_SO3::update_all(const Matrix3f &R_c, const Matrix3f &R, const Vector3f &Omega, float dt)
{
    // don't process inf or NaN //检查输入的有效性，避免处理空值NaN与无穷大inf的值
    if (R_c.is_nan() || R.is_nan()) {
        return Vector3f{}; //返回一个vector3f避免报错
    }
    _R = R;
    _Omega = Omega;
    //更新_Omega_hat斜对称矩阵
    _Omega_hat.a.x = 0.0f;_Omega_hat.a.y = -_Omega.z;_Omega_hat.a.z = _Omega.y;
    _Omega_hat.b.x = _Omega.z;_Omega_hat.b.y = 0.0f;_Omega_hat.b.z = -_Omega.x;
    _Omega_hat.c.x = -_Omega.y;_Omega_hat.c.y = _Omega.x;_Omega_hat.c.z = 0.0f;
 

    // reset input filter to value received //无人机重启pdnn姿态控制时的初始化
    if (_reset) { //初始化逻辑
        _reset = false;
        
        _R_c = R_c; //更新当前循环的_R_c
        
        //旋转矩阵误差_e_R初始化
        _e_R_hat = (_R_c.transposed() * _R - _R.transposed() * _R_c) * 0.5f; //计算旋转矩阵误差的斜对称矩阵, 注意要把0.5f放在Matrix3f后面，因为函数重载的格式要求
        _e_R.x = -_e_R_hat.b.z; //斜对称矩阵.V逆运算，对_e_R进行赋值得到旋转矩阵误差_e_R，注意是一个Vector3f
        _e_R.y = _e_R_hat.a.z;
        _e_R.z = _e_R_hat.b.x;

        //防止微分爆炸，初始化微分项
        _dot_R_c.zero();
        _dot_Omega_c.zero();
        _Omega_c.zero();

        //角速度误差_e_Omega初始化
        //！！！先尝试初始化为Omega，因为前面初始化了微分项！！这里可能需要修改
        _e_Omega = _Omega;

    } else { //更新循环
        Matrix3f _R_c_last{_R_c}; //将上一个循环的_R_c存储到一个临时变量 _R_c_last 中，用于后续的微分项计算。这里用到拷贝函数，等价于Matrix3f error_last = _error;
        
        //更新当前循环的_R_c
        _R_c = R_c; 

        //更新当前循环旋转矩阵误差_e_R
        _e_R_hat = (_R_c.transposed() * _R - _R.transposed() * _R_c) * 0.5f; //计算旋转矩阵误差的斜对称矩阵，注意要把0.5f放在Matrix3f后面，因为函数重载的格式要求
        _e_R.x = -_e_R_hat.b.z; //斜对称矩阵.V逆运算，对_e_R进行赋值得到旋转矩阵误差_e_R，注意是一个Vector3f
        _e_R.y = _e_R_hat.a.z;
        _e_R.z = _e_R_hat.b.x;

        //计算_R_c微分项，这里暂时不考虑滤波
        if (is_positive(dt)) { //检查时间步长是否有效
            _dot_R_c = (_R_c - _R_c_last) / dt;  //理论上应该可以实现逐元素求导（考虑进行正交化或者转化为四元数后归一化！！！）
        }
        
        Vector3f _Omega_c_last{_Omega_c}; //将上一个循环的_Omega_c存储到一个临时变量 _Omegac_c_last 中

        //更新当前循环的期望角速度_Omega_c
        _Omega_c_hat = _R_c.transposed() * _dot_R_c;
        _Omega_c.x = -_Omega_c_hat.b.z; //斜对称矩阵.V逆运算
        _Omega_c.y = _Omega_c_hat.a.z;
        _Omega_c.z = _Omega_c_hat.b.x;
    
        //更新当前循环角速度误差_e_Omega ！！考虑加上滤波！！！！（暂时没加）
        _e_Omega = _Omega - _R.transposed() * _R_c * _Omega_c;
        //_e_Omega = _Omega; //暂时第二项设置为0 

        //计算_Omega_c微分项，这里暂时不考虑滤波
        if (is_positive(dt)) { //检查时间步长是否有效
            _dot_Omega_c = (_Omega_c - _Omega_c_last) / dt;  //理论上应该可以实现逐元素求导
        }

    }
 

    _pdnn_output_R.x = -_e_R.x * _kR; //计算P项输出
    _pdnn_output_R.y = -_e_R.y * _kR;
    _pdnn_output_R.z = -_e_R.z * _kR_z;

    _pdnn_output_Omega.x = -_e_Omega.x * _kOmega; //计算D项输出
    _pdnn_output_Omega.y = -_e_Omega.y * _kOmega;
    _pdnn_output_Omega.z = -_e_Omega.z * _kOmega_z;
    
    //计算几何控制项，这里惯性张量
    Matrix3f J;
    J.a.x=0.001;J.a.y=0;     J.a.z=0;
    J.b.x=0;      J.b.y=0.001;J.b.z=0;
    J.c.x=0;      J.c.y=0;     J.c.z=0.005;
    _geomrtry_output = J*(_Omega_hat * _R.transposed() * _R_c * _Omega_c  - _R.transposed() * _R_c * _dot_Omega_c);

    //计算总输出，每个方向上乘以惯性张量
    _pdnn_output.x = -_e_R.x * _kR - _e_Omega.x * _kOmega - _geomrtry_output.x; 
    _pdnn_output.y = -_e_R.y * _kR - _e_Omega.y * _kOmega - _geomrtry_output.y;
    _pdnn_output.z = -_e_R.z * _kR_z - _e_Omega.z * _kOmega_z - _geomrtry_output.z;


    return _pdnn_output; //返回pdnn控制器输出
}

Vector3f AC_PDNN_SO3::get_R() const
{
    return _pdnn_output_R;
}

Vector3f AC_PDNN_SO3::get_Omega() const
{
    return _pdnn_output_Omega;
}


// save_gains - save gains to eeprom
void AC_PDNN_SO3::save_gains()
{
    _kR.save();
    _kOmega.save();
    _kR_z.save();
    _kOmega_z.save();
    //_filt_E_hz.save();
    //_filt_D_hz.save();
}



  
