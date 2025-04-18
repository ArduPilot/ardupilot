/// @file	AC_PDNN_3D.cpp
/// @brief	Generic PDNN algorithm 注意此控制器应该转换外部误差单位cm -> m

#include <AP_Math/AP_Math.h>
#include "AC_PDNN_3D.h"

#define AC_PDNN_3D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency 作为提醒定义的宏，在别处好像没有被调用


const AP_Param::GroupInfo AC_PDNN_3D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P_xy",    0, AC_PDNN_3D, _kp, default_kp),

    // @Param: FLTE
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 1, AC_PDNN_3D, _filt_E_hz, default_filt_E_hz),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_xy",    2, AC_PDNN_3D, _kd, default_kd),

    // @Param: FLTD
    // @DisplayName: D term filter frequency in Hz
    // @Description: D term filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 3, AC_PDNN_3D, _filt_D_hz, default_filt_D_hz),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF",    4, AC_PDNN_3D, _kff, default_kff),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P_z",    5, AC_PDNN_3D, _kp_z, default_kp_z),
    
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_z",    6, AC_PDNN_3D, _kd_z, default_kd_z),

    AP_GROUPEND
};

// Constructor 构造函数
AC_PDNN_3D::AC_PDNN_3D(float initial_kP, float initial_kD, float initial_kP_z, float initial_kD_z, float initial_kFF, float initial_filt_E_hz, float initial_filt_D_hz) :
    default_kp(initial_kP),
    default_kd(initial_kD),
    default_kp_z(initial_kP_z),
    default_kd_z(initial_kD_z),
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
Vector3f AC_PDNN_3D::update_all(const Vector3f &target, const Vector3f &measurement, const Vector3f &acc_desired,float dt)
{
    // don't process inf or NaN //检查输入的有效性，避免处理空值NaN与无穷大inf的值
    if (target.is_nan() || target.is_inf() ||
        measurement.is_nan() || measurement.is_inf()) {
        return Vector3f{};
    }

    _target = target; //_target 是一个私有成员变量，作为内部使用的值
    _acc_desired = acc_desired; //_acc_desired 是一个私有成员变量，作为内部使用的值
    
    //设置默认RBF网络中心矩阵 Setting the centers of RBF c=Matrix5*2
    float _c_1_1 = -1.0f; float _c_1_2 = -0.5f; float _c_1_3 = 0.0f; float _c_1_4 = 0.5f; float _c_1_5 = 1.0f; 
    float _c_2_1 = -1.0f; float _c_2_2 = -0.5f; float _c_2_3 = 0.0f; float _c_2_4 = 0.5f; float _c_2_5 = 1.0f; 
    //定义 特定方向 第j个 隐藏层对应的RBF网络中心（可以理解为上面RBF网络中心矩阵的第j列）
    Vector2f _c_x_1, _c_x_2, _c_x_3, _c_x_4, _c_x_5; //x方向
    _c_x_1.x = _c_1_1; _c_x_1.y = _c_2_1;//第1个隐藏层中心2*1向量
    _c_x_2.x = _c_1_2; _c_x_2.y = _c_2_2;//第2个隐藏层中心2*1向量
    _c_x_3.x = _c_1_3; _c_x_3.y = _c_2_3;//第3个隐藏层中心2*1向量
    _c_x_4.x = _c_1_4; _c_x_4.y = _c_2_4;//第4个隐藏层中心2*1向量
    _c_x_5.x = _c_1_5; _c_x_5.y = _c_2_5;//第5个隐藏层中心2*1向量

    Vector2f _c_y_1, _c_y_2, _c_y_3, _c_y_4, _c_y_5; //y方向
    _c_y_1.x = _c_1_1; _c_y_1.y = _c_2_1;//第1个隐藏层中心2*1向量
    _c_y_2.x = _c_1_2; _c_y_2.y = _c_2_2;//第2个隐藏层中心2*1向量
    _c_y_3.x = _c_1_3; _c_y_3.y = _c_2_3;//第3个隐藏层中心2*1向量
    _c_y_4.x = _c_1_4; _c_y_4.y = _c_2_4;//第4个隐藏层中心2*1向量
    _c_y_5.x = _c_1_5; _c_y_5.y = _c_2_5;//第5个隐藏层中心2*1向量

    Vector2f _c_z_1, _c_z_2, _c_z_3, _c_z_4, _c_z_5; //z方向
    _c_z_1.x = _c_1_1; _c_z_1.y = _c_2_1;//第1个隐藏层中心2*1向量
    _c_z_2.x = _c_1_2; _c_z_2.y = _c_2_2;//第2个隐藏层中心2*1向量
    _c_z_3.x = _c_1_3; _c_z_3.y = _c_2_3;//第3个隐藏层中心2*1向量
    _c_z_4.x = _c_1_4; _c_z_4.y = _c_2_4;//第4个隐藏层中心2*1向量
    _c_z_5.x = _c_1_5; _c_z_5.y = _c_2_5;//第5个隐藏层中心2*1向量

    //设置RBF网络的宽度 Setting the width of the RBF network
    float _b_x = 0.6f;   
    float _b_y = 0.6f;  
    float _b_z = 1.0f;
    //Lyapunov矩阵P（通过MATLAB求解后在这里定义）
    float _P_x_1_1,_P_x_1_2,_P_x_2_1,_P_x_2_2;   
    _P_x_1_1 = 4.3333f;  _P_x_1_2 = 0.00100f;           (void)_P_x_1_1; //标记为未使用，避免编译的时候报错
    _P_x_2_1 = 3.3333f; _P_x_2_2 = 0.00120f;           (void)_P_x_2_1; //标记为未使用，避免编译的时候报错
    float _P_y_1_1,_P_y_1_2,_P_y_2_1,_P_y_2_2;
    _P_y_1_1 = 4.3333f;     _P_y_1_2 = 0.00100f;           (void)_P_y_1_1; //标记为未使用，避免编译的时候报错
    _P_y_2_1 = 3.3333f; _P_y_2_2 = 0.00120f;           (void)_P_y_2_1; //标记为未使用，避免编译的时候报错
    float _P_z_1_1,_P_z_1_2,_P_z_2_1,_P_z_2_2;
    _P_z_1_1 = 4.3333f;    _P_z_1_2 = 0.00100f;           (void)_P_z_1_1; //标记为未使用，避免编译的时候报错
    _P_z_2_1 = 3.3333f;_P_z_2_2 = 0.00120f;           (void)_P_z_2_1; //标记为未使用，避免编译的时候报错

    

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // reset input filter to value received //无人机重启pid时的初始化
    if (_reset_filter) { 
        _reset_filter = false;
        _error = (measurement - _target); 
        _error_m = (measurement - _target) / 100.0f; 
        _error_ori = (measurement - _target);
        _derivative.zero();  //.zero是vector3f自带的成员函数，可以便捷地初始化归零一个三维向量
        _integrator.zero();  //初始化归零积分项

        //初始化隐藏层输入X
        _X_x.x = _error.x; _X_x.y = _derivative.x; //x方向2*1
        _X_y.x = _error.y; _X_y.y = _derivative.y; //x方向2*1
        _X_z.x = _error.z; _X_z.y = _derivative.z; //x方向2*1

        //初始化隐藏层输出
        _h_x_1 = _h_x_2 = _h_x_3 = _h_x_4 = _h_x_5 = 0.0f; //x方向
        _h_y_1 = _h_y_2 = _h_y_3 = _h_y_4 = _h_y_5 = 0.0f; //y方向
        _h_z_1 = _h_z_2 = _h_z_3 = _h_z_4 = _h_z_5 = 0.0f; //z方向

        //初始化权重更新律
        _dot_W_x_1 = _dot_W_x_2 = _dot_W_x_3 = _dot_W_x_4 = _dot_W_x_5 = 0.0f;  //x方向
        _dot_W_y_1 = _dot_W_y_2 = _dot_W_y_3 = _dot_W_y_4 = _dot_W_y_5 = 0.0f;  //y方向
        _dot_W_z_1 = _dot_W_z_2 = _dot_W_z_3 = _dot_W_z_4 = _dot_W_z_5 = 0.0f;  //z方向
 
        //初始化权重
        _W_x_1 = _W_x_2 = _W_x_3 = _W_x_4 = _W_x_5 = 0.0f; //x方向
        _W_y_1 = _W_y_2 = _W_y_3 = _W_y_4 = _W_y_5 = 0.0f; //y方向
        _W_z_1 = _W_z_2 = _W_z_3 = _W_z_4 = _W_z_5 = 0.0f; //z方向

        //初始化神经网络输出_phi
        _phi_x = _phi_y = _phi_z = 0.0f;

        //定义自适应参数初始值
        _m_x =_m_y = _m_z = 2.0f;
        //初始化归零控制器输出
        _pdnn_output.x = 0;
        _pdnn_output.y = 0;
        _pdnn_output.z = 0;

        
    } else {
       
        Vector3f error_last{_error}; //将上一个循环计算出的误差 _error 存储到一个临时变量 error_last 中，用于后续的微分项计算。
        _error += ((measurement - _target) - _error) * get_filt_E_alpha(dt); //低通滤波：误差变化量*滤波系数 
        _error_m = _error / 100.0f; //转化单位为米
        

        // calculate and filter derivative
        if (is_positive(dt)) { //检查时间步长是否有效
            const Vector3f derivative{(_error - error_last) / dt}; //_error - error_last：计算当前误差与上一时刻误差之间的差，表示误差的变化量。计算误差变化量除以时间步长 dt，得到误差变化的速率，即微分项。
            _derivative += (derivative - _derivative) * get_filt_D_alpha(dt); //低通滤波：微分项变化量*滤波系数。使用低通滤波来对微分项进行平滑处理
            _derivative_m = _derivative / 100.0f; //转化单位为米
        }

        Vector3f error_ori_last{_error_ori};
        _error_ori = measurement - _target; //未经过滤波的误差
        if (is_positive(dt)) {
            _derivative_ori = (_error_ori - error_ori_last) / dt;
        }

        // update I term 更新积分项
        //void AC_PDNN_3D::update_i(float dt, float _ki, float _c1, float _kimax, bool limit)
        update_i(dt, 1.0f, 1.0f, 10.0f, true);
   
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~神经网络NN~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //定义隐藏层输入X
        _X_x.x = _error_m.x; _X_x.y = _derivative_m.x; //x方向2*1
        _X_y.x = _error_m.y; _X_y.y = _derivative_m.y; //y方向2*1
        _X_z.x = _error_m.z; _X_z.y = _derivative_m.z; //z方向2*1
        
        //计算隐藏层输出
        float L_x_1 = (_X_x-_c_x_1).length(); //存储欧式距离
        _h_x_1 = expf(-L_x_1*L_x_1/(2*_b_x*_b_x));//x方向第1个隐藏层输出
        float L_x_2 = (_X_x-_c_x_2).length(); //存储欧式距离
        _h_x_2 = expf(-L_x_2*L_x_2/(2*_b_x*_b_x));//x方向第2个隐藏层输出
        float L_x_3 = (_X_x-_c_x_3).length(); //存储欧式距离
        _h_x_3 = expf(-L_x_3*L_x_3/(2*_b_x*_b_x));//x方向第3个隐藏层输出
        float L_x_4 = (_X_x-_c_x_4).length(); //存储欧式距离
        _h_x_4 = expf(-L_x_4*L_x_4/(2*_b_x*_b_x));//x方向第4个隐藏层输出
        float L_x_5 = (_X_x-_c_x_5).length(); //存储欧式距离
        _h_x_5 = expf(-L_x_5*L_x_5/(2*_b_x*_b_x));//x方向第5个隐藏层输出

        float L_y_1 = (_X_y-_c_y_1).length(); //存储欧式距离
        _h_y_1 = expf(-L_y_1*L_y_1/(2*_b_y*_b_y));//y方向第1个隐藏层输出
        float L_y_2 = (_X_y-_c_y_2).length(); //存储欧式距离
        _h_y_2 = expf(-L_y_2*L_y_2/(2*_b_y*_b_y));//y方向第2个隐藏层输出
        float L_y_3 = (_X_y-_c_y_3).length(); //存储欧式距离
        _h_y_3 = expf(-L_y_3*L_y_3/(2*_b_y*_b_y));//y方向第3个隐藏层输出
        float L_y_4 = (_X_y-_c_y_4).length(); //存储欧式距离
        _h_y_4 = expf(-L_y_4*L_y_4/(2*_b_y*_b_y));//y方向第4个隐藏层输出
        float L_y_5 = (_X_y-_c_y_5).length(); //存储欧式距离
        _h_y_5 = expf(-L_y_5*L_y_5/(2*_b_y*_b_y));//y方向第5个隐藏层输出

         float L_z_1 = (_X_z-_c_z_1).length(); //存储欧式距离
        _h_z_1 = expf(-L_z_1*L_z_1/(2*_b_z*_b_z));//z方向第1个隐藏层输出
        float L_z_2 = (_X_z-_c_z_2).length(); //存储欧式距离
        _h_z_2 = expf(-L_z_2*L_z_2/(2*_b_z*_b_z));//z方向第2个隐藏层输出
        float L_z_3 = (_X_z-_c_z_3).length(); //存储欧式距离
        _h_z_3 = expf(-L_z_3*L_z_3/(2*_b_z*_b_z));//z方向第3个隐藏层输出
        float L_z_4 = (_X_z-_c_z_4).length(); //存储欧式距离
        _h_z_4 = expf(-L_z_4*L_z_4/(2*_b_z*_b_z));//z方向第4个隐藏层输出
        float L_z_5 = (_X_z-_c_z_5).length(); //存储欧式距离
        _h_z_5 = expf(-L_z_5*L_z_5/(2*_b_z*_b_z));//z方向第5个隐藏层输出

        //计算权重更新律
        float _gamma_x = 10000.0f;
        _dot_W_x_1 = _gamma_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _h_x_1;//x方向第1个权重更新律
        _dot_W_x_2 = _gamma_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _h_x_2;//x方向第2个权重更新律
        _dot_W_x_3 = _gamma_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _h_x_3;//x方向第3个权重更新律
        _dot_W_x_4 = _gamma_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _h_x_4;//x方向第4个权重更新律
        _dot_W_x_5 = _gamma_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _h_x_5;//x方向第5个权重更新律

        float _gamma_y = 10000.0f;
        _dot_W_y_1 = _gamma_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _h_y_1;//y方向第1个权重更新律
        _dot_W_y_2 = _gamma_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _h_y_2;//y方向第2个权重更新律
        _dot_W_y_3 = _gamma_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _h_y_3;//y方向第3个权重更新律
        _dot_W_y_4 = _gamma_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _h_y_4;//y方向第4个权重更新律
        _dot_W_y_5 = _gamma_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _h_y_5;//y方向第5个权重更新律

        float _gamma_z = 1000.0f;
        _dot_W_z_1 = _gamma_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _h_z_1;//z方向第1个权重更新律
        _dot_W_z_2 = _gamma_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _h_z_2;//z方向第2个权重更新律
        _dot_W_z_3 = _gamma_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _h_z_3;//z方向第3个权重更新律
        _dot_W_z_4 = _gamma_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _h_z_4;//z方向第4个权重更新律
        _dot_W_z_5 = _gamma_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _h_z_5;//z方向第5个权重更新律

        //计算当前权重（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的权重
          _W_x_1 += _dot_W_x_1 * dt;
          //_W_x_1 += (W_x_1 - _W_x_1) * get_filt_D_alpha(dt);
          _W_x_2 += _dot_W_x_2 * dt;
          //_W_x_2 += (W_x_2 - _W_x_2) * get_filt_D_alpha(dt);
          _W_x_3 += _dot_W_x_3 * dt;
          //_W_x_3 += (W_x_3 - _W_x_3) * get_filt_D_alpha(dt);
          _W_x_4 += _dot_W_x_4 * dt;
          //W_x_4 += (W_x_4 - _W_x_4) * get_filt_D_alpha(dt);
          _W_x_5 += _dot_W_x_5 * dt;
          //_W_x_5 += (W_x_5 - _W_x_5) * get_filt_D_alpha(dt);
          //y方向的权重
          _W_y_1 += _dot_W_y_1 * dt;
          _W_y_2 += _dot_W_y_2 * dt;
          _W_y_3 += _dot_W_y_3 * dt;
          _W_y_4 += _dot_W_y_4 * dt;
          _W_y_5 += _dot_W_y_5 * dt;
          //z方向的权重
          _W_z_1 += _dot_W_z_1 * dt;
          _W_z_2 += _dot_W_z_2 * dt;
          _W_z_3 += _dot_W_z_3 * dt;
          _W_z_4 += _dot_W_z_4 * dt;
          _W_z_5 += _dot_W_z_5 * dt;
        }

        //计算神经网络输出_phi = W' * h
        _phi_x = _W_x_1 * _h_x_1 + _W_x_2 * _h_x_2 + _W_x_3 * _h_x_3 + _W_x_4 * _h_x_4 + _W_x_5 * _h_x_5; //x方向神经网络输出
        _phi_y = _W_y_1 * _h_y_1 + _W_y_2 * _h_y_2 + _W_y_3 * _h_y_3 + _W_y_4 * _h_y_4 + _W_y_5 * _h_y_5; //y方向神经网络输出
        _phi_z = _W_z_1 * _h_z_1 + _W_z_2 * _h_z_2 + _W_z_3 * _h_z_3 + _W_z_4 * _h_z_4 + _W_z_5 * _h_z_5; //z方向神经网络输出
        //(void)_phi_x;
        //(void)_phi_y;
        //(void)_phi_z;//暂时标记为未使用，避免报错
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~自适应律~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        float _m_max = 3.5f; //定义自适应参数上限
        //~~~~~x方向
        float _eta_x = 0.05f; //定义自适应律参数，eta越大越平滑
        float _s_x = 0.002f;  //定义缩放因子
        if ((_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _pdnn_output.x > 0 )
        {
            _dot_m_x = -(_m_x * _m_x) / _eta_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _pdnn_output.x;
        }
        if ((_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _pdnn_output.x <= 0 )
        {
            if (_m_x < _m_max)
            {
                _dot_m_x = -(_m_x * _m_x) / _eta_x * (_error_m.x * _P_x_1_2 + _derivative_m.x * _P_x_2_2) * _pdnn_output.x;
            }
            else
            {
                _dot_m_x = -_s_x * (_m_x * _m_x);  
            }
        }
         //计算当前自适应参数（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的自适应参数_m_x
          _m_x += _dot_m_x * dt;
        }
        //~~~~~~y方向
        float _eta_y = 0.05f; //定义自适应律参数，eta越大越平滑
        float _s_y = 0.002f;  //定义缩放因子
        if ((_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _pdnn_output.y > 0 )
        {
            _dot_m_y = -(_m_y * _m_y) / _eta_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _pdnn_output.y;
        }
        if ((_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _pdnn_output.y <= 0 )
        {
            if (_m_y < _m_max)
            {
                _dot_m_y = -(_m_y * _m_y) / _eta_y * (_error_m.y * _P_y_1_2 + _derivative_m.y * _P_y_2_2) * _pdnn_output.y;
            }
            else
            {
                _dot_m_y = -_s_y * (_m_y * _m_y);  
            }
        }
         //计算当前自适应参数（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的自适应参数_m_x
          _m_y += _dot_m_y * dt;
        }
        //~~~~~~z方向
        float _eta_z = 2.0f; //定义自适应律参数，eta越大越平滑
        float _s_z = 0.002f;  //定义缩放因子
        if ((_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _pdnn_output.z > 0 )
        {
            _dot_m_z = -(_m_z * _m_z) / _eta_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _pdnn_output.z;
        }
        if ((_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _pdnn_output.z <= 0 )
        {
            if (_m_z < _m_max)
            {
                _dot_m_z = -(_m_z * _m_z) / _eta_z * (_error_m.z * _P_z_1_2 + _derivative_m.z * _P_z_2_2) * _pdnn_output.z;
            }
            else
            {
                _dot_m_z = -_s_z * (_m_z * _m_z);  
            }
        }
         //计算当前自适应参数（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的自适应参数_m_x
          _m_z += _dot_m_z * dt;
        }



        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
    _pdnn_info_z.P = _error.z * _kp_z;
    _pdnn_info_z.D = _derivative.z * _kd_z;
    _pdnn_info_z.FF = _target.z * _kff;//这里应该改成期望加速度前馈ddotZd

    _pdnn_output_P.x = _error.x * _kp; //计算P项输出
    _pdnn_output_P.y = _error.y * _kp;
    _pdnn_output_P.z = _error.z * _kp_z;

    _pdnn_output_D.x = _error.x * _kd; //计算D项输出
    _pdnn_output_D.y = _error.y * _kd; 
    _pdnn_output_D.z = _error.z * _kd_z;

    //_pdnn_output.x = _m_x * (-_error_m.x * _kp - 0.0f*_integrator.x - _derivative_m.x * _kd + _acc_desired.x - 1.0f*_phi_x); //计算总输出
    //_pdnn_output.y = _m_y * (-_error_m.y * _kp - 0.0f*_integrator.y - _derivative_m.y * _kd + _acc_desired.y -1.0f * _phi_y);
    //_pdnn_output.z = _m_z * (-_error_m.z * _kp_z - 0.0f*_integrator.z - _derivative_m.z * _kd_z - 9.80665f + _acc_desired.z -1.0f *_phi_z); //加入重力值9.80665, NED坐标系下为负数


    _pdnn_output.x =1.3f*(-_error_m.x * 5.0f - 0.0f*_integrator.x - _derivative_m.x * 4.0f + _acc_desired.x - 0.0f*_phi_x); //计算总输出
    _pdnn_output.y =1.3f*(-_error_m.y * 5.0f - 0.0f*_integrator.y - _derivative_m.y * 4.0f + _acc_desired.y -0.0f * _phi_y);
    _pdnn_output.z =1.3f*(-_error_m.z * 7.0f - 0.0f*_integrator.z - _derivative_m.z * 3.0f - 9.80665f + _acc_desired.z -0.0f *_phi_z); //加入重力值9.80665, NED坐标系下为负数

    return _pdnn_output; //返回pdnn控制器输出

}

void AC_PDNN_3D::update_i(float dt, float _ki, float _c1, float _kimax, bool limit)
{
   if (limit){

    //Vector3f delta_integrator = (_derivative_m + _error_m * _c1) * dt;
    Vector3f delta_integrator = (_error_m * _c1) * dt;
    _integrator += delta_integrator;
    
    float _integrator_x = _integrator.x;
    float _integrator_y = _integrator.y;
    float _integrator_z = _integrator.z;
    _integrator_x = constrain_float(_integrator_x, -_kimax, _kimax); //分别在xyz方向上限制积分大小
    _integrator_y = constrain_float(_integrator_y, -_kimax, _kimax);
    _integrator_z = constrain_float(_integrator_z, -_kimax, _kimax);

    _integrator.x = _integrator_x; //把限制后的值重新赋值
    _integrator.y = _integrator_y;
    _integrator.z = _integrator_z;

    _integrator =  _integrator * _ki;
} else{
    _integrator.zero();
}

}


Vector3f AC_PDNN_3D::get_p() const
{
    return _pdnn_output_P;
}

Vector3f AC_PDNN_3D::get_d() const
{
    return _pdnn_output_D;
}

Vector3f AC_PDNN_3D::get_ff()
{
    _pdnn_info_x.FF = _target.x * _kff;
    _pdnn_info_y.FF = _target.y * _kff;
    _pdnn_info_z.FF = _target.z * _kff;
    return _target * _kff;
}

Vector3f AC_PDNN_3D::get_phi() const
{   
    Vector3f _phi;
    _phi.x = _phi_x;
    _phi.y = _phi_y;
    _phi.z = _phi_z;
    return _phi;
}

Vector3f AC_PDNN_3D::get_m() const
{   
    Vector3f _m;
    _m.x = _m_x;
    _m.y = _m_y;
    _m.z = _m_z;
    return _m;
}



// save_gains - save gains to eeprom
void AC_PDNN_3D::save_gains()
{
    _kp.save();
    _kd.save();
    _kp_z.save();
    _kd_z.save();
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
