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
Vector3f AC_PDNN_SO3::update_all(const Matrix3f &R_c, const Matrix3f &R, const Vector3f &Omega, float dt, bool Rc_active)
{
    // don't process inf or NaN //检查输入的有效性，避免处理空值NaN与无穷大inf的值
    if (R_c.is_nan() || R.is_nan()) {
        return Vector3f{}; //返回一个vector3f避免报错
    }
    _R = R;
    _Omega = Omega;
    _Rc_active = Rc_active; //检查位置控制环是否被调用
    //更新_Omega_hat斜对称矩阵
    _Omega_hat.a.x = 0.0f;_Omega_hat.a.y = -_Omega.z;_Omega_hat.a.z = _Omega.y;
    _Omega_hat.b.x = _Omega.z;_Omega_hat.b.y = 0.0f;_Omega_hat.b.z = -_Omega.x;
    _Omega_hat.c.x = -_Omega.y;_Omega_hat.c.y = _Omega.x;_Omega_hat.c.z = 0.0f;
 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Neural Networks变量声明和定义~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//设置默认RBF网络中心矩阵 Setting the centers of RBF c=Matrix5*2
    float _c_1_1 = -1.0f; float _c_1_2 = -0.5f; float _c_1_3 = 0.0f; float _c_1_4 = 0.5f; float _c_1_5 = 1.0f; 
    float _c_2_1 = -10.0f; float _c_2_2 = -5.0f; float _c_2_3 = 0.0f; float _c_2_4 = 5.0f; float _c_2_5 = 10.0f; 
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
    _c_z_1.x = _c_1_1; _c_z_1.y = -6.0f;//第1个隐藏层中心2*1向量
    _c_z_2.x = _c_1_2; _c_z_2.y = -3.0f;//第2个隐藏层中心2*1向量
    _c_z_3.x = _c_1_3; _c_z_3.y = 0.0f;//第3个隐藏层中心2*1向量
    _c_z_4.x = _c_1_4; _c_z_4.y = 3.0f;//第4个隐藏层中心2*1向量
    _c_z_5.x = _c_1_5; _c_z_5.y = 6.0f;//第5个隐藏层中心2*1向量

    //设置RBF网络的宽度 Setting the width of the RBF network 注意！！：宽度越大约平滑，太小会发散
    float _b_x = 2.0f;   
    float _b_y = 2.0f;  
    float _b_z = 3.0f;
    
    // reset input filter to value received //无人机重启pdnn姿态控制时的初始化
    if (_reset) { //初始化逻辑
        _reset = false;
        
        _R_c = R_c; //更新当前循环的_R_c
        
        //旋转矩阵误差_e_R初始化
        _e_R_hat = (_R_c.transposed() * _R - _R.transposed() * _R_c) * 0.5f; //计算旋转矩阵误差的斜对称矩阵, 注意要把0.5f放在Matrix3f后面，因为函数重载的格式要求
        _e_R.x = -_e_R_hat.b.z; //斜对称矩阵.V逆运算，对_e_R进行赋值得到旋转矩阵误差_e_R，注意是一个Vector3f
        _e_R.y = _e_R_hat.a.z;
        _e_R.z = _e_R_hat.b.x;

        //Psi_R姿态误差标量函数初始化
        _Psi_R = (1.0f-(_R_c.transposed() * _R).a.x + 1.0f - (_R_c.transposed() * _R).b.y + 1.0f - (_R_c.transposed() * _R).c.z) * 0.5f;

        //防止微分爆炸，初始化微分项
        _dot_R_c.zero();
        _dot_Omega_c.zero();
        _Omega_c.zero();

        //角速度误差_e_Omega初始化
        //！！！先尝试初始化为Omega，因为前面初始化了微分项！！这里可能需要修改
        _e_Omega = _Omega;

        //初始化隐藏层输入X
        _X_x.x = _e_R.x; _X_x.y = _e_Omega.x; //x方向2*1
        _X_y.x = _e_R.y; _X_y.y = _e_Omega.y; //x方向2*1
        _X_z.x = _e_R.z; _X_z.y = _e_Omega.z; //x方向2*1

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
        _J_x = 0.01f;_J_y = 0.02f;_J_z = 0.02f;
        //初始化归零控制器输出
        _pdnn_output.x = 0;
        _pdnn_output.y = 0;
        _pdnn_output.z = 0;


    } else { //更新循环
        Matrix3f _R_c_last{_R_c}; //将上一个循环的_R_c存储到一个临时变量 _R_c_last 中，用于后续的微分项计算。这里用到拷贝函数，等价于Matrix3f error_last = _error;
        
        //更新当前循环的_R_c
        _R_c = R_c; 

        //更新当前循环旋转矩阵误差_e_R
        _e_R_hat = (_R_c.transposed() * _R - _R.transposed() * _R_c) * 0.5f; //计算旋转矩阵误差的斜对称矩阵，注意要把0.5f放在Matrix3f后面，因为函数重载的格式要求
        _e_R.x = -_e_R_hat.b.z; //斜对称矩阵.V逆运算，对_e_R进行赋值得到旋转矩阵误差_e_R，注意是一个Vector3f
        _e_R.y = _e_R_hat.a.z;
        _e_R.z = _e_R_hat.b.x;
       
        //更新Psi_R姿态误差标量函数
        _Psi_R = (1.0f-(_R_c.transposed() * _R).a.x + 1.0f - (_R_c.transposed() * _R).b.y + 1.0f - (_R_c.transposed() * _R).c.z) * 0.5f;

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

        //计算_Omega_c微分项，这里进行了滤波来消除微分爆炸（重要）
        if (is_positive(dt)) { //检查时间步长是否有效
            const Vector3f dot_Omega_c{(_Omega_c - _Omega_c_last) / dt}; //_error - error_last：计算当前误差与上一时刻误差之间的差，表示误差的变化量。计算误差变化量除以时间步长 dt，得到误差变化的速率，即微分项。
            _dot_Omega_c += (dot_Omega_c - _dot_Omega_c) * get_filt_D_alpha(dt);
            //_dot_Omega_c = (_Omega_c - _Omega_c_last) / dt;  //理论上应该可以实现逐元素求导，这里是不滤波的代码
        }

        //update I term 更新积分项
        //void AC_PDNN_3D::update_i(float dt, float _ki, float _c1, float _kimax, bool limit)
        update_i(dt, 1.0f, 100.0f, 200.0f, true); //尽量小，姿态控制要求实时性

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~神经网络NN~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        if (_Rc_active) {       //只在takeoffmode启用，Rc被调用时开始输入
        //定义隐藏层输入X
        _X_x.x = _e_R.x; _X_x.y = _e_Omega.x; //x方向2*1
        _X_y.x = _e_R.y; _X_y.y = _e_Omega.y; //y方向2*1
        _X_z.x = _e_R.z; _X_z.y = _e_Omega.z; //z方向2*1 

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
        _h_z_2 = expf(-L_z_2*L_z_2/(2*3.0f*3.0f));//z方向第2个隐藏层输出
        float L_z_3 = (_X_z-_c_z_3).length(); //存储欧式距离
        _h_z_3 = expf(-L_z_3*L_z_3/(2*3.0f*3.0f));//z方向第3个隐藏层输出
        float L_z_4 = (_X_z-_c_z_4).length(); //存储欧式距离
        _h_z_4 = expf(-L_z_4*L_z_4/(2*3.0f*3.0f));//z方向第4个隐藏层输出
        float L_z_5 = (_X_z-_c_z_5).length(); //存储欧式距离
        _h_z_5 = expf(-L_z_5*L_z_5/(2*_b_z*_b_z));//z方向第5个隐藏层输出

        //计算权重更新律
        float _gamma_x = 120.0f;float c_R = 0.6f;
        _dot_W_x_1 = _gamma_x * (_e_Omega.x + c_R * _e_R.x) * _h_x_1;//x方向第1个权重更新律
        _dot_W_x_2 = _gamma_x * (_e_Omega.x + c_R * _e_R.x) * _h_x_2;//x方向第2个权重更新律
        _dot_W_x_3 = _gamma_x * (_e_Omega.x + c_R * _e_R.x) * _h_x_3;//x方向第3个权重更新律
        _dot_W_x_4 = _gamma_x * (_e_Omega.x + c_R * _e_R.x) * _h_x_4;//x方向第4个权重更新律
        _dot_W_x_5 = _gamma_x * (_e_Omega.x + c_R * _e_R.x) * _h_x_5;//x方向第5个权重更新律

        float _gamma_y = 120.0f;
        _dot_W_y_1 = _gamma_y * (_e_Omega.y + c_R * _e_R.y) * _h_y_1;//y方向第1个权重更新律
        _dot_W_y_2 = _gamma_y * (_e_Omega.y + c_R * _e_R.y) * _h_y_2;//y方向第2个权重更新律
        _dot_W_y_3 = _gamma_y * (_e_Omega.y + c_R * _e_R.y) * _h_y_3;//y方向第3个权重更新律
        _dot_W_y_4 = _gamma_y * (_e_Omega.y + c_R * _e_R.y) * _h_y_4;//y方向第4个权重更新律
        _dot_W_y_5 = _gamma_y * (_e_Omega.y + c_R * _e_R.y) * _h_y_5;//y方向第5个权重更新律

        float _gamma_z = 50.0f;
        _dot_W_z_1 = _gamma_z * (_e_Omega.z + c_R * _e_R.z) * _h_z_1;//z方向第1个权重更新律
        _dot_W_z_2 = _gamma_z * (_e_Omega.z + c_R * _e_R.z) * _h_z_2;//z方向第2个权重更新律
        _dot_W_z_3 = _gamma_z * (_e_Omega.z + c_R * _e_R.z) * _h_z_3;//z方向第3个权重更新律
        _dot_W_z_4 = _gamma_z * (_e_Omega.z + c_R * _e_R.z) * _h_z_4;//z方向第4个权重更新律
        _dot_W_z_5 = _gamma_z * (_e_Omega.z + c_R * _e_R.z) * _h_z_5;//z方向第5个权重更新律

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
        }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~自适应律~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        float _J_max = 0.03f; //定义自适应参数上限
        //~~~~~x方向
        float _eta_x = 0.01f; //定义自适应律参数，eta越大越平滑
        float _s_x = 0.02f;  //定义缩放因子
        float c_R = 0.6f;
        if ((_e_Omega.x + c_R * _e_R.x) * _pdnn_output.x > 0 )
        {
            _dot_J_x = -(_J_x * _J_x) / _eta_x * (_e_Omega.x + c_R * _e_R.x) * _pdnn_output.x;
        }
        if ((_e_Omega.x + c_R * _e_R.x) * _pdnn_output.x <= 0 )
        {
            if (_J_x < _J_max)
            {
                _dot_J_x = -(_J_x * _J_x) / _eta_x * (_e_Omega.x + c_R * _e_R.x) * _pdnn_output.x;
            }
            else
            {
                _dot_J_x = -_s_x * (_J_x * _J_x);  
            }
        }
         //计算当前自适应参数（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的自适应参数_m_x
          _J_x += _dot_J_x * dt;
        }

        //~~~~~~y方向
        float _eta_y = 0.01f; //定义自适应律参数，eta越大越平滑
        float _s_y = 0.02f;  //定义缩放因子
        if ((_e_Omega.y + c_R * _e_R.y) * _pdnn_output.y > 0 )
        {
            _dot_J_y = -(_J_y * _J_y) / _eta_y * (_e_Omega.y + c_R * _e_R.y) * _pdnn_output.y;
        }
        if ((_e_Omega.y + c_R * _e_R.y) * _pdnn_output.y <= 0 )
        {
            if (_J_y < _J_max)
            {
                _dot_J_y = -(_J_y * _J_y) / _eta_y * (_e_Omega.y + c_R * _e_R.y) * _pdnn_output.y;
            }
            else
            {
                _dot_J_y = -_s_y * (_J_y * _J_y);  
            }
        }
         //计算当前自适应参数（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的自适应参数_m_x
          _J_y += _dot_J_y * dt;
        }

        //~~~~~~z方向
        float _J_max_z = 0.04f; //定义自适应参数上限
        float _eta_z = 0.05f; //定义自适应律参数，eta越大越平滑
        float _s_z = 0.02f;  //定义缩放因子
        if ((_e_Omega.z + c_R * _e_R.z) * _pdnn_output.z > 0 )
        {
            _dot_J_z = -(_J_z * _J_z) / _eta_z * (_e_Omega.z + c_R * _e_R.z) * _pdnn_output.z;
        }
        if ((_e_Omega.z + c_R * _e_R.z) * _pdnn_output.z <= 0 )
        {
            if (_J_z < _J_max_z)
            {
                _dot_J_z = -(_J_z * _J_z) / _eta_z * (_e_Omega.z + c_R * _e_R.z) * _pdnn_output.z;
            }
            else
            {
                _dot_J_z = -_s_z * (_J_z * _J_z);  
            }
        }
         //计算当前自适应参数（求积分）
        if (is_positive(dt)) { //检查时间步长是否有效

        //x方向的自适应参数_m_x
          _J_z += _dot_J_z * dt;
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    }
   
   
    _pdnn_output_R.x = -_e_R.x * _kR; //计算P项输出
    _pdnn_output_R.y = -_e_R.y * _kR;
    _pdnn_output_R.z = -_e_R.z * _kR_z;

    _pdnn_output_Omega.x = -_e_Omega.x * _kOmega; //计算D项输出
    _pdnn_output_Omega.y = -_e_Omega.y * _kOmega;
    _pdnn_output_Omega.z = -_e_Omega.z * _kOmega_z;
    
    //计算几何控制项，这里惯性张量
    Matrix3f J;
    J.a.x=0.01f;J.a.y=0.0f;     J.a.z=0.0f;
    J.b.x=0.0f;      J.b.y=0.02f;J.b.z=0.0f;
    J.c.x=0.0f;      J.c.y=0.0f;     J.c.z=0.02f;
    Matrix3f J_inv;
    J_inv = J;
    J_inv.a.x = 1/J.a.x;J_inv.b.y = 1/J.b.y;J_inv.c.z = 1/J.c.z;
    //额外增广项，以实现UPAS
    Vector3f Aug;
    Aug = J_inv * _Omega_hat * J * _Omega;

    _geomrtry_output = _Omega_hat * _R.transposed() * _R_c * _Omega_c  - _R.transposed() * _R_c * _dot_Omega_c;

    //(void)_geomrtry_output;

    //计算总输出，每个方向上乘以惯性张量
    //_pdnn_output.x = _J_x * (-_e_R.x * 100.0f - _e_Omega.x * 80.0f - 0.0f * _integrator.x - _geomrtry_output.x - 1.0f *_phi_x + 0.0f*Aug.x); 
    //_pdnn_output.y = _J_y * (-_e_R.y * 100.0f - _e_Omega.y * 80.0f - 0.0f *_integrator.y - _geomrtry_output.y- 1.0f * _phi_y + 0.0f*Aug.y);
    //_pdnn_output.z = _J_z * (-_e_R.z * 100.0f - _e_Omega.z * 80.0f - 0.0f *_integrator.z - _geomrtry_output.z - 1.0f *_phi_z + 0.0f*Aug.z); //偏航误差e_R.z很容易就趋近于0，会导致无法满足持续激励假设
    
    _pdnn_output.x = 0.01f * (-_e_R.x * 80.0f - _e_Omega.x * 80.0f - 0.0f * _integrator.x - _geomrtry_output.x - 0.0f *_phi_x + 0.0f*Aug.x); 
    _pdnn_output.y = 0.01f * (-_e_R.y * 80.0f - _e_Omega.y * 80.0f - 0.0f *_integrator.y - _geomrtry_output.y- 0.0f * _phi_y + 0.0f*Aug.y);
    _pdnn_output.z = 0.02f * (-_e_R.z * 80.0f - _e_Omega.z * 80.0f - 0.0f *_integrator.z - _geomrtry_output.z - 0.0f *_phi_z + 0.0f*Aug.z); //偏航误差e_R.z很容易就趋近于0，会导致无法满足持续激励假设

    return _pdnn_output; //返回pdnn控制器输出
}

void AC_PDNN_SO3::update_i(float dt, float _ki, float _c2, float _kimax, bool limit)
{
   if (limit){

    //Vector3f delta_integrator = (_e_Omega + _e_R * _c2) * dt;
    Vector3f delta_integrator = (_e_R * _c2) * dt;
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


Vector3f AC_PDNN_SO3::get_R() const
{
    return _pdnn_output_R;
}

Vector3f AC_PDNN_SO3::get_Omega() const
{
    return _pdnn_output_Omega;
}

Vector3f AC_PDNN_SO3::get_e_Omega() const
{   
    
    return _e_Omega;
}

Vector3f AC_PDNN_SO3::get_e_R() const
{   
    
    return _e_R;
}

Vector3f AC_PDNN_SO3::get_dot_Omega_c() const
{   
    
    return _dot_Omega_c;
}

Vector3f AC_PDNN_SO3::get_phi() const
{   
    Vector3f _phi;
    _phi.x = _phi_x;
    _phi.y = _phi_y;
    _phi.z = _phi_z;
    return _phi;
}

Vector3f AC_PDNN_SO3::get_J() const
{   
    Vector3f _J;
    _J.x = _J_x;
    _J.y = _J_y;
    _J.z = _J_z;
    return _J;
}

float AC_PDNN_SO3::get_Psi_R() const
{
    return _Psi_R;
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

// get the target filter alpha
float AC_PDNN_SO3::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, 5.0f);
}

// get the derivative filter alpha
float AC_PDNN_SO3::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, 5.0f);
}


  
