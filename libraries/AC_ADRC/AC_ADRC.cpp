/********************************************************************************************************
 *********************************************************************************************************
 *文件名称：AC_ADRC.cpp
 *功能描述：
 *修改作者：coco
 *修改时间：2026-9-4
 *修改内容：
 *备注信息： ADRC 控制器
 **********************************************************************************************************
 **********************************************************************************************************/
#include "AC_ADRC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#define  ADRC_ATT_DT_MIN     0.0005f              // 最小采样周期 (2000Hz)
#define  ADRC_ATT_DT_MAX     0.02f                // 最大采样周期 (50Hz),保证wo*dt<0.6


extern const AP_HAL::HAL& hal;

// 参数表定义
const AP_Param::GroupInfo AC_ADRC::var_info[] = {
	// @Param: MD
	// @DisplayName: 控制模式类型
	// @Description:
	// @Range:
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("MD",0, AC_ADRC, _adrc_type, default_adrc_type),
	// @Param: B0
	// @DisplayName: 系统增益估计
	// @Description: 被控对象增益估计。位置环输入为加速度，理论 b0=100
	// @Range: 0.1 2000
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B0",1, AC_ADRC, _b0,        default_b0),

    // @Param: TDR
    // @DisplayName: TD 速度因子
    // @Description: 跟踪微分器的速度因子，越大过渡越快。建议 50~500
    // @Range: 50 500
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("TDR",2, AC_ADRC, _td_r,     default_tdr),
    // @Param: TDH
    // @DisplayName: TD 滤波因子
    // @Description: 跟踪微分器滤波因子，建议取 dt 的 1~10 倍。运行时自动 max(h0, dt)
    // @Range: 0.0025---- 0.025
    // @Units: s
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("TDH", 3, AC_ADRC, _td_h0,     default_td_h0),


	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B1",4, AC_ADRC, _eso_beta1, default_eso_beta1),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B2",5, AC_ADRC, _eso_beta2, default_eso_beta2),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B3",6, AC_ADRC, _eso_beta3, default_eso_beta3),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D1",7, AC_ADRC, _eso_delta,    default_eso_delta),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("EH",8, AC_ADRC, _eso_h_gain, default_eso_h_gain),


    // @Param: A1
    // @DisplayName: NLSEF 位置误差非线性指数
    // @Description: fal 函数中位置误差的非线性指数。建议 0.5~0.75，越小非线性越强
    // @Range: 0.1 1.0
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("A1",        9, AC_ADRC, _nlsef_alpha1,    default_nlsef_alpha1),

    // @Param: A2
    // @DisplayName: NLSEF 速度误差非线性指数
    // @Description: fal 函数中速度误差的非线性指数。建议 1.25~1.5（大误差强阻尼）
    // @Range: 1.25 1.5
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("A2",        10, AC_ADRC, _nlsef_alpha2,    default_nlsef_alpha2),

    // @Param: DELTA
    // @DisplayName: fal 线性段阈值
    // @Description: fal 函数线性段与非线性段的分界阈值。小误差线性，大误差非线性
    // @Range: 0.001 1.0
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D2",     11, AC_ADRC, _nlsef_delta,    default_nlsef_delta),
    // @Param: AMAX
    // @DisplayName: 输出加速度上限
    // @Description:ADRC 输出的最大加速度限制，与 POS_ACCEL_XY 保持一致
    // @Range: 100 500
    // @Units: cm/s/s

	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("KP",12, AC_ADRC, _nlsef_kp, default_nlsef_kp),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("KD",13, AC_ADRC, _nlsef_kd, default_nlsef_kd),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("UM",14, AC_ADRC, _limit_u_max, default_limit_u_max),
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLT", 15, AC_ADRC, _filt_hz, default_filt_hz),
	AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WC",16, AC_ADRC, _wc, default_wc),
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WO", 17, AC_ADRC, _wo, default_wo),
    AP_GROUPEND
};


// 构造函数
AC_ADRC::AC_ADRC(const AC_ADRC::Defaults &defaults) :
		default_adrc_type(defaults.adrc_type), //模式类型
		default_b0(defaults.b0),                     //参数b0
		default_tdr(defaults.td_r),                  //TD-R
		default_td_h0(defaults.td_h0),               //TD-H0
		default_eso_beta1(defaults.eso_beta1),       //ESO-B1
		default_eso_beta2(defaults.eso_beta2),       //ESO-B2
		default_eso_beta3(defaults.eso_beta3),       //ESO-B3
		default_eso_delta(defaults.eso_delta),       //ESO-DELT
		default_eso_h_gain(defaults.eso_h_gain),     //ESO-步长
		default_nlsef_alpha1(defaults.nlsef_alpha1), //NLSEF-A1
		default_nlsef_alpha2(defaults.nlsef_alpha2), //NLSEF-A2
		default_nlsef_delta(defaults.nlsef_delta),   //NLSEF-DELT
		default_nlsef_kp(defaults.nlsef_kp),         //NLSEF-KP
		default_nlsef_kd(defaults.nlsef_kd),         //NLSEFKD
		default_limit_u_max(defaults.limit_u_max),   //输出限制
		default_filt_hz(defaults.fil_hz) ,      //数据滤波处理-10hz
		default_wc(defaults.wc) ,       //控制带宽
		default_wo(defaults.wo)         //观测带宽(3-5)*wc

{
    AP_Param::setup_object_defaults(this, var_info);

     //复位数据
     reset_filter(0.0f,0.0f);
}


/********************************************************************************************************************************************************
*函数原型：update_axis
*函数功能：
*修改日期：2026-9-4
*修改作者：
*备注信息：单轴更新（内部）
******************************************************************************************************************************************************/
float AC_ADRC::update_axis(float target, float measure, float dt,float& v1, float& v2,float& z1, float& z2, float& z3,float& last_u,bool limit,uint8_t aix)
{
	//日子记录使用
    static uint16_t log_loop_delay1=0;
    static uint16_t log_loop_delay2=0;
    static uint16_t log_loop_delay3=0;

	//目标滤波
	_target=_target+get_filt_alpha(dt)*(target-_target);
    //测量滤波
	_measure=_measure+get_filt_alpha(dt)*(measure-_measure);
    //使用的adrc类型-1阶ladrc 2阶eso
	if(_adrc_type==0)
	{

	    //伪代码示例，update_axis开头
	    if (!is_valid_data(z1) || !is_valid_data(z2) || !is_valid_data(z3)
	    	|| !is_valid_data(v1) || !is_valid_data(v2))
	    {
	    	// 重置该轴状态，避免永久锁死
	    	v1 = 0;
	    	v2 = 0;
	    	z1 = measure;
	    	_measure=measure;
	    	_last_measure=measure;
	    	_target=target;
	    	_last_target=target;
	    	z2 = 0;
	    	z3 = 0;
	    	last_u = 0.0f;
	        return 0.0f;
	    }
        //获取参数信息
	    const float b0   = MAX(_b0.get(),1.0f);                   // 系统增益估计
	    const float output_max  = MAX(_limit_u_max.get(),1.0f);   //输出限制
        //总共3个参数-这里三其中2个参数
        float temp_kp=MAX(_wc.get(),0.0f);   //wc
        float temp_wo=MAX(_wo.get(),0.0f);   //wo
        float temp_beta1=2.0f*temp_wo;      //2*wo
        float temp_beta2=temp_wo*temp_wo;   //wo*wo
        //估计误差
	    float error_eso = z1 - _measure;
	    z1 = z1 + dt * (z2 - temp_beta1 * error_eso+ b0 * last_u);
	    z2 = z2 + dt * ( - temp_beta2 * error_eso );
	    //限制z2不能太大
	    z2=constrain_float(z2, -b0*output_max, b0*output_max);

        //计算控制误差
	    float error_control = _target - z1;
	    //计算u0
	    float u0= temp_kp * error_control ;
	    // ================================================================
	    // 第四步：扰动补偿
	    //   u = (u0 - z2) / b0
	    // ================================================================
	    float u_temp = (u0 - z2) / b0;
        //最终输出
	    float u = constrain_float(u_temp, -output_max, output_max);

	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）
	    if(aix==0)
	    {
	    	log_loop_delay1++;
	        if(log_loop_delay1>=10)
	        {
	        	log_loop_delay1=0;
	            AP::logger().Write("RARC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
	        		   (float)target, //t1
	        		   (float)_target, //t2
	        		   (float)measure,     //t3
	        		   (float)_measure,   //t4
	        		   (float)error_eso,   //t5
	        		   (float)error_control,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)temp_beta2 * error_eso,      //t9
	        		   (float)temp_kp * error_control  ,      //t10
	        		   (float) temp_beta1 * error_eso,      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(u0 - z2),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }
	    if(aix==1)
	    {

	    	log_loop_delay2++;
	        if(log_loop_delay2>=10)
	        {
	        	log_loop_delay2=0;
	            AP::logger().Write("PARC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
	        		   (float)target, //t1
	        		   (float)_target, //t2
	        		   (float)measure,     //t3
	        		   (float)_measure,   //t4
	        		   (float)error_eso,   //t5
	        		   (float)error_control,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)temp_beta2 * error_eso,      //t9
	        		   (float)temp_kp * error_control,      //t10
	        		   (float)temp_beta1 * error_eso,      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(u0 -z2),      //t14
					   (float)(u_temp)     //t15

	        		   );
	        }
	    }
	    if(aix==2)
	    {

	    	log_loop_delay3++;
	        if(log_loop_delay3>=10)
	        {
	        	log_loop_delay3=0;
	            AP::logger().Write("YARC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
	        		   (float)target, //t1
	        		   (float)_target, //t2
	        		   (float)measure,     //t3
	        		   (float)_measure,   //t4
	        		   (float)error_eso,   //t5
	        		   (float)error_control,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)temp_beta2 * error_eso,      //t9
	        		   (float)temp_kp * error_control,      //t10
	        		   (float)temp_beta1 * error_eso,      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(u0 -z2),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }
	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）
	    return u;
	}
    else if(_adrc_type==1) //2阶ladrc-3阶eso
	{
	    //伪代码示例，update_axis开头
	    if (!is_valid_data(z1) || !is_valid_data(z2) || !is_valid_data(z3)
	    	|| !is_valid_data(v1) || !is_valid_data(v2))
	    {
	    	// 重置该轴状态，避免永久锁死
	    	v1 = 0;
	    	v2 = 0;
	    	z1 = measure;
	    	z2 = 0;
	    	z3 = 0;
	    	last_u = 0.0f;
	    	_measure=measure;
	    	_last_measure=measure;
	    	_target=target;
	    	_last_target=target;
	        return 0.0f;
	    }

	    const float b0   = MAX(_b0.get(),1.0f);      // 系统增益估计
	    const float output_max  = MAX(_limit_u_max.get(),0.1f);   //输出限制
	    float temp_wc=MAX(_wc.get(),0.0f);   //_wc
        float temp_wo=MAX(_wo.get(),0.0f);   //wo


        float temp_kp=temp_wc*temp_wc; //wc*wc
        float temp_kd=2.0f*temp_wc; //2*wc
        float temp_beta1=3.0f*temp_wo;//3*wo
        float temp_beta2=3.0f*temp_wo*temp_wo; //3*wo*wo
        float temp_beta3=temp_wo*temp_wo*temp_wo; //wo*wo*wo
        //ESO 估计误差
	    float error_eso = z1 - _measure;
	    z1 = z1 + dt * (z2 - temp_beta1 * error_eso); //
	    z2 = z2 + dt * (z3 - temp_beta2 * error_eso + b0 * last_u);
	    z3 = z3 + dt * (-temp_beta3 * error_eso);
	    //限制z2,z3不能太大
	    z2=constrain_float(z2, -b0*output_max, b0*output_max);
	    z3=constrain_float(z3, -b0*output_max, b0*output_max);
        //计算控制误差
	    float error_control = _target - z1;
	    float u0= temp_kp * error_control - temp_kd * z2;
	    // ================================================================
	    // 第四步：扰动补偿
	    //   u = (u0 - z3) / b0
	    // ================================================================
	    float u_temp = (u0 - z3) / b0;
	    float u = constrain_float(u_temp, -output_max, output_max);
	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）

	    if(aix==0)
	    {
	    	log_loop_delay1++;
	        if(log_loop_delay1>=10)
	        {
	        	log_loop_delay1=0;
	            AP::logger().Write("RBRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
	        		   (float)error_eso,   //t5
	        		   (float)error_control,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * error_control  ,      //t10
	        		   (float) -(temp_kd * z2),      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(u0 - z3),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }

	    if(aix==1)
	    {

	    	log_loop_delay2++;
	        if(log_loop_delay2>=10)
	        {
	        	log_loop_delay2=0;
	            AP::logger().Write("PBRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
	        		   (float)error_eso,   //t5
	        		   (float)error_control,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * error_control,      //t10
	        		   (float)-(temp_kd * z2),      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(u0 -z3),      //t14
					   (float)(u_temp)     //t15

	        		   );
	        }
	    }
	    if(aix==2)
	    {
	    	log_loop_delay3++;
	        if(log_loop_delay3>=10)
	        {
	        	log_loop_delay3=0;
	            AP::logger().Write("YBRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
	        		   (float)error_eso,   //t5
	        		   (float)error_control,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * error_control,      //t10
	        		   (float)-(temp_kd * z2),      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(u0 -z3),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }

	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）
	    return u;
	}
	else if(_adrc_type==2)//数字离散极点配置发2阶-ladrc
	{
	    //伪代码示例，update_axis开头
	    if (!is_valid_data(z1) || !is_valid_data(z2) || !is_valid_data(z3)
	    	|| !is_valid_data(v1) || !is_valid_data(v2))
	    {
	    	// 重置该轴状态，避免永久锁死
	    	v1 = 0;
	    	v2 = 0;
	    	z1 = measure;
	    	z2 = 0;
	    	z3 = 0;
	    	last_u = 0.0f;
	    	_measure=measure;
	    	_last_measure=measure;
	    	_target=target;
	    	_last_target=target;
	        return 0.0f;
	    }

	    const float b0   = MAX(_b0.get(),1.0f);      // 系统增益估计
	    const float output_max  = MAX(_limit_u_max.get(),0.1f);   //输出限制
	    float temp_wc=MAX(_wc.get(),0.0f);   //_wc
        float temp_wo=MAX(_wo.get(),0.0f);   //wo


        float temp_kp=temp_wc*temp_wc; //wc*wc
        float temp_kd=2.0f*temp_wc; //2*wc


        float r = expf(-temp_wo * dt);
        float temp_beta1 = 3.0f * (1.0f - r);
        float temp_beta2 = (1.0f - r) * (1.0f - r) * (5.0f + r) / (2.0f * dt);
        float temp_beta3 = (1.0f - r) * (1.0f - r) * (1.0f - r) / (dt * dt);


        //ESO 估计误差
	    float error_eso = z1 - _measure;
	    z1 = z1 + dt * z2 - temp_beta1 * error_eso; //
	    z2 = z2 + dt * z3 - temp_beta2 * error_eso +  dt *b0 * last_u;
	    z3 = z3 -temp_beta3 * error_eso;
	    //限制z2,z3不能太大
	    z2=constrain_float(z2, -b0*output_max, b0*output_max);
	    z3=constrain_float(z3, -b0*output_max, b0*output_max);
        //计算控制误差
	    float error_control = _target - z1;
	    float u0= temp_kp * error_control - temp_kd * z2;
	    // ================================================================
	    // 第四步：扰动补偿
	    //   u = (u0 - z3) / b0
	    // ================================================================
	    float u_temp = (u0 - z3) / b0;
	    float u = constrain_float(u_temp, -output_max, output_max);
	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）

	    if(aix==0)
	    {
	    	log_loop_delay1++;
	        if(log_loop_delay1>=10)
	        {
	        	log_loop_delay1=0;
	            AP::logger().Write("RCRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
	        		   (float)temp_beta1,   //t5
	        		   (float)temp_beta2,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * error_control  ,      //t10
	        		   (float) -(temp_kd * z2),      //t11
	        		   (float)u0,      //t12
	        		   (float)u,      //t13
	        		   (float)(temp_beta3),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }

	    if(aix==1)
	    {

	    	log_loop_delay2++;
	        if(log_loop_delay2>=10)
	        {
	        	log_loop_delay2=0;
	            AP::logger().Write("PCRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
					   (float)temp_beta1,   //t5
					   (float)temp_beta2,   //t6
					   (float)z1,      //t7
					   (float)z2,      //t8
					   (float)z3,      //t9
					   (float)temp_kp * error_control  ,      //t10
					   (float) -(temp_kd * z2),      //t11
					   (float)u0,      //t12
					   (float)u,      //t13
					   (float)(temp_beta3),      //t14
					   (float)(u_temp)     //t15

	        		   );
	        }
	    }
	    if(aix==2)
	    {
	    	log_loop_delay3++;
	        if(log_loop_delay3>=10)
	        {
	        	log_loop_delay3=0;
	            AP::logger().Write("YCRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
					   (float)temp_beta1,   //t5
					   (float)temp_beta2,   //t6
					   (float)z1,      //t7
					   (float)z2,      //t8
					   (float)z3,      //t9
					   (float)temp_kp * error_control  ,      //t10
					   (float) -(temp_kd * z2),      //t11
					   (float)u0,      //t12
					   (float)u,      //t13
					   (float)(temp_beta3),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }

	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）
	    return u;
	}
	else
	{
	    //伪代码示例，update_axis开头
	    if (!is_valid_data(z1) || !is_valid_data(z2) || !is_valid_data(z3)
	    	|| !is_valid_data(v1) || !is_valid_data(v2))
	    {
	    	// 重置该轴状态，避免永久锁死
	    	v1 = target;
	    	v2 = 0;
	    	z1 = measure;
	    	z2 = 0;
	    	z3 = 0;
	    	last_u = 0;
	    	_target=target;
	    	_last_target=target;
	    	_measure=measure;
	    	_last_measure=measure;

	        return 0.0f;
	    }

	    const float b0   = MAX(_b0.get(),0.1);      // 系统增益估计
	    const float r    = MAX(_td_r.get(),1.0f);    // TD 速度因子
	    const float h0 = MAX(_td_h0.get(), dt); // TD 滤波因子,TD 滤波因子不小于采样周期
	    const float nlsef_a1   = MAX(_nlsef_alpha1.get(),0.001f);  // NLSEF 位置误差非线性指数
	    const float nlsef_a2   = MAX(_nlsef_alpha2.get(),0.001f);  // NLSEF 速度误差非线性指数
	    const float nlsef_delt  = MAX(_nlsef_delta.get(),0.001f);   // fal 线性段阈值
	    const float eso_delt  = MAX(_nlsef_delta.get(),0.001f);   // fal 线性段阈值
	    const float output_max  = MAX(_limit_u_max.get(),1.0f);   //输出限制

        float temp_kp=MAX(_nlsef_kp.get(),0.0f);
        float temp_kd=MAX(_nlsef_kd.get(),0.0f);
        float temp_beta1=MAX(_eso_beta1.get(),0.0f);
        float temp_beta2=MAX(_eso_beta2.get(),0.0f);
        float temp_beta3=MAX(_eso_beta3.get(),0.0f);

	    // 带宽法实时计算增益


	    // ================================================================
	    // 第一步：跟踪微分器 TD
	    //   fh = fhan(v1 - target, v2, r, h0)
	    //   v1 = v1 + dt * v2
	    //   v2 = v2 + dt * fh
	    // ================================================================
	    float fh = fhan(v1 - _target, v2, r, h0);
	    v1 = v1 + dt * v2;
	    v2 = v2 + dt * fh;
	    //估算下真实的角加速度
        float target_dot=(_target-_last_target)/dt;
        //更新模板值
        _last_target=_target;

	    // ================================================================
	    // 第二步：扩张状态观测器 ESO（三阶，前向欧拉离散化）
	    //   e = z1 - y
	    //   z1 = z1 + dt * (z2 - beta01 * e)
	    //   z2 = z2 + dt * (z3 - beta02 * fal(e, 0.5, delta) + b0 * u_prev)
	    //   z3 = z3 + dt * (-beta03 * fal(e, 0.25, delta))
	    // 使用上一周期的控制量 u_prev，避免代数环
	    // ================================================================
	    float e_eso = z1 - _measure;
	    float measure_dot=(_measure-_last_measure)/dt;
	    _last_measure=_measure;

	    float fal_e1 = fal(e_eso, 0.5f, eso_delt);
	    float fal_e2 = fal(e_eso, 0.25f, eso_delt);


	    z1 = z1 + _eso_h_gain*dt * (z2 - temp_beta1 * e_eso);
	    z2 = z2 + _eso_h_gain*dt * (z3 - temp_beta2 * fal_e1 + b0 * last_u);
	    z3 = z3 + _eso_h_gain*dt * (-temp_beta3 * fal_e2);

	    // ================================================================
	    // 第三步：非线性状态误差反馈 NLSEF
	    //   e1 = v1 - z1
	    //   e2 = v2 - z2
	    //   u0 = k1 * fal(e1, alpha1, eso_delt) + k2 * fal(e2, alpha2, eso_delt)
	    // 带宽法：k1 = wc^2, k2 = 2*wc
	    // ================================================================
	    float e1_nlsef = v1 - z1;
	    float e2_nlsef = v2 - z2;
	    float u0= temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt) + temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt);

	    // ================================================================
	    // 第四步：扰动补偿
	    //   u = (u0 - z3) / b0
	    // ================================================================

	    float u_temp = (u0 - z3) / b0;

	    float u = constrain_float(u_temp, -output_max, output_max);

	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）
	    if(aix==0)
	    {
	    	log_loop_delay1++;
	        if(log_loop_delay1>=10)
	        {
	        	log_loop_delay1=0;
	            AP::logger().Write("RDRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
	        		   (float)v1,     //t5
	        		   (float)v2,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt) ,      //t10
	        		   (float)temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt),      //t11
	        		   (float)u0,      //t12
	        		   (float)measure_dot,      //t13
	        		   (float)(target_dot),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }

	    if(aix==1)
	    {

	    	log_loop_delay2++;
	        if(log_loop_delay2>=10)
	        {
	        	log_loop_delay2=0;
	            AP::logger().Write("PDRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
					   (float)v1,     //t5
					   (float)v2,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt),      //t10
	        		   (float)temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt),      //t11
	        		   (float)u0,      //t12
	        		   (float)measure_dot,      //t13
	        		   (float)(target_dot),      //t14
					   (float)(u_temp)     //t15

	        		   );
	        }
	    }
	    if(aix==2)
	    {

	    	log_loop_delay3++;
	        if(log_loop_delay3>=10)
	        {
	        	log_loop_delay3=0;
	            AP::logger().Write("YDRC",
	            		"TimeUS,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15",
	        			"s---------------",
	        			"F---------------",
	        			"Qfffffffffffffff",
	        		    AP_HAL::micros64(),
					   (float)target, //t1
					   (float)_target, //t2
					   (float)measure,     //t3
					   (float)_measure,   //t4
					   (float)v1,     //t5
					   (float)v2,   //t6
	        		   (float)z1,      //t7
	        		   (float)z2,      //t8
	        		   (float)z3,      //t9
	        		   (float)temp_kp * fal(e1_nlsef, nlsef_a1, nlsef_delt),      //t10
	        		   (float)temp_kd * fal(e2_nlsef, nlsef_a2, nlsef_delt),      //t11
	        		   (float)u0,      //t12
	        		   (float)measure_dot,      //t13
	        		   (float)(target_dot),      //t14
					   (float)(u_temp)     //t15
	        		   );
	        }
	    }
	    // 保存本周期输出供下一周期 ESO 使用（调用方负责合限幅后写回）
	    return u;
	}

}


/********************************************************************************************************
*函数原型：update_all
*函数功能：
*修改日期：2026-8-14
*修改作者：
*备注信息：一维更新
**********************************************************************************************************/
float AC_ADRC::update_all(const float& target,const float& measure,float dt,bool limit,uint8_t aix)
{

	// 输入合法性检查
    if ( !is_positive(dt) //时间合理
    	|| !is_valid_data(target)
		||!is_valid_data(measure))
    {
        return 0.0f;
    }

    _error=target-measure;

    if (dt < ADRC_ATT_DT_MIN)
    {
        // 高频时跳过本次更新或累积
        return _last_u_out;
    }
    dt = MIN(dt, ADRC_ATT_DT_MAX); // 只限制上限

    // X 轴（北）—— u_prev 用上一周期限幅后的输出，避免代数环
    float u = update_axis(target, measure, dt,_v1, _v2, _z1, _z2, _z3, _last_u_out,limit,aix);

    //更新历史值
    _last_u_out=u;
    // 保存限幅后的输出供下一周期 ESO 使用
    return u;
}


/********************************************************************************************************
*函数原型：bool AC_ADRC::is_valid_data(float v)
*函数功能：
*修改日期：2026-8-14
*修改作者：
*备注信息：数据是否有效
**********************************************************************************************************/
bool AC_ADRC::is_valid_data(float v)
{
	if(isfinite(v) && !isnan(v))
	{
		return true;
	}
	return false;
}


/********************************************************************************************************
*函数原型：void AC_ADRC::reset_filter(float target,float measure)
*函数功能：
*修改日期：2026-8-14
*修改作者：
*备注信息：数据复位
**********************************************************************************************************/
void AC_ADRC::reset_filter(float target,float measure)
{
    _v1 = target;
    _v2 = 0.0f;
    _z1 = measure;
    _z2 = 0.0f;
    _z3=0.0f;
    _last_u_out=0.0f;
	_target=target;
	_last_target=target;
	_measure=measure;
	_last_measure=measure;
}
/********************************************************************************************************
*函数原型：float AC_ADRC_ATT::fal(float e, float alpha, float delta)
*函数功能：
*修改日期：2026-8-14
*修改作者：
*备注信息： fal 非线性函数
**********************************************************************************************************/
float AC_ADRC::fal(float e, float alpha, float delta)
{
    float abs_e = fabsf(e);
    if (abs_e > delta) {
        // 非线性段：|e|^alpha * sign(e)
        return powf(abs_e, alpha) * (e > 0.0f ? 1.0f : -1.0f);
    } else {
        // 线性段：e / delta^(1-alpha)，保证在 delta 处连续
        return e / powf(delta, 1.0f - alpha);
    }
}



/********************************************************************************************************
*函数原型：float AC_ADRC_PSC_2D::fhan(float x1, float x2, float r, float h)
*函数功能：
*修改日期：2026-8-14
*修改作者：
*备注信息：fhan 最速控制综合函数（离散 TD 用）
**********************************************************************************************************/
float AC_ADRC::fhan(float x1, float x2, float r, float h)
{
    float d  = r * h * h;
    float a0 = h * x2;
    float y  = x1 + a0;
    float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));

    float a;
    if (fabsf(y) > d)
    {
        // 大误差：饱和区域
        a = a0 + (y > 0.0f ? 1.0f : -1.0f) * (a1 - d) * 0.5f;
    } else
    {
        // 小误差：线性区域
        a = a0 + y;
    }

    // 计算最终输出
    if (fabsf(a) > d)
    {
    	return -r * (a > 0.0f ? 1.0f : -1.0f);
    } else
    {
    	return -r * a / d;
    }
}

/********************************************************************************************************
*函数原型：float AC_ADRC_ATT::get_filt_alpha(float dt) const
*函数功能：
*修改日期：2026-8-14
*修改作者：
*备注信息：
**********************************************************************************************************/
float AC_ADRC::get_filt_alpha(float dt)
{
    return calc_lowpass_alpha_dt(dt, _filt_hz);
}

/****************************************************************************************************************************************************
*     file -end
*****************************************************************************************************************************************************/





