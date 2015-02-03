/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
/*功能:对自稳功能进行初始化 将目位置中的高度置0 并返回真
*/
static bool stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // stabilize should never be made to fail
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
/*功能:当遥控器输入的油门小于等于或是还未解锁时重新设yaw的目标角速度,roll.pitch的目标角速.并将电机转速置0。
否则将遥控器操纵杆的控制量对应到目标角度target_roll, target_pitch,航向的目标角速度target_yaw_rate.并与默认的roll,pitch目标角速度（载体坐标系）
进行计算得出最新的目标角速度_rate_bf_target，这个值将通过rate_controller_run()计算输出到电机的Pwm信号
*/
static void stabilize_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    //如果处于锁定状态或是遥控器的油门通道输出值小于等于0
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();//用当前陀螺静止时的角速度作为载体目标角速度(载体坐标系)
        attitude_control.set_yaw_target_to_current_heading();/用当前的航向角作为航向的目标角度(载体坐标系)
        attitude_control.set_throttle_out(0, false);//将输出的比例转化为PWM信号。该函数第一个参数是输入油门量.
        //第二个是是否使用角度增益（更准确的说应该是升力补偿。因为飞行器倾斜垂直向上的升力就会减少，造成飞行高度下降倾斜角度越大这个现象越明显）
        //
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    /*功能:将遥控器CH1，CH2相对于飞行器操纵控制飞行器横滚和仰俯角转化为相对于导航坐标即X轴指北y轴指东，也就是我控制CH1
    就是控制飞行器向北方向飞行（这是无论飞行器机头指向哪），CH2就控制飞行器向东方向飞行（这是无论飞行器机头指向哪）.
    默状态下是没有打开这个功能
    */
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    //将遥控器操纵杆的控制量对应到目标角度target_roll, target_pitch
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    //将遥控器操纵杆的控制量对应到航向的目标角速度
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    
    // get pilot's desired throttle
    //将遥控器操纵杆输入的油门控制量值(0-1000)根据设定的中点划分成两部分 两部分分别进行比例转化对应到油门值(0-1000)
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // call attitude controller
    //将遥控器操纵杆对应的角度、角速度(target_roll, target_pitch, target_yaw_rate)->目标角速度(载体坐标) _rate_bf_target
    /该函数具体功能将在定义他的文件AC_AttitudeControl.cpp中详细说明
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    //将油门值转化为用于电机的控制信号pwm同时还要求进行角度补偿（这个在上面说过，这里就不重复）
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
}
