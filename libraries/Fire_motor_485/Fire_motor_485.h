#pragma once
#ifndef _FIRE_MOTOR_485_H_
#define _FIRE_MOTOR_485_H_
#include <FireFight/FireFight.h>

#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
/*  本机波特率 19200
    本机485通道ID：0x02*/
class Fire_motor_485
{
private:
    FireFight FF;
    void update_status();        //更新状态数值
    uint16_t Left_motor = 100;   //左电机驱动
    uint16_t Right_motor = 101;  //右电机驱动
    void set_Reverse(uint8_t addressID);                 //设定电机反向
    void set_Forward(uint8_t addressID);                  //设定电机正向
    void shutdown(uint8_t addressID);                     //停机
    void free_stop(uint8_t addressID);                    //自由停机(紧急停机) 
    void agent_stop(uint8_t addressID);                   //电磁刹车停机
    void set_RPM(uint8_t addressID,uint16_t RPM);          //设定电机转速
    /*紧急控制暂时为此，后可以增设读取驱动器1、2状态字发送到地面站实时进行监控*/
    /*下面为初始化设置参数*/
    void change_address(uint8_t addressID,uint16_t val);   //改变485协议地址，在多个设备的时候需要使用～
    void set_acc_time(uint8_t addressID,uint16_t time);   //设定加速时间
    void set_redu_time(uint8_t addressID,uint16_t time);  //设定减速时间
    void set_motor_poles(uint8_t addressID,uint16_t num); //设定电机极对数
    void read_drive1_status(uint8_t addressID);           //驱动器状态字1
    void read_drive2_status(uint8_t addressID);           //驱动器状态字2
    void drive_error(uint8_t addressID);                  //故障驱动代码
    void read_RPM(uint8_t addressID);                     //读取电机输出转速      
        /* data */
    public : /*addressID为控制板的485ID，不是功能地址*/
                /*下面为动态调整参数*/

                void function_fire_motor_485(uint8_t DT_ms);
};







#endif

