#include "Fire_motor_485.h"
#include <GCS_MAVLink/GCS.h>    //地面站


// Fire_motor_485::Fire_motor_485(/* args */)
// {
// }

void Fire_motor_485::change_address(uint8_t addressID,uint16_t val) //初始化需要进行设置
{
    // FF.read_one()
    FF.write_one(0x01,0x2008,100);    //设定电机控制器ID为100和101,只有调试运行一次
    // FF.write_one(0x2008,101);    //设定电机控制器ID为100和101,只有调试运行一次
}

void Fire_motor_485::set_Forward(uint8_t addressID)
{
    FF.write_one(addressID,0x2000,0x0001);
}

void Fire_motor_485::set_Reverse(uint8_t addressID)
{
    FF.write_one(addressID,0x2000,0x0002);
}

void Fire_motor_485::shutdown(uint8_t addressID)                     //停机
{
    FF.write_one(addressID,0x2000,0x0005);
}

void Fire_motor_485::free_stop(uint8_t addressID)                   //自由停机(紧急停机) 
{
    FF.write_one(addressID,0x2000,0x0006);
}

void Fire_motor_485::agent_stop(uint8_t addressID)                   //电磁刹车停机
{
    FF.write_one(addressID,0x2000,0x0009);
}

void Fire_motor_485::set_RPM(uint8_t addressID,uint16_t RPM)         //设定电机转速
{
    FF.write_one(addressID,0x2001,RPM);
}

void Fire_motor_485::set_acc_time(uint8_t addressID,uint16_t time)   //设定加速时间单位0.1S
{
    FF.write_one(addressID,0x2003,time);
}

void Fire_motor_485::set_redu_time(uint8_t addressID,uint16_t time)  //设定减速时间单位0.1S
{
    FF.write_one(addressID,0x2004,time);
}

void Fire_motor_485::set_motor_poles(uint8_t addressID,uint16_t num) //设定电机极对数
{
    FF.write_one(addressID,0x2002,num);
}

void Fire_motor_485::read_drive1_status(uint8_t addressID)           //驱动器状态字1
{
    FF.read_one(addressID,0x2100,0x0001);
}

void Fire_motor_485::read_drive2_status(uint8_t addressID)           //驱动器状态字2
{
    FF.read_one(addressID,0x2101,0x0001);
}

void Fire_motor_485::drive_error(uint8_t addressID)                 //故障代码
{
    FF.read_one(addressID,0x2102,0x0001);
}

void Fire_motor_485::update_status()
{
    read_drive1_status(Left_motor);
    read_drive1_status(Right_motor);
}


void Fire_motor_485::function_fire_motor_485()
{
    const float x_x = 6.66666;       //系数为6.666
    int16_t V_L,V_R,x,y;          //设定速度量为（0～100）
    uint16_t under_offset = 1550;   //死区设置
    uint16_t low_offset = 1450;
    uint16_t mid_offset = 1500;
    uint8_t stop_button = 0;
    uint16_t rcin_1 = 3000 - hal.rcin->read(1); 
    uint16_t rcin_0 = hal.rcin->read(0); 
    uint16_t rcin_9 = hal.rcin->read(9); 
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin(9):%d",rcin_9);
    static uint16_t last_rcin_9 = mid_offset;    
    static uint8_t L_ord = 0,R_ord = 0,golab_cnt = 0; //判断朝向，当为0的时候没有朝向，为1的时候向前，2的时候向后    
    static uint8_t L_cnt=0,R_cnt=0;
    // static int16_t last_V_R  = 0;
    // static int16_t last_V_L  = 0;
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"按键标志位:%d",abs(rcin_9 - last_rcin_9));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin_1标志位:%d",abs(rcin_1 - mid_offset));
    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin_0志位:%d",abs(rcin_0 - mid_offset));
   
    if(abs(rcin_9 - last_rcin_9) > 800) //判断是否有案件按下
    {
        stop_button = 1;//相当于按键被按下
    }
    last_rcin_9 = rcin_9; 
    if(rcin_1 > under_offset && rcin_0 > under_offset)  //表示在第一象限
    {
        y = rcin_1 - mid_offset;
        x = rcin_0 - mid_offset;
        V_L = y * x_x;                                    //转换成PWM
        V_R = (y - x/450*y*0.5)*x_x;
    }
    else if(rcin_1 > under_offset && rcin_0 < low_offset)  //表示在第二象限
    {
        y = rcin_1 - mid_offset;
        x = rcin_0 - mid_offset;
        V_L = (y + x/450*y*0.5)*x_x;                               //转换成PWM
        V_R = y * x_x;     
    }
    else if(rcin_1 < low_offset && rcin_0 < low_offset)  //表示在第三象限
    {
        y = rcin_1 - mid_offset;
        x = rcin_0 - mid_offset;
        V_L = (y + x/450*y*0.5)*x_x;                               //转换成PWM
        V_R = y * x_x;     
    }    
    else if(rcin_1 < low_offset && rcin_0 > under_offset)  //第四象限
    {
        y = rcin_1 - mid_offset;
        x = rcin_0 - mid_offset;
        V_L = y * x_x;                                    //转换成PWM
        V_R = (y - x/450*y*0.5)*x_x;
    }
    else if(rcin_1 > under_offset && (abs(rcin_0 - mid_offset) <100))  //直线前进 
    {
        y = rcin_1 - mid_offset;
        V_L = y * x_x;                                    //转换成PWM
        V_R = y * x_x; 
    }
    else if(rcin_1 < low_offset && (abs(rcin_0 - mid_offset) <100))  //直线后退
    {
        y = rcin_1 - mid_offset;
        V_L = y * x_x;                                    //转换成PWM
        V_R = y * x_x; 
    }
    else if(rcin_0 < low_offset && (abs(rcin_1 - mid_offset) <100))  //原地逆时间
    {
        x = rcin_0 - mid_offset;
        V_L = x * x_x;                                    //转换成PWM
        V_R =-x * x_x; 
    }
    else if(rcin_0 > under_offset && (abs(rcin_1 - mid_offset) <100))  //原地顺时间 
    {
        x = rcin_0 - mid_offset;
        V_L = x * x_x;                                    //转换成PWM
        V_R =-x * x_x; 
    }   
    else if (stop_button || ((abs(rcin_1 - mid_offset) <100 && abs(rcin_0 - mid_offset) <100))/* condition */)
    {
        /* code */
        V_L = 0;                                    //转换成PWM
        V_R = 0; 
    }
    V_L = LIMIT(V_L,-3000,3000);   //输出限幅
    V_R = LIMIT(V_R,-3000,3000);
    
    if( golab_cnt == 0)  //如果更新数值没有改变，则见不输出V_L != last_V_L &&
    {
        V_L = -V_L;
        if (V_L > 0)
        {
            if (L_cnt==0/* condition */|| L_ord != 1)  //若这个不等于1,则代表已经转过方向
            {
                // if(L_ord != 1)
                // {
                    set_Forward(Left_motor);                 
                    L_ord = 1;
                    L_cnt = 0;//延时标志位                    
                // }
            }
            L_cnt++;
            if (/* condition */L_cnt==2)
            {
                /* code */
                set_RPM(Left_motor,V_L);
                L_cnt = 0;

            }
        }
        else if(V_L < 0)
        {
            if (L_cnt==0/* condition */|| L_ord != 2)
            {
                set_Reverse(Left_motor);
                L_ord = 2;
                L_cnt = 0;  //延时标志位
            }
            L_cnt++;
            if (/* condition */L_cnt==2)
            {
                /* code */
                // V_L = -V_L;
                set_RPM(Left_motor,-V_L);
                L_cnt = 0;
            }        
        }
        else if(V_L == 0)
        {
            if (L_cnt==0/* condition */|| L_ord != 0)
            {
                free_stop(Left_motor);
                L_ord = 0;
                L_cnt = 0;  //延时标志位
            }
            L_cnt++;
            if (/* condition */L_cnt==2)
            {
                set_RPM(Left_motor,0);
                L_cnt = 0;
            }   
        } 
        // last_V_L = V_L;     

    }
    
    if(golab_cnt == 1)  //如果更新数值没有改变，则见不输出V_R != last_V_R 
    {
        if (V_R > 0)
        {
            if (R_cnt==0/* condition */|| R_ord != 1)
            {
                set_Forward(Right_motor);
                R_ord = 1;
                R_cnt = 0;
            }
            R_cnt++;
            if (/* condition */R_cnt==2)
            {
                /* code */
                set_RPM(Right_motor,V_R);
                R_cnt = 0;
            }
        }
        else if(V_R < 0)
        {
            if (R_cnt==0/* condition */|| R_ord != 2)
            {
                set_Reverse(Right_motor);
                R_ord = 2;

            }
            R_cnt++;
            if (/* condition */R_cnt==2)
            {
                /* code */
                // V_R = -V_R;
                set_RPM(Right_motor,-V_R);
                R_cnt = 0;
            }        
        }
        else if(V_R == 0)
        {
            if (R_cnt==0/* condition */|| R_ord != 0)
            {
                free_stop(Right_motor);
                R_ord = 0;
                R_cnt = 0;
            }
            R_cnt++;
            if (/* condition */R_cnt==2)
            {
                set_RPM(Right_motor,0);
                R_cnt = 0;
            }   
        }
        // last_V_R = V_R;
        golab_cnt = 2;
    }

    if (golab_cnt == 2/* condition */)
    {
        golab_cnt = 0;
        /* code */
    }
    else if (golab_cnt == 0/* condition */)
    {
        golab_cnt++;
        /* code */
    }
    
    

}


