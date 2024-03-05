#include "FireFight.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>    //地面站
#include <array>                // 使用标准库中的array代替C风格数组
// #include "RC_Channel.h"         //加入遥控读取通道
// #include "rover/Rover.h"

#define FRAME_LENGTH 9          //帧长
#define MAX_ACTIONS 200       // 最大动作数量100


class Action{
    public:
        int16_t record_Left_Right_pulse, record_Up_Down_pulse;
        uint16_t record_delay;
        Action() : record_Left_Right_pulse(0), record_Up_Down_pulse(0), record_delay(0) {}   //添加默认构建函数
        Action(int16_t Action_record_Left_Right_pulse, int16_t Action_record_Up_Down_pulse, uint16_t Action_record_delay) : record_Left_Right_pulse(Action_record_Left_Right_pulse), record_Up_Down_pulse(Action_record_Up_Down_pulse), record_delay(Action_record_delay) {}
};

// const volatile Action actions[MAX_ACTIONS]; // 动作数组
std::array<Action, MAX_ACTIONS> actions;
volatile uint8_t num_actions = 0;     // 记录的动作数量
volatile uint8_t action_index= 0;     // 执行动作


    void
    FireFight::uart_init()
{

    hal.serial(1)->begin(19200);      //初始化串口程序
    hal.serial(1)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    hal.serial(1)->set_unbuffered_writes(true);
    hal.scheduler->delay(100);       //等待初始化串口
    write_two(100,0x2003,10,5);
    hal.scheduler->delay(100);       //设置电机加减速时间
    write_two(101,0x2003,10,5);
    hal.scheduler->delay(100);       //等待初始化串口
    write_one(0x01, 0x0002, 10);
    hal.scheduler->delay(100);       //上下电机堵转电流
    write_one(0x01, 0x0003, 200);
    hal.scheduler->delay(100);       //雾柱电机堵转电流
    write_one(0x01, 0x0004, 1);
    hal.scheduler->delay(100);       //堵转时间
    // hal.serial(3)->begin(115200);
    
    
    gcs().send_text(MAV_SEVERITY_CRITICAL,  //地面站消息发送
                "uart set ok");
}

void FireFight::read_one(uint8_t address_ID,uint16_t reg_adress,uint16_t reg_num)  //只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] =   address_ID;  //设备地址为01
    data_to_send[cnt++] =   0x03;  //读取的功能码为03
    data_to_send[cnt++] =   BYTE1(reg_adress);
    data_to_send[cnt++] =   BYTE0(reg_adress);
    data_to_send[cnt++] =   BYTE1(reg_num);
    data_to_send[cnt++] =   BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send,cnt);   //官方给的CRC校验
    data_to_send[cnt++] =   BYTE0(crc);
    data_to_send[cnt++] =   BYTE1(crc);
    hal.serial(1)->write(data_to_send,cnt);
}

void FireFight::write_one(uint8_t address_ID,uint16_t reg_adress,uint16_t reg_num)  //只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] =   address_ID;  //设备地址为01
    data_to_send[cnt++] =   0x06;  //写入的功能码为06
    data_to_send[cnt++] =   BYTE1(reg_adress);
    data_to_send[cnt++] =   BYTE0(reg_adress);
    data_to_send[cnt++] =   BYTE1(reg_num);
    data_to_send[cnt++] =   BYTE0(reg_num);
    crc = CRC.Funct_CRC16(data_to_send,cnt);   //官方给的CRC校验
    data_to_send[cnt++] =   BYTE0(crc);
    data_to_send[cnt++] =   BYTE1(crc);
    hal.serial(1)->write(data_to_send,cnt);
}


void FireFight::write_two(uint8_t address_ID,uint16_t start_reg_adress,uint16_t val_1,uint16_t val_2)//写两个寄存器，用于归零
{
    uint8_t data_to_send[15];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    uint16_t write_num  = 0x0002;   //怀疑写入地址错误，未验证，若验证清删除
    data_to_send[cnt++] =   address_ID;  //设备地址为01
    data_to_send[cnt++] =   0x10;  //写入的功能码为0x10
    data_to_send[cnt++] =   BYTE1(start_reg_adress);
    data_to_send[cnt++] =   BYTE0(start_reg_adress);
    data_to_send[cnt++] =   BYTE1(write_num);            //写入数量
    data_to_send[cnt++] =   BYTE0(write_num);
    data_to_send[cnt++] =   0x04;                   //写入字节数
    data_to_send[cnt++] =   BYTE1(val_1);
    data_to_send[cnt++] =   BYTE0(val_1);
    data_to_send[cnt++] =   BYTE1(val_2);
    data_to_send[cnt++] =   BYTE0(val_2);
    crc = CRC.Funct_CRC16(data_to_send,cnt);   //官方给的CRC校验
    data_to_send[cnt++] =   BYTE0(crc);
    data_to_send[cnt++] =   BYTE1(crc);
    hal.serial(1)->write(data_to_send,cnt);
}

uint8_t FireFight::check_send_one(uint8_t addressID)
{
    // uint16_t reg_adress,reg_num;
    uint8_t num = hal.serial(1)->available();   //读取串口有多少个数据
    uint8_t c;
    static uint8_t stat = 0, len_date = 0;
    if (num > 0)
    {
        // hal.console->printf("当前有%d个数据",num);
        for (; num > 0; num--)
        {
            c = hal.serial(1)->read();
            if (linebuf_len == 0) {
                if (c == 1 || c == 100 || c==101 ) //ID正确
                {
                    linebuf[linebuf_len++] = c;
                }
            }
            else if (linebuf_len == 1)
            {
                if(c == 3)       //确定是读出的指令
                {
                    linebuf[linebuf_len++] = c;
                }

            }
            else if (linebuf_len == 2)
            {
                if (c == 4)     //读出4个字符
                {
                    linebuf[linebuf_len++] = c;
                    stat = 1;  //开始接受剩下的数据
                    len_date = 6;
                }
            }
            else if (stat == 1 && len_date > 0)
            {
                len_date--;
                linebuf[linebuf_len++] = c;
                if (len_date == 0)
                {
                    stat = 2;
                }
            }
            else if (stat == 2)
            {
                if (linebuf_len == FRAME_LENGTH)
                {
                    uint16_t crc = CRC.Funct_CRC16(linebuf, FRAME_LENGTH - 2);
                    if (crc == ((linebuf[FRAME_LENGTH-2]) | linebuf[FRAME_LENGTH - 1] << 8))
                    {
                        if (linebuf[0] == 1)  //如果是消防炮消息
                        {
                            Up_Down_pulse = ((linebuf[3] << 8) | linebuf[4]) / 10;
                            Left_Right_pulse = ((linebuf[5] << 8) | linebuf[6]) / 10;
                            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Left_Right_pulse:%d", Left_Right_pulse);
                            /* code */
                        }
                        else if(linebuf[0] == 100)  //读取左边电机转速设定值和当前转速
                        {
                            Set_Left_motor = ((linebuf[3] << 8) | linebuf[4]);
                            Read_Left_motor = ((linebuf[5] << 8) | linebuf[6]);
                            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Set_Left_motor:%d", (int16_t)Set_Left_motor);
                            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Read_Left_motor:%d", (int16_t)Read_Left_motor);
                        }
                        else if (linebuf[0] == 101) // 读取右边电机转速设定值和当前转速
                        {
                            Set_Right_motor = ((linebuf[3] << 8) | linebuf[4]);
                            Read_Right_motor = ((linebuf[5] << 8) | linebuf[6]);
                            // gcs().send_text(MAV_SEVERITY_CRITICAL, "Read_Right_motor:%d", (int16_t)Read_Right_motor);
                        }

                        // gcs().send_text(MAV_SEVERITY_CRITICAL, "上下的脉冲值为:%d", Up_Down_pulse);

                    }
                    linebuf_len = 0;
                    stat = 0;
                }
            }
            else
            {
                linebuf_len = 0;
                stat = 0;
            }
            // hal.serial(1)->write(c);
        }

    }
 
        return 0;
}

void FireFight::up_button(uint16_t val)
{
    write_one(0x01,0x000C,val);       //发送内存地址12,指令1，按键上功能
}

void FireFight::down_button(uint16_t val)
{
    write_one(0x01,0x000D,val);       //发送内存地址13,指令1，按键上功能
}

void FireFight::left_button(uint16_t val)
{
    write_one(0x01,0x000E,val);       //发送内存地址14,指令1，按键上功能
}

void FireFight::right_button(uint16_t val)
{
    write_one(0x01,0x000F,val);       //发送内存地址15,指令1，按键上功能
}

void FireFight::zhu_button(uint16_t val)
{
    write_one(0x01,0x0010,val);       //发送内存地址16,指令1，按键上功能
}

void FireFight::wu_button(uint16_t val)
{
    write_one(0x01,0x0011,val);       //发送内存地址17,指令1，按键上功能
}

void FireFight::upanddown_zero()  //上下行程归零
{
    write_two(0x01,0x000C,0,0);       //发送内存地址12,指令1，按键上功能
}

void FireFight::leftandright_zero()  //左右行程归零
{
    write_two(0x01,0x000E,0,0);       //发送内存地址12,指令1，按键上功能
}

void FireFight::wuzhu_zero()
{
    write_two(0x01,0x0010,0,0);       //发送内存地址12,指令1，按键上功能
}

void FireFight::zhu_zero()    //柱归零
{
    write_one(0x01,0x0010,0);       //发送内存地址16,指令1，按键上功能
}

void FireFight::wu_zero()      //雾归零
{
    write_one(0x01,0x0011,0);       //发送内存地址17,指令1，按键上功能
}

void FireFight::valve_button(uint16_t val)//具体定义请参考.h文件

{
    write_one(0x01,0x0013,val);
}

void FireFight::pump_button(uint16_t val)
{
    write_one(0x01,0x0014,val);
}

void FireFight::Record_button(uint16_t val)
{
    write_one(0x01,0x0015,val);
}

void FireFight::playback_button(uint16_t val)
{
    write_one(0x01,0x0016,val);
}

void FireFight::function_fire_fight(uint8_t DT_ms)   //执行周期，传入DT很重要
{
    // RC_Channel &_rc = rc();
    // uint64_t start = AP_HAL::micros64();
    static uint16_t time_samp = 0;            //每个执行周期只能发送一条信息
    static int8_t up_down = 0, up_down_last = 88;
    static int8_t left_right = 0, left_right_last = 88;
    static int16_t aim_Left_Right_pulse = 0, aim_Up_Down_pulse = 0;
    static uint16_t record_delay = 0,aim_delay = 0,current_delay = 0;
    static uint8_t replay_flag = 0;        //当为1时候表示正在执行回放
    static uint16_t record_T = 0;           //每100个周期强制记录一次当前位置
    // static uint8_t stalled_cnt_flag_LR = 0, stalled_cnt_flag_UD = 0;
    // static uint16_t Left_Right_pulse_Last = 8888, Up_Down_pulse_Last = 8888;
    // static int8_t stalled_protect_LR = 0, stalled_protect_UD = 0;
    // static uint8_t wu_zhu = 0;
    // static uint8_t record_move = 0;
    uint16_t under_offset = 1550;
    uint16_t low_offset = 1450;
    // uint16_t temp = 0;
    // uint8_t add_offset = 30; // 遥控器增加数值
    int16_t exp_offset_Up_Down = 0, exp_offset_Left_Right = 0;
    int16_t dead_offset_motor = 30; 
    // uint8_t temp;

    static uint8_t time_cnt_up = 0,time_cnt_left = 0,time_cnt_zhu = 0;//,time_cnt_record = 0;

    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin(9):%d",hal.rcin->read(9));

    exp_offset_Up_Down = -(aim_Up_Down_pulse - Up_Down_pulse); // 计算与目标偏差值
    exp_offset_Left_Right = aim_Left_Right_pulse - Left_Right_pulse;
    if (exp_offset_Up_Down > 3276) // 表示走原路了
        exp_offset_Up_Down -= 6553;
    else if (exp_offset_Up_Down < -3276)
        exp_offset_Up_Down += 6553;

    if (exp_offset_Left_Right > 3276) // 表示走原路了
        exp_offset_Left_Right -= 6553;
    else if (exp_offset_Left_Right < -3276)
        exp_offset_Left_Right += 6553;


    if ((hal.rcin->read(2)) > under_offset)
    {
        // aim_Up_Down_pulse += add_offset;
        exp_offset_Up_Down = 6666;
    }
    else if ((hal.rcin->read(2)) < low_offset)
    {
        exp_offset_Up_Down = -6666;
        // aim_Up_Down_pulse -= add_offset;
    }
    else if (replay_flag!=1)
    {
        exp_offset_Up_Down = 0;  //期望值给0
            // aim_Up_Down_pulse = Up_Down_pulse;
    }

    if ((hal.rcin->read(3)) > under_offset)
    {
        // aim_Left_Right_pulse += add_offset;
        exp_offset_Left_Right = 6666;
    }
    else if ((hal.rcin->read(3)) < low_offset)
    {
        // aim_Left_Right_pulse -= add_offset;
        exp_offset_Left_Right = -6666;
    }
    else if(replay_flag != 1)
    {
        exp_offset_Left_Right = 0; // 期望值给0
        // aim_Left_Right_pulse = Left_Right_pulse;
    }

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "exp_offset_Up_Down:%d", exp_offset_Up_Down);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "exp_offset_Left_Right:%d", exp_offset_Left_Right);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "上下的脉冲值为:%d", Up_Down_pulse);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "左右的脉冲值为:%d", Left_Right_pulse);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "左右的期望值为:%d", aim_Left_Right_pulse);
    if (time_samp <= DT_ms)  //第一个周期
    {
        if (exp_offset_Up_Down > dead_offset_motor) // 当计算期望值为正数时候，启动按键上按钮
        {
            /*写入两个地址的格式 从机ID 开始写入地址 写入参数1 写入参数2*/
            /*第一个参数是上，第二个是下*/

            if (/* condition */time_cnt_up == 0)
            {
                down_button(0);                /* code */

            }
            time_cnt_up++;
            if (time_cnt_up >= 2 ) //延时一个执行周期
            {
                /* code */
                up_button(1);
                time_cnt_up = 0;
                up_down = 1;   //表示当前正在向上
                // stalled_protect_UD = 0;
            }
            // write_two(0x01,0x000C,1,0);
        }
        if (exp_offset_Up_Down < -dead_offset_motor) // 当计算期望值为负数时候，启动按键下按钮
        {
            if (/* condition */time_cnt_up == 0)
            {
                up_button(0);                /* code */
            }

            time_cnt_up++;
            if (time_cnt_up >= 2 ) //延时一个执行周期
            {
                /* code */
                down_button(1);
                time_cnt_up = 0;
                up_down = -1;  //表示正在向下
                // stalled_protect_UD = 0;
            }

            // firefight_rover.up_button(0);

            // write_two(0x01,0x000C,0,1);
        }
        else if (abs(exp_offset_Up_Down) < dead_offset_motor) // 重复发送4次
        {

            up_down = 0;

            if (/* condition */ time_cnt_up == 0)
            {
               up_button(0);     //将柱清零            /* code */
            }
            time_cnt_up++;
            if (time_cnt_up >= 2 ) //延时一个执行周期
            {
                /* code */
                down_button(0);
                time_cnt_up = 0;
                // up_down++;
            }
            // write_two(0x01,0x000C,0,0);

        }
    }

    else if (time_samp <= 3*DT_ms )  //第二个周期
    {
        if (exp_offset_Left_Right < -dead_offset_motor)
        {

            if (/* condition */ time_cnt_left == 0)
            {
                    right_button(0); /* code */
            }
            time_cnt_left++;
            if (time_cnt_left >= 2) // 延时一个执行周期
            {
                /* code */
                left_button(1);
                time_cnt_left = 0;
                left_right = -1;
                // stalled_protect_LR = 0;
            }
            // write_two(0x01,0x000E,1,0);
        }
        else if (exp_offset_Left_Right > dead_offset_motor)
        {
            if (/* condition */time_cnt_left == 0)
            {
                left_button(0);               /* code */
            }

            time_cnt_left++;
            if (time_cnt_left >= 2 ) //延时一个执行周期
            {
                /* code */
                right_button(1);
                time_cnt_left = 0;
                left_right = 1;
                // stalled_protect_LR = 0;
            }
            // write_two(0x01,0x000E,0,1);
        }
        else if (abs(exp_offset_Left_Right) < dead_offset_motor) // 重复发送4次
        {
            left_right = 0;
            if (/* condition */ time_cnt_left == 0)
            {
                left_button(0); // 将柱清零            /* code */
            }
            time_cnt_left++;
            if (time_cnt_left >= 2) // 延时一个执行周期
            {
                /* code */
                right_button(0);
                time_cnt_left = 0;
                // left_right++;
            }
            // leftandright_zero();
            // write_two(0x01,0x000E,0,0);
        }
    }

    else if (time_samp <= 5*DT_ms)  //第三个周期
    {
        if ((hal.rcin->read(4)) > under_offset)
        {
            if (/* condition */time_cnt_zhu == 0)
            {
                wu_button(0);    //将雾清零             /* code */
            }
            time_cnt_zhu++;
            if (time_cnt_zhu >= 2 ) //延时一个执行周期
            {
                /* code */
                zhu_button(1);
                time_cnt_zhu = 0;
            }
            // write_two(0x01,0x0010,1,0);
        }
        else if((hal.rcin->read(4)) < low_offset)
        {
            if (/* condition */time_cnt_zhu == 0)
            {
                zhu_button(0);  //将柱清零            /* code */
            }
            time_cnt_zhu++;
            if (time_cnt_zhu >= 2 ) //延时一个执行周期
            {
                /* code */
                wu_button(1);
                time_cnt_zhu = 0;

            }
            // write_two(0x01,0x0010,0,1);
        }
    
        else if(((hal.rcin->read(4)) > low_offset) && ((hal.rcin->read(4)) < under_offset))
        {
            if (/* condition */time_cnt_zhu == 0)
            {
                zhu_button(0);  //将柱清零            /* code */
            }
            time_cnt_zhu++;
            if (time_cnt_zhu >= 2 ) //延时一个执行周期
            {

                /* code */
                
                wu_button(0);
                time_cnt_zhu = 0;
                // wu_zhu++;
            }
        // write_two(0x01,0x0010,0,0);

        }
    }
    // if (time_samp <= 5 * DT_ms)
    // {
    //     if (left_right != 0)
    //     {
    //         temp = abs(Left_Right_pulse_Last - Left_Right_pulse);
    //         if(temp > 5000)
    //         {
    //             temp = 6553 - temp;
    //         }

    //         if (temp < 2)
    //         {
    //             stalled_cnt_flag_LR++;   
    //         }
    //         else
    //         {
    //             stalled_cnt_flag_LR = 0;
    //         }
    //         if (stalled_cnt_flag_LR >= 10)
    //         {
    //             aim_Left_Right_pulse = Left_Right_pulse;
    //             if (left_right == 1)
    //             {
    //                 stalled_protect_LR = 1;
    //                 gcs().send_text(MAV_SEVERITY_CRITICAL, "左右差值值为:%d", temp);
                    
    //             }

    //             else if (left_right == -1)
    //             {
    //                 stalled_protect_LR = -1;
    //             }
    //             stalled_cnt_flag_LR = 0;
    //             gcs().send_text(MAV_SEVERITY_CRITICAL, "左右保护值为:%d", stalled_protect_LR);
    //         }

    //         Left_Right_pulse_Last = Left_Right_pulse;
    //     }

    //     if (up_down != 0)
    //     {
    //         temp = abs(Up_Down_pulse_Last - Up_Down_pulse);
    //         if (temp > 5000)
    //         {
    //             temp = 6553 - temp;
    //         }

    //         if (temp < 2)
    //         {
    //             stalled_cnt_flag_UD++;
    //         }
    //         else
    //         {
    //             stalled_cnt_flag_UD = 0;
    //         }
    //         if (stalled_cnt_flag_UD >= 10)
    //         {
    //             aim_Up_Down_pulse = Up_Down_pulse;
    //             if (up_down == 1)
    //                 stalled_protect_UD = 1;
    //             else if (up_down == -1)
    //                 stalled_protect_UD = -1;
    //         }
    //         stalled_cnt_flag_UD = 0;
    //         Up_Down_pulse_Last = Up_Down_pulse;
    //         // gcs().send_text(MAV_SEVERITY_CRITICAL, "上下差值值为:%d", abs(Up_Down_pulse_Last - Up_Down_pulse));
    //     }
    // }


    if ((hal.rcin->read(5)) > under_offset)   //表示正在录制动作
    {
        record_delay++;                                                                         // 记录时间
        if ((left_right != 0 || up_down != 0) && (up_down_last == 88 && left_right_last == 88)) // 有动作时开始记录
        {
            record_delay = 0;
            current_delay = 0;
            aim_delay = 0;
            up_down_last = up_down;
            left_right_last = left_right;
            num_actions = 0;
            record_T = 0;
            // record_Left_Right_pulse = Left_Right_pulse;  //记录初始数值
            // record_Up_Down_pulse =  Up_Down_pulse;
            actions[num_actions++] = Action(Left_Right_pulse, Up_Down_pulse, record_delay);
        }

        if (num_actions < MAX_ACTIONS)   //若指令满了，则停止记录
        {
            if ((up_down_last != 88 && left_right_last != 88)) // 过了初始化才能进入记录
            {
                if (up_down != up_down_last || left_right != left_right_last ||record_T*DT_ms*5 > 2000) // 当动作发生改变时，记录当前电机脉冲数值
                {
                    actions[num_actions++] = Action(Left_Right_pulse, Up_Down_pulse, record_delay);
                    up_down_last = up_down;
                    left_right_last = left_right;
                   record_T = 0;
                }
               record_T++;
            }

        }

    }
    else if((hal.rcin->read(5)) < low_offset)
    {
        replay_flag = 1;
        if (action_index == 0)  //当执行第一次动作时，需要进行归位
        {
            aim_Left_Right_pulse = actions[action_index].record_Left_Right_pulse;
            aim_Up_Down_pulse = actions[action_index].record_Up_Down_pulse;
            aim_delay = actions[action_index].record_delay;
            action_index++;
            current_delay = 0;
        }
        else if ((abs(exp_offset_Left_Right) < 33 && abs(exp_offset_Up_Down) < 33))
        {
            if ((current_delay >= aim_delay))
            {
                if (action_index < num_actions)
                {
                    aim_Left_Right_pulse = actions[action_index].record_Left_Right_pulse;
                    aim_Up_Down_pulse = actions[action_index].record_Up_Down_pulse;
                    aim_delay = actions[action_index].record_delay;
                    action_index++;
                }
                else
                {
                    action_index = 0; // 重复动作
                    current_delay = 0;
                    aim_delay = 0;
                }
            }
        }
        current_delay++;
    }

    else if(((hal.rcin->read(5)) > low_offset) && ((hal.rcin->read(5)) < under_offset))
    {
        if ((up_down_last != 88 && left_right_last != 88) || replay_flag)
        {
            up_down_last = 88;
            left_right_last = 88;
            aim_Left_Right_pulse = Left_Right_pulse;
            aim_Up_Down_pulse = Up_Down_pulse;
            replay_flag = 0;
            action_index = 0;
        }

        // write_two(0x01,0x0010,0,0);

    }

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "通道11的数值:%d", hal.rcin->read(10));
    time_samp += DT_ms;
    if(time_samp == 6*DT_ms)
    {
        time_samp = 0;
    }

}

