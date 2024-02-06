#include "FireFight.h"
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>    //地面站
// #include "RC_Channel.h"         //加入遥控读取通道
// #include "rover/Rover.h"

#define FRAME_LENGTH 8          //帧长
#define FRAME_HEADER 0x01          //帧头

void FireFight::uart_init()
{

    hal.serial(1)->begin(19200);      //初始化串口程序
    hal.serial(1)->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    hal.serial(1)->set_unbuffered_writes(true);
    hal.scheduler->delay(100);       //等待初始化串口
    write_two(100,0x2003,10,5);
    hal.scheduler->delay(100);       //设置电机加减速时间
    write_two(101,0x2003,10,5);
    hal.scheduler->delay(100);       //等待初始化串口
    write_one(1,2,1000);
    hal.scheduler->delay(100);       //上下电机堵转电流
    write_one(1,3,1000);
    hal.scheduler->delay(100);       //雾柱电机堵转电流
    write_one(1,4,1);
    hal.scheduler->delay(100);       //堵转时间

    
    
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
    uint16_t reg_adress,reg_num;
    // read any available lines from the lidar
    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!hal.serial(1)->read(c)) {
            break;
        }
        // if buffer is empty and this byte is 0x57, add to buffer
        if (linebuf_len == 0) {
            if (c == FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        // buffer is not empty, add byte to buffer
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 5 items try to decode it
            if (linebuf_len == FRAME_LENGTH) {
                // calculate CRC8 (tbd)
                if (addressID == 0x01)   //如果当前是消防炮返回则使用下面函数
                {
                    uint16_t crc = 0;
                    crc = CRC.Funct_CRC16(linebuf,FRAME_LENGTH-2);
                    // if crc matches, extract contents
                    if (crc == ((linebuf[6])|linebuf[7]<<8)) {
                        // calculate distance
                        reg_adress = ((linebuf[2]<<8)|linebuf[3]);   //获取寄存器地址
                        reg_num = ((linebuf[4]<<8)|linebuf[5]);      //获取写入数值
                        switch (reg_adress)
                        {
                        case 0x000C/* constant-expression 上边状态检测*/:
                            /* code */
                            if (reg_num == 1/* condition */)
                            {
                                return 1;
                                /* code */
                            }
                            break;
                        case 0x000D/* constant-expression 下边状态检测*/:
                            /* code */
                            if (reg_num == 1/* condition */)
                            {
                                return 2;
                                /* code */
                            }
                            break;
                        case 0x000E/* constant-expression 左边状态检测*/:
                            /* code */
                            if (reg_num == 1/* condition */)
                            {
                                return 3;
                                /* code */
                            }
                            break;
                        case 0x000F/* constant-expression右边状态检测 */:
                            /* code */
                            if (reg_num == 1/* condition */)
                            {
                                return 4;
                                /* code */
                            }
                            break;
                        case 0x0010:    //柱状态检测
                            if (reg_num == 1/* condition */)
                            {
                                return 5;
                                /* code */
                            }
                            break;
                        case 0x0011:    //雾状态检测
                            if (reg_num == 1/* condition */)
                            {
                                return 6;
                                /* code */
                            }
                            break;
                        default:
                            break;
                        }

                    }
                }

                // clear buffer
                linebuf_len = 0;
            }
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
    static uint8_t up_down = 0;
    static uint8_t left_right = 0;
    static uint8_t wu_zhu = 0;
    uint16_t under_offset = 1550;
    uint16_t low_offset = 1450;
    // uint8_t temp;

    static uint8_t time_cnt_up = 0,time_cnt_left = 0,time_cnt_zhu = 0;

    // gcs().send_text(MAV_SEVERITY_CRITICAL,"rcin(9):%d",hal.rcin->read(9));
    // temp = check_send_one(0x01);   //读取消防炮的返回值
    // if ( temp!= 0)
    // {
    //     switch (temp)
    //     {
    //     case 1/* constant-expression */:
    //         /* code */
    //         up_down = 1;
    //         break;
    //     case 2/* constant-expression */:
    //         /* code */
    //         up_down = 2;
    //         break;
    //     case 3/* constant-expression */:
    //         /* code */
    //         left_right = 1;
    //         break;
    //     case 4/* constant-expression */:
    //         /* code */
    //         left_right = 2;
    //         break;
    //     case 5/* constant-expression */:
    //         /* code */
    //         wu_zhu = 1;
    //         break;
    //     case 6/* constant-expression */:
    //         /* code */
    //         wu_zhu = 2;
    //         break;
    //     default:
    //         break;
    //     }
    // }
    if (time_samp <= DT_ms)  //第一个周期
    {
        if ((hal.rcin->read(2)) > under_offset  && up_down != 1)
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
            }
            // write_two(0x01,0x000C,1,0);
        }
        else if ((hal.rcin->read(2)) < low_offset  && up_down != 2)
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
            }

            // firefight_rover.up_button(0);

            // write_two(0x01,0x000C,0,1);
        }
        else if(up_down != 3 && ((hal.rcin->read(2)) > low_offset) && ((hal.rcin->read(2)) < under_offset))   //重复发送4次
        {
            if (/* condition */time_cnt_up == 0)
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

    else if (time_samp <= 3*DT_ms)  //第二个周期
    {
        if ((hal.rcin->read(3)) < low_offset && left_right != 1)
        {

            if (/* condition */time_cnt_left == 0)
            {
                right_button(0);              /* code */
            }
            time_cnt_left++;
            if (time_cnt_left >= 2 ) //延时一个执行周期
            {
                /* code */
                left_button(1);
                time_cnt_left = 0;
            }
            // write_two(0x01,0x000E,1,0);
        }
        else if((hal.rcin->read(3)) > under_offset && left_right != 2)
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
            }
            // write_two(0x01,0x000E,0,1);
        }
        else if (left_right != 3 && ((hal.rcin->read(3)) > low_offset) && ((hal.rcin->read(3)) < under_offset))   //重复发送4次
        {
            if (/* condition */time_cnt_left == 0)
            {
                left_button(0);    //将柱清零            /* code */
            }
            time_cnt_left++;
            if (time_cnt_left >= 2 ) //延时一个执行周期
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
        if ((hal.rcin->read(4)) > under_offset  && wu_zhu != 1)
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
        else if((hal.rcin->read(4)) < low_offset && wu_zhu != 2)
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
    
        else if(wu_zhu != 3 && ((hal.rcin->read(4)) > low_offset) && ((hal.rcin->read(4)) < under_offset))
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
    time_samp += DT_ms;
    if(time_samp == 6*DT_ms)
    {
        time_samp = 0;
    }

}

