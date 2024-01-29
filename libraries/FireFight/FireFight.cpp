#include "FireFight.h"
#include <stdio.h>
#include "FireFightCRC.h"
#include <GCS_MAVLink/GCS.h>    //地面站
// #include "RC_Channel.h"         //加入遥控读取通道
// #include "rover/Rover.h"
FireFightCRC CRC;
#define FRAME_LENGTH 8          //帧长
#define FRAME_HEADER 0x01          //帧头
FireFight::FireFight(/* args */)
{
    ;
}

void FireFight::uart_init()
{
    
    hal.serial(1)->begin(19200);      //初始化串口程序
    hal.scheduler->delay(1000);       //等待初始化串口
    gcs().send_text(MAV_SEVERITY_CRITICAL,  //地面站消息发送
                "uart set ok");
}

void FireFight::read_one(uint16_t reg_adress,uint16_t reg_num)  //只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] =   0x01;  //设备地址为01
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

void FireFight::write_one(uint16_t reg_adress,uint16_t reg_num)  //只需要填写寄存器ID和寄存器个数
{
    uint8_t data_to_send[10];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    data_to_send[cnt++] =   0x01;  //设备地址为01
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

void FireFight::write_two(uint16_t start_reg_adress,uint16_t val_1,uint16_t val_2)//写两个寄存器，用于归零
{
    uint8_t data_to_send[15];
    uint8_t cnt = 0;
    uint16_t crc = 0;
    uint16_t write_num  = 0x0002;
    data_to_send[cnt++] =   0x01;  //设备地址为01
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

uint8_t FireFight::check_send_one()
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
                // clear buffer
                linebuf_len = 0;
            }
        }
    }
    return 0;
}

void FireFight::up_button(uint16_t val)
{
    write_one(0x000C,val);       //发送内存地址12,指令1，按键上功能
}

void FireFight::down_button(uint16_t val)
{
    write_one(0x000D,val);       //发送内存地址13,指令1，按键上功能
}

void FireFight::left_button(uint16_t val)
{
    write_one(0x000E,val);       //发送内存地址14,指令1，按键上功能
}

void FireFight::right_button(uint16_t val)
{
    write_one(0x000F,val);       //发送内存地址15,指令1，按键上功能
}

void FireFight::zhu_button(uint16_t val)
{
    write_one(0x0010,val);       //发送内存地址16,指令1，按键上功能
}

void FireFight::wu_button(uint16_t val)
{
    write_one(0x0011,val);       //发送内存地址17,指令1，按键上功能
}

void FireFight::upanddown_zero()  //上下行程归零
{
    write_two(0x000C,0,0);       //发送内存地址12,指令1，按键上功能
}

void FireFight::leftandright_zero()  //上下行程归零
{
    write_two(0x000E,0,0);       //发送内存地址12,指令1，按键上功能
}

void FireFight::zhu_zero()    //柱归零
{
    write_one(0x0010,0);       //发送内存地址16,指令1，按键上功能
}

void FireFight::wu_zero()      //雾归零
{
    write_one(0x0011,0);       //发送内存地址17,指令1，按键上功能
}




bool FireFight::updata()
{
    ;
    // if channel_throttle->get_control_in() > 1550
    //     down_button(0);
    //     up_button(1);
    // else if channel_throttle->get_control_in() < 1450
    //     up_button(0);
    //     down_button(1);
    // else
    //     upanddown_zero();

    // if channel_yaw->get_control_in() > 1550
    //     left_button(0);
    //     right_button(1);
    // else if channel_yaw->get_control_in() < 1450
    //     right_button(0); 
    //     left_button(1);
    // else
    //     leftandright_zero();
      //uart();
    // uint8_t _uart_data;
    // uint8_t _flag = 0;
    // int16_t i;
    // uint8_t scope = hal.serial(1)->available();     //check if any bytes are waiting
    //  hal.serial(1)->printf("******test*********");
    // write_one(1,1);
        // for(i = 0; i < scope; i++)
        // {
        //     _uart_data = hal.serial(1)->read();     //获取当前数据
        //     switch (_flag)
        //     {
        //     case 0:
        //         if (_uart_data == 0x52)
        //             _flag = 1;
        //         break;
        //     case 1:
        //         if (_uart_data == 0x31)
        //             _flag = 2;
        //         break;
        //     case 2:
        //         gcs().send_text(MAV_SEVERITY_CRITICAL,
        //             "data ok");
        //         _flag = 0;
        //         return 1;
        //         break;
 
        //     default:
        //         _flag = 0;
        //         break;
        //     }
        // }
    return 0;

}

