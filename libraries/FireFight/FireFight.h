#pragma once
#ifndef _FireFight_H_
#define _FireFight_H_
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "FireFightCRC.h"
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

extern const AP_HAL::HAL &hal;

class FireFight
{
private:
    uint8_t linebuf[10];
    uint8_t linebuf_len = 0; 
    FireFightCRC CRC;

    // void write_two(uint8_t address_ID,uint16_t start_reg_adress,uint16_t val_1,uint16_t val_2);

    /* data */
public:
    void uart_init(void);
    void read_one(uint8_t address_ID,uint16_t reg_adress, uint16_t reg_num);
    void write_one(uint8_t address_ID,uint16_t reg_adress, uint16_t reg_num);    
    // uint8_t check_send_one(uint16_t val);
    uint8_t check_send_one(uint8_t addressID);
    void function_fire_fight(uint8_t DT_ms);
    void write_two(uint8_t address_ID,uint16_t start_reg_adress,uint16_t val_1,uint16_t val_2);
    void up_button(uint16_t val);
    void down_button(uint16_t val);
    void left_button(uint16_t val);
    void right_button(uint16_t val);
    void zhu_button(uint16_t val);
    void wu_button(uint16_t val);
    void upanddown_zero();
    void leftandright_zero();
    void wuzhu_zero();
    void zhu_zero();
    void wu_zero();
    void valve_button(uint16_t val);   //阀门按钮
    void pump_button(uint16_t val);    //泵按钮
    void Record_button(uint16_t val);  //录制按钮
    void playback_button(uint16_t val);//回放按钮
    int16_t Left_Right_pulse, Up_Down_pulse;
    uint16_t Set_Left_motor, Read_Left_motor;
    uint16_t Set_Right_motor, Read_Right_motor;
};

#endif