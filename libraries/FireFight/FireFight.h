#pragma once
#ifndef _FireFight_H_
#define _FireFight_H_
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

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

    /* data */
public:
    friend class FireFightCRC;
    FireFight(void);
    // ~FireFight(void);
    void uart_init(void);
    bool updata(void);
    void read_one(uint16_t reg_adress, uint16_t reg_num);
    void write_one(uint16_t reg_adress, uint16_t reg_num);
    void write_two(uint16_t start_reg_adress,uint16_t val_1,uint16_t val_2);
    void up_button(uint16_t val);
    void down_button(uint16_t val);
    void left_button(uint16_t val);
    void right_button(uint16_t val);
    void zhu_button(uint16_t val);
    void wu_button(uint16_t val);
    void upanddown_zero();
    void leftandright_zero();
    void zhu_zero();
    void wu_zero();
    uint8_t check_send_one();
};

#endif