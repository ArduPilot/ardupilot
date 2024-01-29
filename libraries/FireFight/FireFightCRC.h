#pragma once
#ifndef _FireFightCRC_H_
#define _FireFightCRC_H_
#include <AP_HAL/AP_HAL.h>
// #include <AP_AHRS/AP_AHRS.h>
// #include <AP_SerialManager/AP_SerialManager.h>
// #include <stdio.h>

class FireFightCRC
{
private:
    /* data */
public:
    FireFightCRC(/* args */);
    uint16_t Funct_CRC16(unsigned char *puchMsg, uint16_t DataLen);
    
};





#endif