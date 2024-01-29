#include "FireFightCRC.h"
// #include <AP_HAL/AP_HAL.h>
FireFightCRC::FireFightCRC(/* args */)
{

}

uint16_t FireFightCRC::Funct_CRC16(unsigned char *puchMsg, uint16_t DataLen)
{
    uint16_t i, j, tmp;
    uint16_t crcdata = 0xFFFF;
    for (i = 0; i < DataLen; i++)
    {
        crcdata = (*puchMsg) ^ crcdata;
        puchMsg++;
        for (j = 0; j < 8; j++)
        {
            tmp = crcdata & 0x0001;
            crcdata = crcdata >> 1;
            if (tmp)
            {
                crcdata = crcdata ^ 0xA001;
            }
        }
    }
    return crcdata;
}
