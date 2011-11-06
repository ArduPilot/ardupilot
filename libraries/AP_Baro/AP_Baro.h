
#ifndef __AP_BARO_H__
#define __AP_BARO_H__

class AP_Baro
{
    public:
    AP_Baro() {}
    virtual void    init() = 0;
    virtual uint8_t update() = 0;
    virtual int32_t get_pressure() = 0;
    virtual float   get_temp() = 0;

};

#include "AP_Baro_MS5611.h"

#endif // __AP_BARO_H__
