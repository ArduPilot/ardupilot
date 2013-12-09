/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO__HIL_H__
#define __AP_BARO__HIL_H__

#include "AP_Baro.h"

class AP_Baro_HIL : public AP_Baro
{
private:
    uint8_t BMP085_State;
    float Temp;
    float Press;
    int32_t _pressure_sum;
    int32_t _temperature_sum;
    volatile uint8_t _count;

public:
    bool init();
    uint8_t read();
    float get_pressure();
    float get_temperature();
    void setHIL(float altitude_msl);
};

#endif //  __AP_BARO__HIL_H__
