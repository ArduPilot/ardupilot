/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO__HIL_H__
#define __AP_BARO__HIL_H__

#include "AP_Baro.h"

class AP_Baro_HIL : public AP_Baro
{
private:
    float Temp;
    float Press;
    float _pressure_sum;
    float _temperature_sum;
    volatile uint8_t _count;

public:
    bool init();
    uint8_t read();
    float get_pressure();
    float get_temperature() const;
    void setHIL(float altitude_msl);
    void setHIL(float pressure, float temperature);
};

#endif //  __AP_BARO__HIL_H__
