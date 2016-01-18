/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_VRBRAIN_H__
#define __AP_BARO_VRBRAIN_H__

#include "AP_Baro.h"

class AP_Baro_VRBRAIN : public AP_Baro
{
public:
    bool init();
    uint8_t read();
    float get_pressure();
    float get_temperature();

private:
    float _temperature;
    float _pressure;
    float _pressure_sum;
    float _temperature_sum;
    uint32_t _sum_count;
    void _accumulate(void);
    void _baro_timer(uint32_t now);
    uint64_t _last_timestamp;
    // baro driver handle
    int _baro_fd;
};

#endif //  __AP_BARO_VRBRAIN_H__
