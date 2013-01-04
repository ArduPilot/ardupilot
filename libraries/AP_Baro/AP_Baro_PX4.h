/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_PX4_H__
#define __AP_BARO_PX4_H__

#include "AP_Baro.h"

class AP_Baro_PX4 : public AP_Baro
{
public:
    AP_Baro_PX4() : AP_Baro() {
        _baro_fd = -1;
    }
    bool init();
    uint8_t read();
    float get_pressure();
    float get_temperature();
    int32_t get_raw_pressure();
    int32_t get_raw_temp();

private:
    float _temperature;
    float _pressure;

    // baro driver handle
    int _baro_fd;
};

#endif //  __AP_BARO_PX4_H__
