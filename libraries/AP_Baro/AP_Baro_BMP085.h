/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_BMP085 : public AP_Baro_Backend
{
public:
    // Constructor
    AP_Baro_BMP085(AP_Baro &baro);

    /* AP_Baro public interface: */
    void update();
    void accumulate(void);

private:
    void _cmd_read_pressure();
    void _cmd_read_temp();
    bool _read_pressure();
    void _read_temp();
    void _calculate();

    uint8_t _instance = 0;
    float _temp_sum = 0.f;
    float _press_sum = 0.f;
    uint8_t _count = 0;

    // Boards with no EOC pin: use times instead
    uint32_t _last_press_read_command_time;
    uint32_t _last_temp_read_command_time;

    // State machine
    uint8_t BMP085_State = 0;

    // Internal calibration registers
    int16_t ac1 = 0, ac2 = 0, ac3 = 0, b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
    uint16_t ac4 = 0, ac5 = 0, ac6 = 0;

    uint32_t _retry_time = 0;
    int32_t _raw_pressure;
    int32_t _raw_temp;
};
