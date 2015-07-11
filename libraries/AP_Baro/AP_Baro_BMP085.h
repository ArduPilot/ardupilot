/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_BARO_BMP085_H__
#define __AP_BARO_BMP085_H__

#include <AP_HAL.h>

#include "AP_Baro.h"

class AP_Baro_BMP085 : public AP_Baro_Backend
{
public:
    // Constructor
    AP_Baro_BMP085(AP_Baro &baro);

    /* AP_Baro public interface: */
    void update();
    void accumulate(void);

private:
    uint8_t         _instance;
    float           _temp_sum = 0.f;
    float           _press_sum = 0.f;
    uint8_t         _count = 0;
    AP_HAL::DigitalSource *_eoc = nullptr;

    // For boards with no EOC pin, use time instead
    uint32_t        _last_press_read_command_time;
    uint32_t        _last_temp_read_command_time;

    // State machine
    uint8_t         _state;

    // Internal calibration registers
    int16_t         _ac1, _ac2, _ac3, _b1, _b2, _mb, _mc, _md;
    uint16_t        _ac4, _ac5, _ac6;

    int32_t         _raw_press;
    int32_t         _raw_temp;

    void            _command_read_press();
    void            _command_read_temp();
    bool            _read_press();
    void            _read_temp();
    void            _calculate();
    bool            _data_ready();
};

#endif // __AP_BARO_BMP085_H__
