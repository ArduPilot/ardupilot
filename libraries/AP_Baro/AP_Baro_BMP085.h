/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_Baro_Backend.h"

class AP_Baro_BMP085 : public AP_Baro_Backend
{
public:
    AP_Baro_BMP085(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /* AP_Baro public interface: */
    void update();
    void accumulate(void);

private:
    void _cmd_read_pressure();
    void _cmd_read_temp();
    bool _read_pressure();
    void _read_temp();
    void _calculate();
    bool _data_ready();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    uint8_t _instance;
    float _temp_sum;
    float _press_sum;
    uint8_t _count;

    // Boards with no EOC pin: use times instead
    uint32_t _last_press_read_command_time;
    uint32_t _last_temp_read_command_time;

    // State machine
    uint8_t BMP085_State;

    // Internal calibration registers
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

    uint32_t _retry_time;
    int32_t _raw_pressure;
    int32_t _raw_temp;
};
