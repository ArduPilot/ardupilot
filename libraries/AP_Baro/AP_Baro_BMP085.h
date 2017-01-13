#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <Filter/Filter.h>

#include "AP_Baro_Backend.h"

class AP_Baro_BMP085 : public AP_Baro_Backend
{
public:
    AP_Baro_BMP085(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /* AP_Baro public interface: */
    void update();

private:
    void _cmd_read_pressure();
    void _cmd_read_temp();
    bool _read_pressure();
    void _read_temp();
    void _calculate();
    bool _data_ready();

    void _timer(void);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_HAL::DigitalSource *_eoc;

    uint8_t _instance;
    bool _has_sample;

    // Boards with no EOC pin: use times instead
    uint32_t _last_press_read_command_time;
    uint32_t _last_temp_read_command_time;

    // State machine
    uint8_t _state;

    // Internal calibration registers
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

    uint32_t _retry_time;
    int32_t _raw_pressure;
    int32_t _raw_temp;
    int32_t _temp;
    AverageIntegralFilter<int32_t, int32_t, 10> _pressure_filter;
};
