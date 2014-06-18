/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_BARO_BMP085_H__
#define __AP_BARO_BMP085_H__

#define PRESS_FILTER_SIZE 2

#include "AP_Baro.h"
#include <AverageFilter.h>

class AP_Baro_BMP085 : public AP_Baro
{
public:
    AP_Baro_BMP085() {
        _pressure_samples = 1;
    };       // Constructor


    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    void 			accumulate(void);
    float           get_pressure();
    float           get_temperature();

private:
    int32_t         RawPress;
    int32_t         RawTemp;
    float		    _temp_sum;
    float			_press_sum;
    uint8_t			_count;
    float           Temp;
    float           Press;
    // Flymaple has no EOC pin, so use times instead
    uint32_t        _last_press_read_command_time;
    uint32_t        _last_temp_read_command_time;

    
    // State machine
    uint8_t                         BMP085_State;
    // Internal calibration registers
    int16_t                         ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t                        ac4, ac5, ac6;

    AverageFilterInt32_Size4        _temp_filter;

    uint32_t                        _retry_time;

    void                            Command_ReadPress();
    void                            Command_ReadTemp();
    void                            ReadPress();
    void                            ReadTemp();
    void                            Calculate();
};

#endif // __AP_BARO_BMP085_H__
