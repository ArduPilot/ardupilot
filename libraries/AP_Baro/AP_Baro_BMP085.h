/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_BARO_BMP085_H__
#define __AP_BARO_BMP085_H__

#define PRESS_FILTER_SIZE 2

#include <AP_Baro.h>
#include <AverageFilter.h>

class AP_Baro_BMP085 : public AP_Baro
{
public:
    AP_Baro_BMP085(bool apm2_hardware) :
        _apm2_hardware(apm2_hardware) {
        _pressure_samples = 1;
    };       // Constructor


    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure();
    float           get_temperature();

    int32_t         get_raw_pressure();
    int32_t         get_raw_temp();

private:
    int32_t         RawPress;
    int32_t         RawTemp;
    int16_t         Temp;
    uint32_t        Press;
    bool            _apm2_hardware;


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
