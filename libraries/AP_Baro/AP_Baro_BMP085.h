/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_BARO_BMP085_H__
#define __AP_BARO_BMP085_H__

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
    float		    _temp_sum;
    float			_press_sum;
    uint8_t			_count;

    // Flymaple has no EOC pin, so use times instead
    uint32_t        _last_press_read_command_time;
    uint32_t        _last_temp_read_command_time;
    
    // State machine
    uint8_t                         BMP085_State;

    // Internal calibration registers
    int16_t                         ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t                        ac4, ac5, ac6;

    uint32_t                        _retry_time;
    int32_t                         RawPress;
    int32_t                         RawTemp;

    void                            Command_ReadPress();
    void                            Command_ReadTemp();
    bool                            ReadPress();
    void                            ReadTemp();
    void                            Calculate();
};

#endif // __AP_BARO_BMP085_H__
