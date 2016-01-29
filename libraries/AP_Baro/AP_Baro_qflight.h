/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Baro_Backend.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include <AP_HAL_Linux/qflight/qflight_buffer.h>

class AP_Baro_QFLIGHT : public AP_Baro_Backend
{
public:
    AP_Baro_QFLIGHT(AP_Baro &);
    void update();

private:
    DSPBuffer::BARO *barobuf;
    uint8_t instance;
    float pressure_sum;
    float temperature_sum;
    uint32_t sum_count;
    uint32_t last_check_ms;
    
    void timer_update();
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
