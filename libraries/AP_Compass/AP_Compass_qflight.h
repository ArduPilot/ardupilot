/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#pragma once

#include "Compass.h"
#include "AP_Compass_Backend.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include <AP_HAL_Linux/qflight/qflight_buffer.h>

class AP_Compass_QFLIGHT : public AP_Compass_Backend
{
public:
    bool        init(void);
    void        read(void);

    AP_Compass_QFLIGHT(Compass &compass);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    void timer_update();
    DSPBuffer::MAG *magbuf;
    uint8_t  instance;
    Vector3f sum;
    uint32_t count;
    uint32_t last_check_ms;
};

#endif // CONFIG_HAL_BOARD_SUBTYPE
