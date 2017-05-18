#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT

#include <AP_HAL_Linux/qflight/qflight_buffer.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

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
