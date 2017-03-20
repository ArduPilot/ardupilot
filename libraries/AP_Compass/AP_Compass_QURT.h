#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_QURT : public AP_Compass_Backend
{
public:
    void        read(void) override;

    AP_Compass_QURT(Compass &compass);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    bool        init(void);
    void timer_update(void);

    uint8_t  instance;
    Vector3f sum;
    uint32_t count;
    uint64_t last_timestamp;
};


