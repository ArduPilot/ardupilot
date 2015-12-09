/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#pragma once

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_QURT : public AP_Compass_Backend
{
public:
    bool        init(void) override;
    void        read(void) override;

    AP_Compass_QURT(Compass &compass);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    void timer_update(void);
    
    uint8_t  instance;
    Vector3f sum;
    uint32_t count;
    uint64_t last_timestamp;
};


