/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_PX4 : public AP_Compass_Backend
{
public:
    bool        init(void);
    void        read(void);
    void        accumulate(void);

    AP_Compass_PX4(Compass &compass);
    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    uint8_t  _num_sensors;

    uint8_t  _instance[COMPASS_MAX_INSTANCES];
    int      _mag_fd[COMPASS_MAX_INSTANCES];
    Vector3f _sum[COMPASS_MAX_INSTANCES];
    uint32_t _count[COMPASS_MAX_INSTANCES];
    uint64_t _last_timestamp[COMPASS_MAX_INSTANCES];
};
