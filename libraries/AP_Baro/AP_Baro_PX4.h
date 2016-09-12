/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_PX4 : public AP_Baro_Backend
{
public:
    AP_Baro_PX4(AP_Baro &);
    void update();

private:
    uint8_t _num_instances;

    struct px4_instance {
        uint8_t instance;
        int fd;
        float pressure_sum;
        float temperature_sum;
        uint32_t sum_count;
        uint64_t last_timestamp;
    } instances[BARO_MAX_INSTANCES];
};
