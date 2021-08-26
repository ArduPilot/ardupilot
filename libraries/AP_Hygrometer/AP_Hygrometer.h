#pragma once

#include "stdio.h"

#ifndef HYGROMETER_MAX_SENSORS
#define HYGROMETER_MAX_SENSORS  1
#endif

class AP_Hygrometer_Backend;

class AP_Hygrometer
{
public:
    friend class AP_Hygrometer_Backend;

    // constructor
    AP_Hygrometer();

    void init(void);

    bool get_temperature(float &temperature);

    bool get_humidity(float &humidity);

    bool get_id(uint8_t &id);

private:
    // current primary sensor
    uint8_t primary = 0;

    AP_Hygrometer_Backend *sensor;
};
