#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_Hygrometer.h"

class AP_Hygrometer_Backend
{
public:
    AP_Hygrometer_Backend(AP_Hygrometer &frontend, uint8_t instance);
    virtual ~AP_Hygrometer_Backend();

    // probe and initialise the sensor
    virtual bool init(void) = 0;

    virtual bool get_humidity(float &humidity) = 0;

    virtual bool get_temperature(float &temperature) = 0;

    virtual bool get_id(uint8_t &id) = 0;

private:
    AP_Hygrometer &frontend;
    uint8_t instance;
};
