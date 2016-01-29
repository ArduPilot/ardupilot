/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Analog : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_Analog(const AP_Int8 &pin)
        : _source(NULL)
        , _pin(pin)
        , _last_pin(-1)
    { }

    // probe and initialise the sensor
    bool init(void);

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure);

    // temperature not available via analog backend
    bool get_temperature(float &temperature) { return false; }

private:
    AP_HAL::AnalogSource *_source;
    const AP_Int8 &_pin;
    int8_t _last_pin;
};
