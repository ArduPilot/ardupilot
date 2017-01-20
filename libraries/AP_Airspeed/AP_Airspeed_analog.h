#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Analog : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_Analog(AP_Airspeed &frontend);

    // probe and initialise the sensor
    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override { return false; }

private:
    AP_HAL::AnalogSource *_source;
};
