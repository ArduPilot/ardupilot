#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Synthetic : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_Synthetic(AP_Airspeed &frontend, uint8_t _instance);

    // probe and initialise the sensor
    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via synthetic backend
    bool get_temperature(float &temperature) override { return false; }

    // return true if this is a synthetic airspeed sensor
    bool is_synthetic(void) const override { return true; }
};
