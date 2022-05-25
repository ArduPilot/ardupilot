/*
  SITL airspeed backend - a perfect airspeed sensor
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_AIRSPEED_SITL_ENABLED
#define AP_AIRSPEED_SITL_ENABLED AP_SIM_ENABLED
#endif

#if AP_AIRSPEED_SITL_ENABLED

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_SITL : public AP_Airspeed_Backend
{
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    bool init(void) override {
        return true;
    }

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;

private:
};

#endif // AP_AIRSPEED_SITL_ENABLED
