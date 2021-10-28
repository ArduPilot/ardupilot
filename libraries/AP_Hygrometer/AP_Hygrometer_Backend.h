#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_Hygrometer.h"

class AP_Hygrometer_Backend
{
public:
    AP_Hygrometer_Backend(AP_Hygrometer &_frontend, uint8_t _instance, AP_Hygrometer::Hygrometer_State &_state, AP_Hygrometer::Hygrometer_Param &_params);
    virtual ~AP_Hygrometer_Backend();

    float get_humidity() const { return 100*state.humidity; }
    float get_temperature() const { return 100*state.temperature; }

    AP_Hygrometer::hygrometer_type type() const { return (AP_Hygrometer::hygrometer_type)params.type.get(); }

    // probe and initialise the sensor
    virtual bool init(void) = 0;

    virtual bool get_humidity(float &humidity) = 0;

    virtual bool get_temperature(float &temperature) = 0;

    virtual bool get_id(uint8_t &id) = 0;

protected:
    AP_Hygrometer::Hygrometer_State &state;

    //Type Backend initialised with
    AP_Hygrometer::hygrometer_type _backend_type;

    AP_Hygrometer::Hygrometer_Param &params;

private:
    AP_Hygrometer &frontend;
    uint8_t instance;
};
