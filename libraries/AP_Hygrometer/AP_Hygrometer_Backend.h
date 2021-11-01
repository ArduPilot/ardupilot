#pragma once

#if HAL_HYGROMETER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Hygrometer.h"

class AP_Hygrometer_Backend
{
public:
    AP_Hygrometer_Backend(AP_Hygrometer &_frontend, uint8_t _instance, AP_Hygrometer::State &_state, AP_Hygrometer::Param &_params);
    
    virtual ~AP_Hygrometer_Backend(void) { }
   
    float get_humidity() const { return state.humidity; }
    float get_temperature() const { return state.temperature; }

    AP_Hygrometer::Type type() const { return (AP_Hygrometer::Type)params.type.get(); }

    // probe and initialise the sensor
    virtual bool init(void) = 0;

    virtual bool get_humidity(float &humidity) = 0;

    virtual bool get_temperature(float &temperature) = 0;

    virtual bool get_id(uint8_t &id) = 0;

protected:
    AP_Hygrometer::State &state;

    AP_Hygrometer::Param &params;

private:
    AP_Hygrometer &frontend;
    uint8_t instance;
};
#endif // HAL_HYGROMETER_ENABLED
