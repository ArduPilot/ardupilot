#pragma once

#if AP_HYGROMETER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Hygrometer.h"

class AP_Hygrometer_Backend
{
public:
    AP_Hygrometer_Backend(AP_Hygrometer::Param &_params) : params(_params) { }

    virtual ~AP_Hygrometer_Backend(void) { }

    AP_Hygrometer::Type type() const {
        return (AP_Hygrometer::Type)params.type.get();
    }

    // probe and initialise the sensor
    virtual bool init(void) = 0;

    virtual bool get_humidity(float &humidity) = 0;

    virtual bool get_temperature(float &temperature) = 0;

protected:
    AP_Hygrometer::Param &params;
};
#endif // AP_HYGROMETER_ENABLED
