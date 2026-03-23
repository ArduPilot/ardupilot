#pragma once

#include "AP_TemperatureSensor_Backend.h"

#if AP_TEMPERATURE_SENSOR_TMP117_ENABLED

class AP_TemperatureSensor_TMP117 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;
public:

    __INITFUNC__ void init(void) override;

    void update() override {};

private:

    // update the temperature, called at 20Hz
    void _timer(void);
};
#endif  // AP_TEMPERATURE_SENSOR_TMP117_ENABLED
