#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_SENSORHUB_ENABLED
template <class T>
class AP_SensorHub_Handler {
public:
    virtual void handle(typename T::data_t *data) = 0;
    virtual bool isValid(typename T::data_t *data) = 0;
};
#endif