#pragma once

#include "AP_Baro.h"

class AP_Baro_Backend
{
public:
    AP_Baro_Backend(AP_Baro &baro);
    virtual ~AP_Baro_Backend(void) {};

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update() = 0;

    // accumulate function. This is used for backends that don't use a
    // timer, and need to be called regularly by the main code to
    // trigger them to read the sensor
    virtual void accumulate(void) {}

    void backend_update(uint8_t instance);

    //  Check that the baro valid by using a mean filter.
    // If the value further that filtrer_range from mean value, it is rejected.
    bool pressure_ok(float press);
    uint32_t get_error_count() const { return _error_count; }
protected:
    // reference to frontend object
    AP_Baro &_frontend;

    void _copy_to_frontend(uint8_t instance, float pressure, float temperature);

    // semaphore for access to shared frontend data
    HAL_Semaphore_Recursive _sem;

    virtual void update_healthy_flag(uint8_t instance);

    // mean pressure for range filter
    float _mean_pressure; 
    // number of dropped samples. Not used for now, but can be usable to choose more reliable sensor
    uint32_t _error_count;
};
