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

    // callback for UAVCAN messages
    virtual void handle_baro_msg(float pressure, float temperature) {}

protected:
    // reference to frontend object
    AP_Baro &_frontend;

    void _copy_to_frontend(uint8_t instance, float pressure, float temperature);

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;    
};
