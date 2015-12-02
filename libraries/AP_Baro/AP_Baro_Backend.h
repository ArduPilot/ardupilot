/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_BACKEND_H__
#define __AP_BARO_BACKEND_H__

#include "AP_Baro.h"

class AP_Baro_Backend
{
public:
    AP_Baro_Backend(AP_Baro &baro);

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update() = 0;

    // accumulate function. This is used for backends that don't use a
    // timer, and need to be called regularly by the main code to
    // trigger them to read the sensor
    virtual void accumulate(void) {}

protected:
    // reference to frontend object
    AP_Baro &_frontend;

    void _copy_to_frontend(uint8_t instance, float pressure, float temperature);
};

#endif // __AP_BARO_BACKEND_H__
