/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_STUB_H__
#define __AP_INERTIAL_SENSOR_STUB_H__

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_Stub : public AP_InertialSensor
{
public:

    AP_InertialSensor_Stub();

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float	        get_delta_time();
    uint32_t        get_last_sample_time_micros();
    float           get_gyro_drift_rate();
    uint16_t        num_samples_available();

protected:
    uint16_t        _init_sensor( Sample_rate sample_rate );
    uint32_t        _sample_period_ms;
    uint32_t        _last_update_ms;
    uint32_t        _delta_time_usec;
};

#endif // __AP_INERTIAL_SENSOR_STUB_H__
