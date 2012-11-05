/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_STUB_H__
#define __AP_INERTIAL_SENSOR_STUB_H__

#include <string.h>
#include <stdint.h>

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "AP_InertialSensor.h"

class AP_InertialSensor_Stub : public AP_InertialSensor
{
public:

    AP_InertialSensor_Stub() {
    }

    uint16_t        _init( AP_PeriodicProcess * scheduler );

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    bool            new_data_available();
    float           gx();
    float           gy();
    float           gz();
    float           ax();
    float           ay();
    float           az();
    float           temperature();
    uint32_t        get_delta_time_micros();
    uint32_t        get_last_sample_time_micros();
    float           get_gyro_drift_rate();
    uint16_t        num_samples_available();
};

#endif // __AP_INERTIAL_SENSOR_STUB_H__
