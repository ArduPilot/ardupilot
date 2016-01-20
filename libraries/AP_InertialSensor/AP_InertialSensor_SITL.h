/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <SITL/SITL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#define INS_SITL_INSTANCES 2

class AP_InertialSensor_SITL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_SITL(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool init_sensor(void);
    void timer_update();
    float rand_float(void);
    float gyro_drift(void);

    SITL::SITL *sitl;

    uint8_t gyro_instance[INS_SITL_INSTANCES];
    uint8_t accel_instance[INS_SITL_INSTANCES];
};
