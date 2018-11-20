#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_HIL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_HIL(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update() override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool _init_sensor(void);
};
