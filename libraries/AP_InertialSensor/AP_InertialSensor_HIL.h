/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIALSENSOR_HIL_H__
#define __AP_INERTIALSENSOR_HIL_H__

#include "AP_InertialSensor.h"

class AP_InertialSensor_HIL : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_HIL(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return true; }
    bool accel_sample_available(void) { return true; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool _init_sensor(void);
};

#endif // __AP_INERTIALSENSOR_HIL_H__
