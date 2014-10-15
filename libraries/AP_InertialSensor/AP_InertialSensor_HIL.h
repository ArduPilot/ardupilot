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

    bool gyro_sample_available(void) { return _sample_available(); }
    bool accel_sample_available(void) { return _sample_available(); }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu,
                                             AP_InertialSensor::Sample_rate sample_rate);

private:
    bool _init_sensor(AP_InertialSensor::Sample_rate sample_rate);
    bool _sample_available(void);
    uint32_t _sample_period_usec;
};

#endif // __AP_INERTIALSENSOR_HIL_H__
