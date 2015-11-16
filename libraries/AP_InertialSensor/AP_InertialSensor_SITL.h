/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIALSENSOR_SITL_H__
#define __AP_INERTIALSENSOR_SITL_H__

#include "AP_InertialSensor.h"
#include <SITL/SITL.h>

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

#endif // __AP_INERTIALSENSOR_SITL_H__
