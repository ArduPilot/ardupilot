/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_HIL.h"

const extern AP_HAL::HAL& hal;

AP_InertialSensor_HIL::AP_InertialSensor_HIL(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _sample_period_usec(0)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_HIL::detect(AP_InertialSensor &_imu, 
                                                         AP_InertialSensor::Sample_rate sample_rate)
{
    AP_InertialSensor_HIL *sensor = new AP_InertialSensor_HIL(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor(sample_rate)) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_HIL::_init_sensor(AP_InertialSensor::Sample_rate sample_rate) 
{
    switch (sample_rate) {
    case AP_InertialSensor::RATE_50HZ:
        _sample_period_usec = 20000;
        break;
    case AP_InertialSensor::RATE_100HZ:
        _sample_period_usec = 10000;
        break;
    case AP_InertialSensor::RATE_200HZ:
        _sample_period_usec = 5000;
        break;
    case AP_InertialSensor::RATE_400HZ:
        _sample_period_usec = 2500;
        break;
    }

    // grab the used instances
    _imu.register_gyro();
    _imu.register_accel();

    return true;
}

bool AP_InertialSensor_HIL::update(void) 
{
    return true;
}

bool AP_InertialSensor_HIL::_sample_available()
{
    return true;
}
