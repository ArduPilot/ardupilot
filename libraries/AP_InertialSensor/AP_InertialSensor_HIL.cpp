/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_HIL.h"

const extern AP_HAL::HAL& hal;

AP_InertialSensor_HIL::AP_InertialSensor_HIL(AP_InertialSensor &imu, Vector3f &gyro, Vector3f &accel) :
    AP_InertialSensor_Backend(imu, gyro, accel),
    _sample_period_usec(0),
    _last_sample_usec(0)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_HIL::detect(AP_InertialSensor &_imu, 
                                                         AP_InertialSensor::Sample_rate sample_rate,
                                                         Vector3f &gyro, 
                                                         Vector3f &accel)
{
    AP_InertialSensor_HIL *sensor = new AP_InertialSensor_HIL(_imu, gyro, accel);
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
    uint32_t now = hal.scheduler->micros();
    while (now - _last_sample_usec > _sample_period_usec) {
        _last_sample_usec += _sample_period_usec;
    }
    return true;
}

bool AP_InertialSensor_HIL::_sample_available()
{
    return (hal.scheduler->micros() - _last_sample_usec > _sample_period_usec);
}

