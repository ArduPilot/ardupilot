/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_OILPAN_H__
#define __AP_INERTIAL_SENSOR_OILPAN_H__

#include <AP_HAL.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_Oilpan : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Oilpan(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _sample_available(); }
    bool accel_sample_available(void) { return _sample_available(); }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool                        _init_sensor(void);
    bool            			_sample_available() const;
    static const uint8_t        _sensors[6];
    static const int8_t         _sensor_signs[6];
    static const float          _gyro_gain_x;
    static const float          _gyro_gain_y;
    static const float          _gyro_gain_z;
    static const float          _adc_constraint;
    uint8_t                     _sample_threshold;
    uint8_t                     _gyro_instance;
    uint8_t                     _accel_instance;
};

#endif // __AP_INERTIAL_SENSOR_OILPAN_H__
