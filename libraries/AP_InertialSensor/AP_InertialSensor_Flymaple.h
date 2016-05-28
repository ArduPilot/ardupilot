/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_FLYMAPLE_H__
#define __AP_INERTIAL_SENSOR_FLYMAPLE_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "AP_InertialSensor.h"
#include <Filter.h>
#include <LowPassFilter2p.h>

class AP_InertialSensor_Flymaple : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Flymaple(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { _accumulate(); return _have_gyro_sample; }
    bool accel_sample_available(void) { _accumulate(); return _have_accel_sample; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool        _init_sensor(void);
    void        _accumulate(void);
    Vector3f	_accel_filtered;
    Vector3f	_gyro_filtered;
    bool        _have_gyro_sample;
    bool        _have_accel_sample;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);
    // Low Pass filters for gyro and accel 
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;

    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    uint32_t _last_gyro_timestamp;
    uint32_t _last_accel_timestamp;
};
#endif
#endif // __AP_INERTIAL_SENSOR_FLYMAPLE_H__
