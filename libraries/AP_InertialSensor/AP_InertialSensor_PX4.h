/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_PX4_H__
#define __AP_INERTIAL_SENSOR_PX4_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

class AP_InertialSensor_PX4 : public AP_InertialSensor
{
public:

    AP_InertialSensor_PX4() : AP_InertialSensor() {}

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time();
    float           get_gyro_drift_rate();
    uint16_t        num_samples_available();

private:
    uint16_t _init_sensor( Sample_rate sample_rate );
    void     _get_sample(void);
    uint64_t _last_update_usec;
    float    _delta_time;
    Vector3f _accel_in;
    Vector3f _gyro_in;
    uint64_t _last_accel_timestamp;
    uint64_t _last_gyro_timestamp;
    uint64_t _last_sample_timestamp;
    uint16_t _num_samples_available;
    uint32_t _sample_time_usec;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);

    // accelerometer and gyro driver handles
    int _accel_fd;
    int _gyro_fd;
};
#endif
#endif // __AP_INERTIAL_SENSOR_PX4_H__
