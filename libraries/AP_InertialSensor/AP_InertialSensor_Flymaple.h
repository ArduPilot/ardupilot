/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_FLYMAPLE_H__
#define __AP_INERTIAL_SENSOR_FLYMAPLE_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_Flymaple : public AP_InertialSensor
{
public:

    AP_InertialSensor_Flymaple() : AP_InertialSensor() {}

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time();
    uint32_t        get_last_sample_time_micros();
    float           get_gyro_drift_rate();
    uint16_t        num_samples_available();

private:
    uint16_t        _init_sensor( Sample_rate sample_rate );
    static		    void _ins_timer(uint32_t now);
    static          void _accumulate(void);
    uint64_t        _last_update_usec;
    float           _delta_time;
    static Vector3f	_accel_sum;
    static uint32_t _accel_sum_count;
    static Vector3f	_gyro_sum;
    static uint32_t _gyro_sum_count;
    static volatile bool _in_accumulate;
    static uint64_t _last_accel_timestamp;
    static uint64_t _last_gyro_timestamp;
    uint8_t  _sample_divider;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);

    // accelerometer and gyro driver handles
    static int _accel_fd;
    static int _gyro_fd;
};
#endif
#endif // __AP_INERTIAL_SENSOR_FLYMAPLE_H__
