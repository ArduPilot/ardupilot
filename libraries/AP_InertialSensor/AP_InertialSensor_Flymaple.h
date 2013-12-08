/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_FLYMAPLE_H__
#define __AP_INERTIAL_SENSOR_FLYMAPLE_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"
#include <Filter.h>
#include <LowPassFilter2p.h>

class AP_InertialSensor_Flymaple : public AP_InertialSensor
{
public:

    AP_InertialSensor_Flymaple() : AP_InertialSensor() {}

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time();
    float           get_gyro_drift_rate();
    bool            wait_for_sample(uint16_t timeout_ms);

private:
    uint16_t        _init_sensor( Sample_rate sample_rate );
    static          void _accumulate(void);
    bool            _sample_available();
    uint64_t        _last_update_usec;
    float           _delta_time;
    static Vector3f	_accel_filtered;
    static uint32_t _accel_samples;
    static Vector3f	_gyro_filtered;
    static uint32_t _gyro_samples;
    static uint64_t _last_accel_timestamp;
    static uint64_t _last_gyro_timestamp;
    uint8_t  _sample_divider;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);
    // Low Pass filters for gyro and accel 
    static LowPassFilter2p _accel_filter_x;
    static LowPassFilter2p _accel_filter_y;
    static LowPassFilter2p _accel_filter_z;
    static LowPassFilter2p _gyro_filter_x;
    static LowPassFilter2p _gyro_filter_y;
    static LowPassFilter2p _gyro_filter_z;
};
#endif
#endif // __AP_INERTIAL_SENSOR_FLYMAPLE_H__
