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

    AP_InertialSensor_PX4() : 
        AP_InertialSensor(),
        _sample_time_usec(0),
        _last_get_sample_timestamp(0)
        {
        }

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time();
    float           get_gyro_drift_rate();
    bool            wait_for_sample(uint16_t timeout_ms);
    bool            healthy(void) const;

    // multi-device interface
    bool get_gyro_health(uint8_t instance) const;
    uint8_t get_gyro_count(void) const;

    bool get_accel_health(uint8_t instance) const;
    uint8_t get_accel_count(void) const;

private:
    uint8_t _get_primary_gyro(void) const;
    uint8_t _get_primary_accel(void) const;

    uint16_t _init_sensor( Sample_rate sample_rate );
    void     _get_sample(void);
    bool     _sample_available(void);
    Vector3f _accel_in[INS_MAX_INSTANCES];
    Vector3f _gyro_in[INS_MAX_INSTANCES];
    uint64_t _last_accel_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_gyro_timestamp[INS_MAX_INSTANCES];
    uint64_t _last_get_sample_timestamp;
    uint64_t _last_sample_timestamp;
    uint32_t _sample_time_usec;
    bool     _have_sample_available;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);

    // accelerometer and gyro driver handles
    uint8_t _num_accel_instances;
    uint8_t _num_gyro_instances;
    int _accel_fd[INS_MAX_INSTANCES];
    int _gyro_fd[INS_MAX_INSTANCES];
};
#endif
#endif // __AP_INERTIAL_SENSOR_PX4_H__
