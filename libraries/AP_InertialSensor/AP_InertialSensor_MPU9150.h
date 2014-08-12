/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU9150_H__
#define __AP_INERTIAL_SENSOR_MPU9150_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"
#include <Filter.h>
#include <LowPassFilter2p.h>


class AP_InertialSensor_MPU9150 : public AP_InertialSensor
{
public:

    AP_InertialSensor_MPU9150();

    /* Implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time() const;
    float           get_gyro_drift_rate();
    bool            wait_for_sample(uint16_t timeout_ms);

private:
    uint16_t        _init_sensor( Sample_rate sample_rate );
    void             _accumulate(void);
    bool            _sample_available();
    // uint64_t        _last_update_usec;
    Vector3f        _accel_filtered;
    Vector3f        _gyro_filtered;
    uint32_t        _sample_period_usec;
    volatile uint32_t _gyro_samples_available;
    uint64_t        _last_sample_timestamp;    
    bool     _have_sample_available;    

    // // support for updating filter at runtime
    uint8_t         _last_filter_hz;
    uint8_t          _default_filter_hz;

    int16_t mpu_set_gyro_fsr(uint16_t fsr);
    int16_t mpu_set_accel_fsr(uint8_t fsr);
    int16_t mpu_set_lpf(uint16_t lpf);
    int16_t mpu_set_sample_rate(uint16_t rate);
    int16_t mpu_set_compass_sample_rate(uint16_t rate, uint16_t chip_sample_rate);
    int16_t mpu_configure_fifo(uint8_t sensors);
    int16_t set_int_enable(uint8_t enable);
    int16_t mpu_reset_fifo(uint8_t sensors);
    int16_t mpu_set_sensors(uint8_t sensors);
    int16_t mpu_set_int_latched(uint8_t enable);
    int16_t mpu_read_fifo(int16_t *gyro, int16_t *accel, uint32_t timestamp, uint8_t *sensors, uint8_t *more);

    // Filter (specify which one)
    void _set_filter_frequency(uint8_t filter_hz);

    // Low Pass filters for gyro and accel 
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;
    // LowPassFilter2p _mag_filter_x;
    // LowPassFilter2p _mag_filter_y;
    // LowPassFilter2p _mag_filter_z;


};
#endif
#endif // __AP_INERTIAL_SENSOR_MPU9150_H__
