/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6000_H__
#define __AP_INERTIAL_SENSOR_MPU6000_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define MPU6000_DEBUG 0

class AP_InertialSensor_MPU6000 : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_MPU6000(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _sum_count >= _sample_count; }
    bool accel_sample_available(void) { return _sum_count >= _sample_count; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu,
                                             AP_InertialSensor::Sample_rate sample_rate);

private:
#if MPU6000_DEBUG
    void _dump_registers(void);
#endif

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    AP_HAL::DigitalSource *_drdy_pin;

    bool                 _init_sensor(AP_InertialSensor::Sample_rate sample_rate);
    bool                 _sample_available();
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);
    bool                 _hardware_init(AP_InertialSensor::Sample_rate sample_rate);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    static const float          _gyro_scale;

    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_count;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;

    void _set_filter_register(uint8_t filter_hz, uint8_t default_filter);

    // count of bus errors
    uint16_t _error_count;

    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _accel_sum;
    Vector3l _gyro_sum;
    volatile uint16_t _sum_count;
};

#endif // __AP_INERTIAL_SENSOR_MPU6000_H__
