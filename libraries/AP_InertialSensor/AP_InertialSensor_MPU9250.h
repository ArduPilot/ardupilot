/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU9250_H__
#define __AP_INERTIAL_SENSOR_MPU9250_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include <Filter.h>
#include <LowPassFilter2p.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define MPU9250_DEBUG 0

class AP_InertialSensor_MPU9250 : public AP_InertialSensor_Backend
{
public:

    AP_InertialSensor_MPU9250(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _have_sample_available; }
    bool accel_sample_available(void) { return _have_sample_available; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
    bool                 _init_sensor(void);

    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    bool                 _hardware_init(void);
    bool                 _sample_available();

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    // support for updating filter at runtime
    int16_t _last_filter_hz;

    // change the filter frequency
    void _set_filter(uint8_t filter_hz);

    // This structure is used to pass data from the timer which reads
    // the sensor to the main thread. The _shared_data_idx is used to
    // prevent race conditions by ensuring the data is fully updated
    // before being used by the consumer
    struct {
        Vector3f _accel_filtered;
        Vector3f _gyro_filtered;
    } _shared_data[2];
    volatile uint8_t _shared_data_idx;

    // Low Pass filters for gyro and accel 
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;

    // do we currently have a sample pending?
    bool _have_sample_available;

    // default filter frequency when set to zero
    uint8_t _default_filter_hz;

    // gyro and accel instances
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

#if MPU9250_DEBUG
    void						_dump_registers(void);
#endif
};

#endif // __AP_INERTIAL_SENSOR_MPU9250_H__
