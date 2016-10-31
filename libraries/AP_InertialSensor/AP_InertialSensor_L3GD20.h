/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_L3GD20_H__
#define __AP_INERTIAL_SENSOR_L3GD20_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define L3GD20_DEBUG 0

class AP_InertialSensor_L3GD20 : public AP_InertialSensor
{
public:

    AP_InertialSensor_L3GD20();

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();
    float               get_gyro_drift_rate();

    // wait for a sample to be available, with timeout in milliseconds
    bool                wait_for_sample(uint16_t timeout_ms);

    // get_delta_time returns the time period in seconds overwhich the sensor data was collected
    float            	get_delta_time() const;

    uint16_t error_count(void) const { return _error_count; }
    bool healthy(void) const { return _error_count <= 4; }
    bool get_gyro_health(uint8_t instance) const { return healthy(); }
    bool get_accel_health(uint8_t instance) const { return healthy(); }

protected:
    uint16_t                    _init_sensor( Sample_rate sample_rate );

private:
    AP_HAL::DigitalSource *_drdy_pin;

    bool                 _sample_available();
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);
    bool                 _hardware_init(Sample_rate sample_rate);
    void                 disable_i2c(void);
    uint8_t              set_samplerate(uint16_t frequency);
    uint8_t              set_range(uint8_t max_dps);

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;

    uint16_t					_num_samples;
    float          _gyro_scale;

    uint32_t _last_sample_time_micros;

    // ensure we can't initialise twice
    bool                        _initialised;
    int16_t              _L3GD20_product_id;

    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_shift;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;

    void _set_filter_register(uint8_t filter_hz, uint8_t default_filter);

    uint16_t _error_count;

    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _gyro_sum;
    volatile int16_t _sum_count;

public:

#if L3GD20_DEBUG
    void						_dump_registers(void);
#endif
};

#endif // __AP_INERTIAL_SENSOR_L3GD20_H__
