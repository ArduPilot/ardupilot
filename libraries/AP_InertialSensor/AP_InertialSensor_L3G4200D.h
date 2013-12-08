/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_L3G4200D_H__
#define __AP_INERTIAL_SENSOR_L3G4200D_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"
#include <Filter.h>
#include <LowPassFilter2p.h>

class AP_InertialSensor_L3G4200D : public AP_InertialSensor
{
public:

    AP_InertialSensor_L3G4200D();

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time();
    float           get_gyro_drift_rate();
    bool            wait_for_sample(uint16_t timeout_ms);

private:
    uint16_t        _init_sensor( Sample_rate sample_rate );
    void             _accumulate(void);
    bool            _sample_available();
    uint64_t        _last_update_usec;
    Vector3f        _accel_filtered;
    Vector3f        _gyro_filtered;
    uint32_t        _sample_period_usec;
    uint32_t        _last_sample_time;
    volatile uint32_t _gyro_samples_available;
    volatile uint8_t  _gyro_samples_needed;

    // support for updating filter at runtime
    uint8_t         _last_filter_hz;
    uint8_t          _default_filter_hz;

    void _set_filter_frequency(uint8_t filter_hz);

    // Low Pass filters for gyro and accel 
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;
};
#endif
#endif // __AP_INERTIAL_SENSOR_L3G4200D_H__
