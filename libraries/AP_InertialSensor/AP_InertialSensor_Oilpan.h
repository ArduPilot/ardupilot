/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_OILPAN_H__
#define __AP_INERTIAL_SENSOR_OILPAN_H__

#include <stdint.h>

#include <AP_ADC.h>
#include <AP_Math.h>
#include "AP_InertialSensor.h"

class AP_InertialSensor_Oilpan : public AP_InertialSensor
{
public:

    AP_InertialSensor_Oilpan( AP_ADC * adc );

    /* Concrete implementation of AP_InertialSensor functions: */
    bool            update();
    float        	get_delta_time();    // get_delta_time returns the time period in seconds overwhich the sensor data was collected    
    float           get_gyro_drift_rate();

    // wait for a sample to be available, with timeout in milliseconds
    bool            wait_for_sample(uint16_t timeout_ms);

protected:
    uint16_t        _init_sensor(Sample_rate sample_rate);

private:

    bool            			_sample_available();

    AP_ADC *                    _adc;

    float                       _temp;

    uint32_t                    _delta_time_micros;

    static const uint8_t        _sensors[6];
    static const int8_t         _sensor_signs[6];
    static const uint8_t        _gyro_temp_ch;

    static const float          _gyro_gain_x;
    static const float          _gyro_gain_y;
    static const float          _gyro_gain_z;

    static const float          _adc_constraint;

    uint8_t                     _sample_threshold;
};

#endif // __AP_INERTIAL_SENSOR_OILPAN_H__
