/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_OILPAN_H__
#define __AP_INERTIAL_SENSOR_OILPAN_H__

#include <string.h>
#include <stdint.h>

#include "../AP_ADC/AP_ADC.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"

class AP_InertialSensor_Oilpan : public AP_InertialSensor
{
public:

    AP_InertialSensor_Oilpan( AP_ADC * adc );

    /* Concrete implementation of AP_InertialSensor functions: */
    uint16_t        init(AP_PeriodicProcess * scheduler);
    bool            update();
    bool            new_data_available();
    float           gx();
    float           gy();
    float           gz();
    void            get_gyros( float * );
    float           ax();
    float           ay();
    float           az();
    void            get_accels( float * );
    void            get_sensors( float * );
    float           temperature();
    uint32_t        sample_time();
    float           get_gyro_drift_rate();

    // get number of samples read from the sensors
    uint16_t        num_samples_available();

    static const struct AP_Param::GroupInfo        var_info[];

    AP_Int16                    _x_high;
    AP_Int16                    _x_low;
    AP_Int16                    _y_high;
    AP_Int16                    _y_low;
    AP_Int16                    _z_high;
    AP_Int16                    _z_low;

    Vector3f                    _accel_scale;

private:
    Vector3f                    _gyro;
    Vector3f                    _accel;

    Vector3f                    _accel_high;
    Vector3f                    _accel_low;
    Vector3f                    _accel_mid;

    AP_ADC *                    _adc;

    float                       _temp;

    uint32_t                    _sample_time;

    static const uint8_t        _sensors[6];
    static const int8_t         _sensor_signs[6];
    static const uint8_t        _gyro_temp_ch;

    static const float          _gravity;

    static const float          _gyro_gain_x;
    static const float          _gyro_gain_y;
    static const float          _gyro_gain_z;

    static const float          _adc_constraint;

    float                       _gyro_apply_std_offset( float adc_value );
    float                       _accel_apply_std_offset( float adc_value );
};

#endif // __AP_INERTIAL_SENSOR_OILPAN_H__
