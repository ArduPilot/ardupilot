/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include "AP_InertialSensor_Oilpan.h"

const extern AP_HAL::HAL& hal;

// ADC channel mappings on for the APM Oilpan
// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_InertialSensor_Oilpan::_sensors[6] = { 1, 2, 0, 4, 5, 6 };

// ADC result sign adjustment for sensors.
const int8_t AP_InertialSensor_Oilpan::_sensor_signs[6] =
{ 1, -1, -1, 1, -1, -1 };

// ADC channel reading the gyro temperature
const uint8_t AP_InertialSensor_Oilpan::_gyro_temp_ch = 3;

// Maximum possible value returned by an offset-corrected sensor channel
const float AP_InertialSensor_Oilpan::_adc_constraint = 900;

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g,
// 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 418

// Oilpan accelerometer scaling & offsets
#define OILPAN_ACCEL_SCALE_1G   (GRAVITY_MSS * 2.0f / (2465.0f - 1617.0f))
#define OILPAN_RAW_ACCEL_OFFSET ((2465.0f + 1617.0f) * 0.5f)
#define OILPAN_RAW_GYRO_OFFSET  1658.0f

#define ToRad(x) radians(x)      // *pi/180
// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s,
// 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
const float AP_InertialSensor_Oilpan::_gyro_gain_x = ToRad(0.4f);
const float AP_InertialSensor_Oilpan::_gyro_gain_y = ToRad(0.41f);
const float AP_InertialSensor_Oilpan::_gyro_gain_z = ToRad(0.41f);

/* ------ Public functions -------------------------------------------*/

AP_InertialSensor_Oilpan::AP_InertialSensor_Oilpan( AP_ADC * adc ) : 
    AP_InertialSensor(),
    _adc(adc)
{
}

uint16_t AP_InertialSensor_Oilpan::_init_sensor( Sample_rate sample_rate)
{
    _adc->Init();

    switch (sample_rate) {
    case RATE_50HZ:
        _sample_threshold = 20;
        break;
    case RATE_100HZ:
        _sample_threshold = 10;
        break;
    case RATE_200HZ:
        _sample_threshold = 5;
        break;
    }

#if defined(__AVR_ATmega1280__)
    return AP_PRODUCT_ID_APM1_1280;
#else
    return AP_PRODUCT_ID_APM1_2560;
#endif
}

bool AP_InertialSensor_Oilpan::update()
{
    if (!wait_for_sample(100)) {
        return false;
    }
    float adc_values[6];
    Vector3f gyro_offset = _gyro_offset[0].get();
    Vector3f accel_scale = _accel_scale[0].get();
    Vector3f accel_offset = _accel_offset[0].get();

    _delta_time_micros = _adc->Ch6(_sensors, adc_values);
    _temp = _adc->Ch(_gyro_temp_ch);

    _gyro[0] = Vector3f(_sensor_signs[0] * ( adc_values[0] - OILPAN_RAW_GYRO_OFFSET ),
                        _sensor_signs[1] * ( adc_values[1] - OILPAN_RAW_GYRO_OFFSET ),
                        _sensor_signs[2] * ( adc_values[2] - OILPAN_RAW_GYRO_OFFSET ));
    _gyro[0].rotate(_board_orientation);
    _gyro[0].x *= _gyro_gain_x;
    _gyro[0].y *= _gyro_gain_y;
    _gyro[0].z *= _gyro_gain_z;
    _gyro[0] -= gyro_offset;

    _previous_accel[0] = _accel[0];

    _accel[0]  = Vector3f(_sensor_signs[3] * (adc_values[3] - OILPAN_RAW_ACCEL_OFFSET),
                          _sensor_signs[4] * (adc_values[4] - OILPAN_RAW_ACCEL_OFFSET),
                          _sensor_signs[5] * (adc_values[5] - OILPAN_RAW_ACCEL_OFFSET));
    _accel[0].rotate(_board_orientation);
    _accel[0].x *= accel_scale.x;
    _accel[0].y *= accel_scale.y;
    _accel[0].z *= accel_scale.z;
    _accel[0]   *= OILPAN_ACCEL_SCALE_1G;
    _accel[0] -= accel_offset;

/*
 *  X  = 1619.30 to 2445.69
 *  Y =  1609.45 to 2435.42
 *  Z =  1627.44  to 2434.82
 */

    return true;
}

float AP_InertialSensor_Oilpan::get_delta_time() {
    return _delta_time_micros * 1.0e-6;
}

/* ------ Private functions -------------------------------------------*/

// return the oilpan gyro drift rate in radian/s/s
float AP_InertialSensor_Oilpan::get_gyro_drift_rate(void)
{
    // 3.0 degrees/second/minute
    return ToRad(3.0/60);
}

// return true if a new sample is available
bool AP_InertialSensor_Oilpan::_sample_available()
{
    return (_adc->num_samples_available(_sensors) / _sample_threshold) > 0;
}

bool AP_InertialSensor_Oilpan::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        hal.scheduler->delay_microseconds(100);
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

#endif // CONFIG_HAL_BOARD

