/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#include "AP_InertialSensor_Oilpan.h"
#include <AP_ADC.h>

const extern AP_HAL::HAL& hal;

// this driver assumes an AP_ADC object has been declared globally
extern AP_ADC_ADS7844 apm1_adc;

// ADC channel mappings on for the APM Oilpan
// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_InertialSensor_Oilpan::_sensors[6] = { 1, 2, 0, 4, 5, 6 };

// ADC result sign adjustment for sensors.
const int8_t AP_InertialSensor_Oilpan::_sensor_signs[6] =
{ 1, -1, -1, 1, -1, -1 };

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

// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s,
// 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
const float AP_InertialSensor_Oilpan::_gyro_gain_x = radians(0.4f);
const float AP_InertialSensor_Oilpan::_gyro_gain_y = radians(0.41f);
const float AP_InertialSensor_Oilpan::_gyro_gain_z = radians(0.41f);

/* ------ Public functions -------------------------------------------*/

AP_InertialSensor_Oilpan::AP_InertialSensor_Oilpan(AP_InertialSensor &imu) : 
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_Oilpan::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_Oilpan *sensor = new AP_InertialSensor_Oilpan(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

bool AP_InertialSensor_Oilpan::_init_sensor(void)
{
    apm1_adc.Init();

    switch (_imu.get_sample_rate()) {
    case AP_InertialSensor::RATE_50HZ:
        _sample_threshold = 20;
        break;
    case AP_InertialSensor::RATE_100HZ:
        _sample_threshold = 10;
        break;
    case AP_InertialSensor::RATE_200HZ:
        _sample_threshold = 5;
        break;
    default:
        // can't do this speed
        return false;
    }

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    _product_id = AP_PRODUCT_ID_APM1_2560;

    return true;
}

/*
  copy data from ADC to frontend
 */
bool AP_InertialSensor_Oilpan::update()
{
    float adc_values[6];

    apm1_adc.Ch6(_sensors, adc_values);

    // copy gyros to frontend
    Vector3f v;
    v(_sensor_signs[0] * ( adc_values[0] - OILPAN_RAW_GYRO_OFFSET ) * _gyro_gain_x,
      _sensor_signs[1] * ( adc_values[1] - OILPAN_RAW_GYRO_OFFSET ) * _gyro_gain_y,
      _sensor_signs[2] * ( adc_values[2] - OILPAN_RAW_GYRO_OFFSET ) * _gyro_gain_z);
    _rotate_and_offset_gyro(_gyro_instance, v);

    // copy accels to frontend
    v(_sensor_signs[3] * (adc_values[3] - OILPAN_RAW_ACCEL_OFFSET),
      _sensor_signs[4] * (adc_values[4] - OILPAN_RAW_ACCEL_OFFSET),
      _sensor_signs[5] * (adc_values[5] - OILPAN_RAW_ACCEL_OFFSET));
    v *= OILPAN_ACCEL_SCALE_1G;
    _rotate_and_offset_accel(_accel_instance, v);

    return true;
}

// return true if a new sample is available
bool AP_InertialSensor_Oilpan::_sample_available() const
{
    return apm1_adc.num_samples_available(_sensors) >= _sample_threshold;
}

#endif // CONFIG_HAL_BOARD

