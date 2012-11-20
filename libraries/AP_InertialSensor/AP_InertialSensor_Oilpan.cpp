/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_InertialSensor_Oilpan.h"

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
#define OILPAN_ACCEL_SCALE_1G   (GRAVITY * 2.0 / (2465.0 - 1617.0))
#define OILPAN_RAW_ACCEL_OFFSET ((2465.0 + 1617.0) * 0.5)
#define OILPAN_RAW_GYRO_OFFSET  1658.0

#define ToRad(x) (x*0.01745329252)  // *pi/180
// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s,
// 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
const float AP_InertialSensor_Oilpan::_gyro_gain_x = ToRad(0.4);
const float AP_InertialSensor_Oilpan::_gyro_gain_y = ToRad(0.41);
const float AP_InertialSensor_Oilpan::_gyro_gain_z = ToRad(0.41);

/* ------ Public functions -------------------------------------------*/

AP_InertialSensor_Oilpan::AP_InertialSensor_Oilpan( AP_ADC * adc ) :
    _adc(adc)
{
}

uint16_t AP_InertialSensor_Oilpan::_init_sensor( AP_PeriodicProcess * scheduler)
{
    _adc->Init(scheduler);

#if defined(DESKTOP_BUILD)
    return AP_PRODUCT_ID_SITL;
#elif defined(__AVR_ATmega1280__)
    return AP_PRODUCT_ID_APM1_1280;
#else
    return AP_PRODUCT_ID_APM1_2560;
#endif
}

bool AP_InertialSensor_Oilpan::update()
{
    float adc_values[6];
    Vector3f gyro_offset = _gyro_offset.get();
    Vector3f accel_scale = _accel_scale.get();
    Vector3f accel_offset = _accel_offset.get();


    _delta_time_micros = _adc->Ch6(_sensors, adc_values);
    _temp = _adc->Ch(_gyro_temp_ch);

    _gyro.x = _gyro_gain_x * _sensor_signs[0] * ( adc_values[0] - OILPAN_RAW_GYRO_OFFSET );
    _gyro.y = _gyro_gain_y * _sensor_signs[1] * ( adc_values[1] - OILPAN_RAW_GYRO_OFFSET );
    _gyro.z = _gyro_gain_z * _sensor_signs[2] * ( adc_values[2] - OILPAN_RAW_GYRO_OFFSET );
    _gyro -= gyro_offset;

    _accel.x = accel_scale.x * _sensor_signs[3] * (adc_values[3] - OILPAN_RAW_ACCEL_OFFSET) * OILPAN_ACCEL_SCALE_1G;
    _accel.y = accel_scale.y * _sensor_signs[4] * (adc_values[4] - OILPAN_RAW_ACCEL_OFFSET) * OILPAN_ACCEL_SCALE_1G;
    _accel.z = accel_scale.z * _sensor_signs[5] * (adc_values[5] - OILPAN_RAW_ACCEL_OFFSET) * OILPAN_ACCEL_SCALE_1G;
    _accel -= accel_offset;

/*
 *  X  = 1619.30 to 2445.69
 *  Y =  1609.45 to 2435.42
 *  Z =  1627.44  to 2434.82
 */

    return true;
}

bool AP_InertialSensor_Oilpan::new_data_available( void )
{
    return _adc->new_data_available(_sensors);
}

float AP_InertialSensor_Oilpan::gx() {
    return _gyro.x;
}
float AP_InertialSensor_Oilpan::gy() {
    return _gyro.y;
}
float AP_InertialSensor_Oilpan::gz() {
    return _gyro.z;
}

float AP_InertialSensor_Oilpan::ax() {
    return _accel.x;
}
float AP_InertialSensor_Oilpan::ay() {
    return _accel.y;
}
float AP_InertialSensor_Oilpan::az() {
    return _accel.z;
}


float AP_InertialSensor_Oilpan::temperature() {
    return _temp;
}

uint32_t AP_InertialSensor_Oilpan::get_delta_time_micros() {
    return _delta_time_micros;
}

/* ------ Private functions -------------------------------------------*/

// return the oilpan gyro drift rate in radian/s/s
float AP_InertialSensor_Oilpan::get_gyro_drift_rate(void)
{
    // 3.0 degrees/second/minute
    return ToRad(3.0/60);
}

// get number of samples read from the sensors
uint16_t AP_InertialSensor_Oilpan::num_samples_available()
{
    return _adc->num_samples_available(_sensors);
}
