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
// 1G in the raw data coming from the accelerometer
// Value based on actual sample data from 20 boards
const float AP_InertialSensor_Oilpan::_gravity = 423.8;

///< would like to use _gravity here, but cannot
//const float AP_InertialSensor_Oilpan::_accel_x_scale = 9.80665 / 413.195;
//const float AP_InertialSensor_Oilpan::_accel_y_scale = 9.80665 / 412.985;
//const float AP_InertialSensor_Oilpan::_accel_z_scale = 9.80665 / 403.69;

#define ToRad(x) (x*0.01745329252)  // *pi/180
// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s,
// 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
const float AP_InertialSensor_Oilpan::_gyro_gain_x = ToRad(0.4);
const float AP_InertialSensor_Oilpan::_gyro_gain_y = ToRad(0.41);
const float AP_InertialSensor_Oilpan::_gyro_gain_z = ToRad(0.41);

const AP_Param::GroupInfo AP_InertialSensor_Oilpan::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix
    AP_GROUPINFO("XH",          0, AP_InertialSensor_Oilpan, _x_high, 2465),
    AP_GROUPINFO("XL",          1, AP_InertialSensor_Oilpan, _x_low,  1617),
    AP_GROUPINFO("YH",          2, AP_InertialSensor_Oilpan, _y_high, 2465),
    AP_GROUPINFO("YL",          3, AP_InertialSensor_Oilpan, _y_low,  1617),
    AP_GROUPINFO("ZH",          4, AP_InertialSensor_Oilpan, _z_high, 2465),
    AP_GROUPINFO("ZL",          5, AP_InertialSensor_Oilpan, _z_low,  1617),
    AP_GROUPEND
};


/* ------ Public functions -------------------------------------------*/

AP_InertialSensor_Oilpan::AP_InertialSensor_Oilpan( AP_ADC * adc ) :
    _adc(adc)
{
    _gyro.x = 0;
    _gyro.y = 0;
    _gyro.z = 0;
    _accel.x = 0;
    _accel.y = 0;
    _accel.z = 0;
}

uint16_t AP_InertialSensor_Oilpan::init( AP_PeriodicProcess * scheduler)
{
    _adc->Init(scheduler);

    _accel_mid.x    = (_x_high + _x_low) / 2;
    _accel_mid.y    = (_y_high + _y_low) / 2;
    _accel_mid.z    = (_z_high + _z_low) / 2;
    _accel_scale.x  = 9.80665 / ((float)_x_high - _accel_mid.x);
    _accel_scale.y  = 9.80665 / ((float)_y_high - _accel_mid.y);
    _accel_scale.z  = 9.80665 / ((float)_z_high - _accel_mid.z);

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

    _sample_time = _adc->Ch6(_sensors, adc_values);
    _temp = _adc->Ch(_gyro_temp_ch);

    _gyro.x = _gyro_gain_x * _sensor_signs[0] * _gyro_apply_std_offset( adc_values[0] );
    _gyro.y = _gyro_gain_y * _sensor_signs[1] * _gyro_apply_std_offset( adc_values[1] );
    _gyro.z = _gyro_gain_z * _sensor_signs[2] * _gyro_apply_std_offset( adc_values[2] );

    // _accel.x = _accel_x_scale * _sensor_signs[3] * _accel_apply_std_offset( adc_values[3] );
    // _accel.y = _accel_y_scale * _sensor_signs[4] * _accel_apply_std_offset( adc_values[4] );
    // _accel.z = _accel_z_scale * _sensor_signs[5] * _accel_apply_std_offset( adc_values[5] );

    _accel.x = _accel_scale.x * _sensor_signs[3] * (adc_values[3] - _accel_mid.x);
    _accel.y = _accel_scale.y * _sensor_signs[4] * (adc_values[4] - _accel_mid.y);
    _accel.z = _accel_scale.z * _sensor_signs[5] * (adc_values[5] - _accel_mid.z);


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

void AP_InertialSensor_Oilpan::get_gyros( float * g )
{
    g[0] = _gyro.x;
    g[1] = _gyro.y;
    g[2] = _gyro.z;
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

void AP_InertialSensor_Oilpan::get_accels( float * a )
{
    a[0] = _accel.x;
    a[1] = _accel.y;
    a[2] = _accel.z;
}

void AP_InertialSensor_Oilpan::get_sensors( float * sensors )
{
    sensors[0] = _gyro.x;
    sensors[1] = _gyro.y;
    sensors[2] = _gyro.z;
    sensors[3] = _accel.x;
    sensors[4] = _accel.y;
    sensors[5] = _accel.z;
}

float AP_InertialSensor_Oilpan::temperature() {
    return _temp;
}

uint32_t AP_InertialSensor_Oilpan::sample_time() {
    return _sample_time;
}
void AP_InertialSensor_Oilpan::reset_sample_time() {
}

/* ------ Private functions -------------------------------------------*/

float AP_InertialSensor_Oilpan::_gyro_apply_std_offset( float adc_value )
{
    /* Magic number from AP_ADC_Oilpan.h */
    return ((float) adc_value ) - 1658.0f;
}

float AP_InertialSensor_Oilpan::_accel_apply_std_offset( float adc_value )
{
    /* Magic number from AP_ADC_Oilpan.h */
    return ((float) adc_value ) - 2041.0f;
}

// return the oilpan gyro drift rate in radian/s/s
float AP_InertialSensor_Oilpan::get_gyro_drift_rate(void)
{
    // 3.0 degrees/second/minute
    return ToRad(3.0/60);
}
