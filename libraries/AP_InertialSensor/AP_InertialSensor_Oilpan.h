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
  void init(AP_PeriodicProcess * scheduler);
  bool update();
  float gx();
  float gy();
  float gz();
  void get_gyros( float * );
  float ax();
  float ay();
  float az();
  void get_accels( float * );
  void get_sensors( float * );
  float temperature();
  uint32_t sample_time();
  void reset_sample_time();

  private:

  AP_ADC *_adc;
  Vector3f _gyro;
  Vector3f _accel;
  float _temp;

  uint32_t _sample_time;

  static const uint8_t _sensors[6];
  static const  int8_t _sensor_signs[6];
  static const uint8_t _gyro_temp_ch;

  static const float _gravity;
  static const float _accel_scale;

  static const float _gyro_gain_x;
  static const float _gyro_gain_y;
  static const float _gyro_gain_z;

  static const float _adc_constraint;

  float _gyro_apply_std_offset( uint16_t adc_value );
  float _accel_apply_std_offset( uint16_t adc_value );
};

#endif // __AP_INERTIAL_SENSOR_OILPAN_H__
