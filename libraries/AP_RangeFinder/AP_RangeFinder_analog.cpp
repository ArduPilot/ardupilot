// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
 *   AP_RangeFinder_analog.cpp - rangefinder for analog source
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 */

#include <AP_HAL.h>
#include "AP_RangeFinder_analog.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#include <AP_ADC_AnalogSource.h>
# define SONAR_DEFAULT_PIN 127
#else
# define SONAR_DEFAULT_PIN 0
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_RangeFinder_analog::var_info[] PROGMEM = {

    // @Param: PIN
    // @DisplayName: Sonar pin
    // @Description: Analog pin that sonar is connected to. Use pin number 127 for an APM1 Oilpan
    AP_GROUPINFO("PIN",     0, AP_RangeFinder_analog, _pin, SONAR_DEFAULT_PIN),

    // @Param: SCALING
    // @DisplayName: Sonar scaling
    // @Description: Scaling factor between sonar reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: meters/Volt
    // @Increment: 0.001
    AP_GROUPINFO("SCALING", 1, AP_RangeFinder_analog, _scaling, 3.0),

    // @Param: OFFSET
    // @DisplayName: Sonar offset
    // @Description: Offset in volts for zero distance
    // @Units: Volts
    // @Increment: 0.001
    AP_GROUPINFO("OFFSET",  2, AP_RangeFinder_analog, _offset, 0.0),

    // @Param: FUNCTION
    // @DisplayName: Sonar function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is ((5.0-voltage)-offset)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    AP_GROUPINFO("FUNCTION",  3, AP_RangeFinder_analog, _function, 0),

    // @Param: MIN_CM
    // @DisplayName: Sonar minimum distance
    // @Description: minimum distance in centimeters that sonar can reliably read
	// @Units: centimeters
    // @Increment: 1
    AP_GROUPINFO("MIN_CM",  4, AP_RangeFinder_analog, _min_distance_cm, 20),

    // @Param: MAX_CM
    // @DisplayName: Sonar maximum distance
    // @Description: maximum distance in centimeters that sonar can reliably read
	// @Units: centimeters
    // @Increment: 1
    AP_GROUPINFO("MAX_CM",  5, AP_RangeFinder_analog, _max_distance_cm, 700),

    AP_GROUPEND
};

// Constructor
AP_RangeFinder_analog::AP_RangeFinder_analog(void)
{
   AP_Param::setup_object_defaults(this, var_info);
}


/* Initialisation:
   we pass the analog source in at Init() time rather than in the
   constructor as otherwise the object could not have parameters, as
   only static objects can have parameters, but the analog sources in
   AP_HAL are allocated at runtime
*/
void AP_RangeFinder_analog::Init(void *adc)
{
   if (_source != NULL) {
	  return;
   }
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
   if (_pin == 127) {
	  _source = new AP_ADC_AnalogSource((AP_ADC*)adc, 7, 0.25);
	  _last_pin = 127;
	  return;
   }
#endif
   _source = hal.analogin->channel(_pin);
   _last_pin = _pin;
}

#define REFERENCE_VOLTAGE 5.0

/*
  return raw voltage
 */
float AP_RangeFinder_analog::voltage(void)
{
   if (_source == NULL) {
	  return 0.0;
   }
   // check for pin changes
   if (_last_pin != 127 && _last_pin != _pin) {
	  _source->set_pin(_pin);
	  _last_pin = _pin;
   }

   /* first convert to volts */
   float v = _source->read_average() * (REFERENCE_VOLTAGE / 1024.0);

   // constrain to max range of ADC
   v = constrain(v, 0.0, REFERENCE_VOLTAGE);

   return v; 
}

/*
  return distance in centimeters
 */
float AP_RangeFinder_analog::distance_cm(void)
{
   /* first convert to volts */
   float v = voltage();
   float dist_m = 0;

   switch ((AP_RangeFinder_analog::RangeFinder_Function)_function.get()) {
   case FUNCTION_LINEAR:
	  dist_m = (v - _offset) * _scaling;
	  break;
	  
   case FUNCTION_INVERTED:
	  dist_m = ((REFERENCE_VOLTAGE-v) - _offset) * _scaling;
	  break;

   case FUNCTION_HYPERBOLA:
	  if (v <= _offset) {
		 dist_m = 0;
	  }
	  dist_m = _scaling / (v - _offset);
	  if (isinf(dist_m) || dist_m > _max_distance_cm) {
		 dist_m = _max_distance_cm * 0.01;
	  }
	  break;
   }
   if (dist_m < 0) {
	  dist_m = 0;
   }
   return dist_m * 100.0;  
}

/*
  return true if we are in the configured range of the device
 */
bool AP_RangeFinder_analog::in_range(void)
{
   float dist_cm = distance_cm();
   if (dist_cm >= _max_distance_cm) {
	  return false;
   }
   if (dist_cm <= _min_distance_cm) {
	  return false;
   }
   return true;
}
