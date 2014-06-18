// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   AP_RangeFinder_analog.cpp - rangefinder for analog source
 *
 */

#include <AP_HAL.h>
#include "AP_RangeFinder_analog.h"

extern const AP_HAL::HAL& hal;

#define SONAR_DEFAULT_PIN 0

// table of user settable parameters
const AP_Param::GroupInfo AP_RangeFinder_analog::var_info[] PROGMEM = {

    // @Param: PIN
    // @DisplayName: Sonar pin
    // @Description: Analog pin that sonar is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
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
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
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

    // @Param: ENABLE
    // @DisplayName: Sonar enabled
    // @Description: set to 1 to enable this sonar
	// @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO("ENABLE",  6, AP_RangeFinder_analog, _enabled, 0),

    // @Param: STOP_PIN
    // @DisplayName: Sonar stop pin
    // @Description: Digital pin that enables/disables sonar measurement. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the sonar and set to 0 to disable it. This can be used to ensure that multiple sonars don't interfere with each other.
    AP_GROUPINFO("STOP_PIN", 7, AP_RangeFinder_analog, _stop_pin, -1),

    // @Param: SETTLE_MS
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the sonar reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the sonar to give a reading after we set the STOP_PIN high. For a sonar with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: milliseconds
    // @Increment: 1
    AP_GROUPINFO("SETTLE_MS", 8, AP_RangeFinder_analog, _settle_time_ms, 0),

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
   if (!_enabled) {
	  return;
   }
   if (_source != NULL) {
	  return;
   }
   _source = hal.analogin->channel(_pin);
   _source->set_stop_pin((uint8_t)_stop_pin);
   _source->set_settle_time((uint16_t)_settle_time_ms);
}

/*
  return raw voltage
 */
float AP_RangeFinder_analog::voltage(void)
{
   if (!_enabled) {
	  return 0.0f;
   }
   if (_source == NULL) {
	  return 0.0f;
   }
   // cope with changed settings
   _source->set_pin(_pin);
   _source->set_stop_pin((uint8_t)_stop_pin);
   _source->set_settle_time((uint16_t)_settle_time_ms);
   return _source->voltage_average_ratiometric();
}

/*
  return distance in centimeters
 */
float AP_RangeFinder_analog::distance_cm(void)
{
   if (!_enabled) {
	  return 0.0f;
   }

   /* first convert to volts */
   float v = voltage();
   float dist_m = 0;

   switch ((AP_RangeFinder_analog::RangeFinder_Function)_function.get()) {
   case FUNCTION_LINEAR:
	  dist_m = (v - _offset) * _scaling;
	  break;
	  
   case FUNCTION_INVERTED:
	  dist_m = (_offset - v) * _scaling;
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
   return dist_m * 100.0f;  
}

/*
  return true if we are in the configured range of the device
 */
bool AP_RangeFinder_analog::in_range(void)
{
   if (!_enabled) {
	  return false;
   }
   float dist_cm = distance_cm();
   if (dist_cm >= _max_distance_cm) {
	  return false;
   }
   if (dist_cm <= _min_distance_cm) {
	  return false;
   }
   return true;
}
