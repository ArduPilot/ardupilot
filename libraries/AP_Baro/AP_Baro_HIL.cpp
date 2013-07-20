/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Baro.h>
#include "AP_Baro_HIL.h"
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_HIL::init()
{
    BMP085_State=1;
    return true;
}

// Read the sensor. This is a state machine
// We read one time Temperature (state = 1) and then 4 times Pressure (states 2-5)
uint8_t AP_Baro_HIL::read()
{
    uint8_t result = 0;

    if (_count != 0) {
        result = 1;
        Press = ((float)_pressure_sum) / _count;
        Temp = ((float)_temperature_sum) / _count;
        _pressure_samples = _count;
        _count = 0;
        _pressure_sum = 0;
        _temperature_sum = 0;
    }

    return result;
}

// ==========================================================================
// based on tables.cpp from http://www.pdas.com/atmosdownload.html

/* 
 Compute the temperature, density, and pressure in the standard atmosphere
 Correct to 20 km.  Only approximate thereafter.
*/
static void SimpleAtmosphere(
	const float alt,                           // geometric altitude, km.
	float& sigma,                   // density/sea-level standard density
	float& delta,                 // pressure/sea-level standard pressure
	float& theta)           // temperature/sea-level standard temperature
{
  const float REARTH = 6369.0f;        // radius of the Earth (km)
  const float GMR    = 34.163195f;     // gas constant
  float h=alt*REARTH/(alt+REARTH);     // geometric to geopotential altitude

  if (h<11.0)
    {                                                          // Troposphere
      theta=(288.15f-6.5f*h)/288.15f;
      delta=powf(theta, GMR/6.5f);
    }
  else
    {                                                         // Stratosphere
      theta=216.65f/288.15f;
      delta=0.2233611f*expf(-GMR*(h-11.0f)/216.65f);
    }

  sigma=delta/theta;
}


void AP_Baro_HIL::setHIL(float altitude_msl)
{
    float sigma, delta, theta;
    const float p0 = 101325;

    SimpleAtmosphere(altitude_msl*0.001, sigma, delta, theta);
    float p = p0 * delta;
    float T = 303.16 * theta - 273.16; // Assume 30 degrees at sea level - converted to degrees Kelvin

    _count++;
    _pressure_sum += p;
    _temperature_sum += T;

    if (_count == 128) {
        // we have summed 128 values. This only happens
        // when we stop reading the barometer for a long time
        // (more than 1.2 seconds)
        _count = 64;
        _pressure_sum /= 2;
        _temperature_sum /= 2;
    }

    healthy = true;
    _last_update = hal.scheduler->millis();
}

float AP_Baro_HIL::get_pressure() {
    return Press;
}

float AP_Baro_HIL::get_temperature() {
    return Temp;
}

int32_t AP_Baro_HIL::get_raw_pressure() {
    return Press;
}

int32_t AP_Baro_HIL::get_raw_temp() {
    return Temp;
}
