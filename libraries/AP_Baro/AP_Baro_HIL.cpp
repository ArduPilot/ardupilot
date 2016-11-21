#include "AP_Baro_HIL.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_Baro_HIL::AP_Baro_HIL(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
}

// ==========================================================================
// based on tables.cpp from http://www.pdas.com/atmosdownload.html

/* 
   Compute the temperature, density, and pressure in the standard atmosphere
   Correct to 20 km.  Only approximate thereafter.
*/
void AP_Baro::SimpleAtmosphere(
	const float alt,                           // geometric altitude, km.
	float& sigma,                   // density/sea-level standard density
	float& delta,                 // pressure/sea-level standard pressure
	float& theta)           // temperature/sea-level standard temperature
{
    const float REARTH = 6369.0f;        // radius of the Earth (km)
    const float GMR    = 34.163195f;     // gas constant
    float h=alt*REARTH/(alt+REARTH);     // geometric to geopotential altitude

    if (h < 11.0f) {
        // Troposphere
        theta=(288.15f-6.5f*h)/288.15f;
        delta=powf(theta, GMR/6.5f);
    } else {
        // Stratosphere
        theta=216.65f/288.15f;
        delta=0.2233611f*expf(-GMR*(h-11.0f)/216.65f);
    }

    sigma = delta/theta;
}


/*
  convert an altitude in meters above sea level to a presssure and temperature
 */
void AP_Baro::setHIL(float altitude_msl)
{
    float sigma, delta, theta;
    const float p0 = 101325;

    SimpleAtmosphere(altitude_msl*0.001f, sigma, delta, theta);
    float p = p0 * delta;
    float T = 303.16f * theta - 273.16f; // Assume 30 degrees at sea level - converted to degrees Kelvin

    _hil.pressure = p;
    _hil.temperature = T;
    _hil.updated = true;
}

/*
  set HIL pressure and temperature for an instance
 */
void AP_Baro::setHIL(uint8_t instance, float pressure, float temperature, float altitude, float climb_rate, uint32_t last_update_ms)
{
    if (instance >= _num_sensors) {
        // invalid
        return;
    }
    _hil.pressure = pressure;
    _hil.temperature = temperature;
    _hil.altitude = altitude;
    _hil.climb_rate = climb_rate;
    _hil.updated = true;
    _hil.have_alt = true;

    if (last_update_ms != 0) {
        _hil.last_update_ms = last_update_ms;
        _hil.have_last_update = true;
    }
}

// Read the sensor
void AP_Baro_HIL::update(void)
{
    if (_frontend._hil.updated) {
        _frontend._hil.updated = false;
        _copy_to_frontend(0, _frontend._hil.pressure, _frontend._hil.temperature);
    }
}
