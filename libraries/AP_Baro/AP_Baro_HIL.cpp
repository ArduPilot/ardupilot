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
        theta = (SSL_AIR_TEMPERATURE - 6.5f * h) / SSL_AIR_TEMPERATURE;
        delta = powf(theta, GMR / 6.5f);
    } else {
        // Stratosphere
        theta = 216.65f / SSL_AIR_TEMPERATURE;
        delta = 0.2233611f * expf(-GMR * (h - 11.0f) / 216.65f);
    }

    sigma = delta/theta;
}

void AP_Baro::SimpleUnderWaterAtmosphere(
	float alt,            // depth, km.
	float& rho,           // density/sea-level
	float& delta,         // pressure/sea-level standard pressure
	float& theta)         // temperature/sea-level standard temperature
{
    // Values and equations based on:
    // https://en.wikipedia.org/wiki/Standard_sea_level
    const float seaDensity = 1.024f;      // g/cm3
    const float maxSeaDensity = 1.028f;   // g/cm3
    const float pAC = maxSeaDensity - seaDensity; // pycnocline angular coefficient

    // From: https://www.windows2universe.org/earth/Water/density.html
    rho = seaDensity;
    if (alt < 1.0f) {
        // inside pycnocline
        rho += pAC*alt;
    } else {
        rho += pAC;
    }
    rho = rho/seaDensity;

    // From: https://www.grc.nasa.gov/www/k-12/WindTunnel/Activities/fluid_pressure.html
    // \f$P = \rho (kg) \cdot gravity (m/s2) \cdot depth (m)\f$
    // \f$P_{atmosphere} = 101.325 kPa\f$
    // \f$P_{total} = P_{atmosphere} + P_{fluid}\f$
    delta = (SSL_AIR_PRESSURE + (seaDensity * 1e3) * GRAVITY_MSS * (alt * 1e3)) / SSL_AIR_PRESSURE;

    // From: http://residualanalysis.blogspot.com.br/2010/02/temperature-of-ocean-water-at-given.html
    // \f$T(D)\f$ Temperature underwater at given temperature
    // \f$S\f$ Surface temperature at the surface
    // \f$T(D)\approx\frac{S}{1.8 \cdot 10^{-4} \cdot S \cdot T + 1}\f$
    const float seaTempSurface = 15.0f; // Celsius
    const float S = seaTempSurface * 0.338f;
    theta = 1.0f / ((1.8e-4f) * S * (alt * 1e3f) + 1.0f);
}

/*
  convert an altitude in meters above sea level to a presssure and temperature
 */
void AP_Baro::setHIL(float altitude_msl)
{
    float sigma, delta, theta;

    SimpleAtmosphere(altitude_msl*0.001f, sigma, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = 303.16f * theta - C_TO_KELVIN; // Assume 30 degrees at sea level - converted to degrees Kelvin

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
