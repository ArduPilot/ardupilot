#include "AP_Baro.h"
#include <AP_InternalError/AP_InternalError.h>

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

/* 1976 U.S. Standard Atmosphere: https://ntrs.nasa.gov/api/citations/19770009539/downloads/19770009539.pdf?attachment=true

The US Standard Atmosphere defines the atmopshere in terms of whether the temperature is an iso-thermal or gradient layer.
Ideal gas laws apply thus P = rho * R_specific * T : P = pressure, rho = density, R_specific = air gas constant, T = temperature

Note: the 1976 model is the same as the 1962 US Standard Atomsphere up to 51km.
R_universal: the universal gas constant is slightly off in the 1976 model and thus R_specific is different than today's value

*/

/* Model Constants
R_universal = 8.31432e-3;   // Universal gas constant (J/(kmol-K)) incorrect to the redefined 2019 value of 8.31446261815324 J⋅K−1⋅mol−1
M_air  = (0.78084 * 28.0134 + 0.209476 * 31.9988 + 9.34e-3 * 39.948 + 3.14e-4 * 44.00995 + 1.818e-5 * 20.183 + 5.24E-6 * 4.0026 + 1.14E-6 * 83.8 + 8.7E-7 * 131.30 + 2E-6 * 16.04303 + 5E-7 * 2.01594) * 1E-3; (kg/mol)
M_air = 28.9644             // Molecular weight of air (kg/kmol) see page 3
R_specific  = 287.053072    // air specifc gas constant (J⋅kg−1⋅K−1), R_universal / M_air;
gama = 1.4;                 // specific heat ratio of air used to determine the speed of sound

R0 = 6356.766E3;            // Earth's radius (in m)
g0 = 9.80665;               // gravity (m/s^2)

Sea-Level Constants
H_asml = 0 meters
T0     = 288.150 K
P0     = 101325 Pa
rho0   = 1.2250 kg/m^3 
T0_slope = -6.5E-3 (K/m')

The tables list altitudes -5 km to 0 km using the same equations as 0 km to 11 km.
*/

#ifndef AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED
// default to using the extended functions when doing double precision EKF (which implies more flash space and faster MCU)
// this allows for using the simple model with the --ekf-single configure option
#define AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED HAL_WITH_EKF_DOUBLE
#endif

/*
  return altitude difference in meters between current pressure and a
  given base_pressure in Pascal. This is a simple atmospheric model
  good to about 11km AMSL.
*/
float AP_Baro::get_altitude_difference_simple(float base_pressure, float pressure) const
{
    float ret;
    float temp_K = C_TO_KELVIN(get_ground_temperature());
    float scaling = pressure / base_pressure;

    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    ret = 153.8462f * temp_K * (1.0f - expf(0.190259f * logf(scaling)));

    return ret;
}

#if AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED || AP_SIM_ENABLED

/*
  Note parameters are as defined in the 1976 model.
  These are slightly different from the ones in definitions.h
*/
static const float radius_earth = 6356.766E3;      // Earth's radius (in m)
static const float R_specific = 287.053072;        // air specifc gas constant (J⋅kg−1⋅K−1) in 1976 model, R_universal / M_air;

static const struct {
    float amsl_m;       // geopotential height above mean sea-level (km')
    float temp_K;       // Temperature (K)
    float pressure_Pa;  // Pressure (Pa)
    float density;      // Density (Pa/kg)
    float temp_lapse;   // Temperature gradients rates (K/m'), see page 3
} atmospheric_1976_consts[] = {
    { -5000,    320.650,     177687,      1.930467,    -6.5E-3 },
    { 11000,    216.650,    22632.1,      0.363918,          0 },
    { 20000,    216.650,    5474.89,    8.80349E-2,       1E-3 },
    { 32000,    228.650,    868.019,    1.32250E-2,     2.8E-3 },
    { 47000,    270.650,    110.906,    1.42753E-3,          0 },
    { 51000,    270.650,    66.9389,    8.61606E-4,    -2.8E-3 },
    { 71000,    214.650,    3.95642,    6.42110E-5,    -2.0E-3 },
    { 84852,    186.946,    0.37338,    6.95788E-6,          0 },
};

/*
  find table entry given geopotential altitude in meters. This returns at least 1
 */
static uint8_t find_atmosphere_layer_by_altitude(float alt_m)
{
    for (uint8_t idx = 1; idx < ARRAY_SIZE(atmospheric_1976_consts); idx++) {
        if(alt_m < atmospheric_1976_consts[idx].amsl_m) {
            return idx - 1;
        }
    }

    // Over the largest altitude return the last index
    return ARRAY_SIZE(atmospheric_1976_consts) - 1;
}

/*
  find table entry given pressure (Pa). This returns at least 1
 */
static uint8_t find_atmosphere_layer_by_pressure(float pressure)
{
    for (uint8_t idx = 1; idx < ARRAY_SIZE(atmospheric_1976_consts); idx++) {
        if (atmospheric_1976_consts[idx].pressure_Pa < pressure) {
            return idx - 1;
        }
    }

    // pressure is less than the smallest pressure return the last index
    return ARRAY_SIZE(atmospheric_1976_consts) - 1;

}

// Convert geopotential altitude to geometric altitude
// 
float AP_Baro::geopotential_alt_to_geometric(float alt)
{
    return (radius_earth * alt) / (radius_earth - alt);
}

float AP_Baro::geometric_alt_to_geopotential(float alt)
{
    return (radius_earth * alt) / (radius_earth + alt);
}

/*
  Compute expected temperature for a given geometric altitude and altitude layer.
*/
float AP_Baro::get_temperature_from_altitude(float alt) const
{
    alt = geometric_alt_to_geopotential(alt);
    const uint8_t idx = find_atmosphere_layer_by_altitude(alt);
    return get_temperature_by_altitude_layer(alt, idx);
}

/*
  Compute expected temperature for a given geopotential altitude and altitude layer.
*/
float AP_Baro::get_temperature_by_altitude_layer(float alt, int8_t idx)
{
    if (is_zero(atmospheric_1976_consts[idx].temp_lapse)) {
        return atmospheric_1976_consts[idx].temp_K;
    }
    return atmospheric_1976_consts[idx].temp_K + atmospheric_1976_consts[idx].temp_lapse * (alt - atmospheric_1976_consts[idx].amsl_m);
}

/*
  return geometric altitude (m) given a pressure (Pa)
*/
float AP_Baro::get_altitude_from_pressure(float pressure) const
{
    const uint8_t idx = find_atmosphere_layer_by_pressure(pressure);
    const float pressure_ratio = pressure / atmospheric_1976_consts[idx].pressure_Pa;

    // Pressure ratio is nonsensical return an error??
    if (!is_positive(pressure_ratio)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return get_altitude_AMSL();
    }

    float alt;
    const float temp_slope = atmospheric_1976_consts[idx].temp_lapse;
    if (is_zero(temp_slope)) {  // Iso-thermal layer
        const float fac = -(atmospheric_1976_consts[idx].temp_K * R_specific) / GRAVITY_MSS;
        alt = atmospheric_1976_consts[idx].amsl_m + fac * logf(pressure_ratio);
    } else {                    // Gradient temperature layer
        const float fac = -(temp_slope * R_specific) / GRAVITY_MSS;
        alt = atmospheric_1976_consts[idx].amsl_m + (atmospheric_1976_consts[idx].temp_K / atmospheric_1976_consts[idx].temp_lapse) * (powf(pressure_ratio, fac) - 1);
    }

    return geopotential_alt_to_geometric(alt);
}

/*
  Compute expected pressure and temperature for a given geometric altitude. Used for SITL
*/
void AP_Baro::get_pressure_temperature_for_alt_amsl(float alt_amsl, float &pressure, float &temperature_K)
{
    alt_amsl = geometric_alt_to_geopotential(alt_amsl);

    const uint8_t idx = find_atmosphere_layer_by_altitude(alt_amsl);
    const float temp_slope = atmospheric_1976_consts[idx].temp_lapse;
    temperature_K =  get_temperature_by_altitude_layer(alt_amsl, idx);

    // Previous versions used the current baro temperature instead of an estimate we could try to incorporate this??? non-standard atmosphere
    // const float temp =  get_temperature();

    if (is_zero(temp_slope)) {    // Iso-thermal layer
        const float fac = expf(-GRAVITY_MSS / (temperature_K * R_specific) * (alt_amsl - atmospheric_1976_consts[idx].amsl_m));
        pressure = atmospheric_1976_consts[idx].pressure_Pa * fac;
    } else {            // Gradient temperature layer
        const float fac =  GRAVITY_MSS / (temp_slope * R_specific);
        const float temp_ratio = temperature_K / atmospheric_1976_consts[idx].temp_K; // temperature ratio [unitless]
        pressure = atmospheric_1976_consts[idx].pressure_Pa * powf(temp_ratio, -fac);
    }
}

/*
  return air density (kg/m^3), given geometric altitude (m)
*/
float AP_Baro::get_air_density_for_alt_amsl(float alt_amsl)
{
    alt_amsl = geometric_alt_to_geopotential(alt_amsl);

    const uint8_t idx = find_atmosphere_layer_by_altitude(alt_amsl);
    const float temp_slope = atmospheric_1976_consts[idx].temp_lapse;
    const float temp =  get_temperature_by_altitude_layer(alt_amsl, idx);
    // const float temp =  get_temperature();

    float rho;
    if (is_zero(temp_slope)) {    // Iso-thermal layer
        const float fac = expf(-GRAVITY_MSS / (temp * R_specific) * (alt_amsl - atmospheric_1976_consts[idx].amsl_m));
        rho      = atmospheric_1976_consts[idx].density     * fac;
    } else {            // Gradient temperature layer
        const float fac =  GRAVITY_MSS / (temp_slope * R_specific);
        const float temp_ratio = temp / atmospheric_1976_consts[idx].temp_K; // temperature ratio [unitless]
        rho      = atmospheric_1976_consts[idx].density     * powf(temp_ratio, -(fac + 1));
    }
    return rho;
}

/*
  return current scale factor that converts from equivalent to true airspeed
*/
float AP_Baro::get_EAS2TAS_extended(float altitude) const
{
    float density = get_air_density_for_alt_amsl(altitude);
    if (!is_positive(density)) {
        // above this height we are getting closer to spacecraft territory...
        const uint8_t table_size = ARRAY_SIZE(atmospheric_1976_consts);
        density = atmospheric_1976_consts[table_size-1].density;
    }
    return sqrtf(SSL_AIR_DENSITY / density);
}

/*
  Given the geometric altitude (m)
  return scale factor that converts from equivalent to true airspeed
  used by SITL only
*/
float AP_Baro::get_EAS2TAS_for_alt_amsl(float alt_amsl)
{
    const float density = get_air_density_for_alt_amsl(alt_amsl);
    return sqrtf(SSL_AIR_DENSITY / MAX(0.00001,density));
}

#endif // AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED || AP_SIM_ENABLED

/*
  return geometric altitude difference in meters between current pressure and a
  given base_pressure in Pascal.
*/
float AP_Baro::get_altitude_difference(float base_pressure, float pressure) const
{
#if AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED
    const float alt1 = get_altitude_from_pressure(base_pressure);
    const float alt2 = get_altitude_from_pressure(pressure);
    return alt2 - alt1;
#else
    return get_altitude_difference_simple(base_pressure, pressure);
#endif
}

/*
  return current scale factor that converts from equivalent to true airspeed
  valid for altitudes up to 11km AMSL
  assumes USA 1976 standard atmosphere lapse rate
*/
float AP_Baro::get_EAS2TAS_simple(float altitude, float pressure) const
{
    if (is_zero(pressure)) {
        return 1.0f;
    }

    // only estimate lapse rate for the difference from the ground location
    // provides a more consistent reading then trying to estimate a complete
    // ISA model atmosphere
    float tempK = C_TO_KELVIN(get_ground_temperature()) - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / (pressure / (ISA_GAS_CONSTANT * tempK));
    if (!is_positive(eas2tas_squared)) {
        return 1.0f;
    }
    return sqrtf(eas2tas_squared);
}

/*
  return current scale factor that converts from equivalent to true airspeed
*/
float AP_Baro::_get_EAS2TAS(void) const
{
    const float altitude = get_altitude_AMSL();

#if AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED
    return get_EAS2TAS_extended(altitude);
#else
    // otherwise use function
    return get_EAS2TAS_simple(altitude, get_pressure());
#endif
}

// lookup expected temperature in degrees C for a given altitude. Used for SITL backend
float AP_Baro::get_temperatureC_for_alt_amsl(const float alt_amsl)
{
    float pressure, temp_K;
    get_pressure_temperature_for_alt_amsl(alt_amsl, pressure, temp_K);
    return KELVIN_TO_C(temp_K);
}

// lookup expected pressure in Pa for a given altitude. Used for SITL backend
float AP_Baro::get_pressure_for_alt_amsl(const float alt_amsl)
{
    float pressure, temp_K;
    get_pressure_temperature_for_alt_amsl(alt_amsl, pressure, temp_K);
    return pressure;
}

/*
  return sea level pressure given a current altitude and pressure reading
  this is the pressure p0 such that
    get_altitude_difference(p0, pressure) == altitude
  this function is used during calibration
*/
float AP_Baro::get_sealevel_pressure(float pressure, float altitude) const
{
    const float min_pressure = 0.01;
    const float max_pressure = 1e6;
    float p0 = pressure;
    /*
      use a simple numerical gradient descent method so we don't need
      the inverse function. This typically converges in about 5 steps,
      we limit it to 20 steps to prevent possible high CPU usage
     */
    uint16_t count = 20;
    while (count--) {
        const float delta = 0.1;
        const float err1 = get_altitude_difference(p0, pressure) - altitude;
        const float err2 = get_altitude_difference(p0+delta, pressure) - altitude;
        const float dalt = err2 - err1;
        if (fabsf(err1) < 0.01) {
            break;
        }
        p0 -= err1 * delta / dalt;
        p0 = constrain_float(p0, min_pressure, max_pressure);
    }
    return p0;
}
