/*
   Aeronautics Atmosphere Calculations.
   Copyright (c) 2019 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
   UPDATE 05072019:

   The improvements about QNH, QFE, Namespace ISA_MATH_CONST Definitions constants and
   Altitude_difference calculations was based on the AviationCalculator 1.8.2 05/10/2017
   revision application from Ph.D. Aerospace Engineering, Joachim K. Hochwarth (GE Principal
   Engineer/System Architect), published on this URL:

   http://www.hochwarth.com/misc/AviationCalculator.html
   http://www.hochwarth.com/script/AviationCalculator.js

   Thanks Hochwarth!

   These functions was tested on the Encoder & Altimeter Benchtest Hardware and Application
   development in cooperation and support with Fenix Aeronautics Academy under the supervision
   and technical support with the FAA Licensed Fixed and Rotor Wing Aircraft Engineer
   Maintenance Service, Rudy Averanga <raveranga@fenixaeronautica.com>.

   Thanks a lot Rudy!
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include "UR_Atmosphere.h"

extern const AP_HAL::HAL& hal;
bool UR_Atmosphere::_initialized = false;
UR_Atmosphere *UR_Atmosphere::_instance = nullptr;

UR_Atmosphere::UR_Atmosphere()
{
    _barometer = AP_Baro::get_singleton();
}

void UR_Atmosphere::init(void)
{
    if (_initialized) {
        return;
    }

    _initialized = true;
    _instance = this;

    if (!_barometer) {
        _barometer = new AP_Baro;
        _barometer->init();
    }
}

void UR_Atmosphere::update(uint8_t sensor)
{
    if (!atm_status.updated) {
        atm_status.updated = true;
        atm_status.altitude_amsl = calculate_altitude_difference(press0, _barometer->sensors[sensor].pressure, _barometer->sensors[sensor].temperature);
        atm_status.qnh_pressure = calculate_qnh(atm_status.altitude_amsl, (_barometer->sensors[sensor].pressure / 100), _barometer->sensors[sensor].temperature, ALTITUDE_UNIT::METER);
        atm_status.qfe_pressure = calculate_qfe(atm_status.altitude_amsl, atm_status.qnh_pressure, _barometer->sensors[sensor].temperature, ALTITUDE_UNIT::METER) * 100;
    }
}

// Get the calculation altitude difference based on th Layer 0 > Troposphere.
float UR_Atmosphere::calculate_altitude_difference(float base_pressure, float pressure, float temp, altitude_unit_t alt_diff_unit) const
{
    const float tempcorrected  = Temp0 + temp - 15;
    float unit_result = 1;

    switch (alt_diff_unit) {
    case FEET:
        break;
    case METER:
        unit_result = CftTOm;
        break;
    default:
        break;
    }

    const float result = unit_result * (tempcorrected * (powf((base_pressure / pressure), (dTdh0SI / CgRGasSI)) - 1.0f) / dTdh0);

    return result;
}

// Calculate QNH by hPa and feet by default.
// [1] Meteorology Specification A2669 Version 4.00 Page 100
float UR_Atmosphere::calculate_qnh(float alt_qnh, float pressure_qnh, float temp, altitude_unit_t alt_qnh_unit) const
{
    uint8_t pQNHUnit = PRESSURE_UNIT::HECTO_PASCAL;
    uint8_t resultQNHUnit = PRESSURE_UNIT::HECTO_PASCAL;
    float hQFE = 0;
    float pQFE = 0;

    switch (alt_qnh_unit) {
    case FEET:                         // [ft] feet
        hQFE = alt_qnh * CftTOm;
        break;
    case METER:                         // [m]  meters
        hQFE = alt_qnh;
        break;
    default:
        break;
    }

    switch (pQNHUnit) {
    case HECTO_PASCAL:                      // [hPa]    hectoPascal
        pQFE = pressure_qnh;
        break;
    case INCH_MERCURY:                      // [inHg]   inches mercury
        pQFE = pressure_qnh / ChPaTOinHG;
        break;
    case MILLIMETER_MERCURY:                // [mmHg]   millimiters mercury
        pQFE = pressure_qnh / ChPaTOmmHG;
        break;
    default:
        break;
    }

    // Calculation(s)
    pressure_qnh = p0hPa * powf(powf((pQFE / p0hPa), (-1 * (CRGasSI * dTdh0SI)) / CgSI) - (hQFE * dTdh0SI) / (Temp0 + temp - 15), (-1 * CgSI) / (CRGasSI * dTdh0SI));

    // Unit Conversion(s)
    switch (resultQNHUnit) {
    case HECTO_PASCAL:                              // [hPa]    hectoPascal
        break;
    case INCH_MERCURY:                              // [inHg]   inches mercury
        pressure_qnh = pressure_qnh * ChPaTOinHG;
        break;
    case MILLIMETER_MERCURY:                        // [mmHg]   millimiters mercury
        pressure_qnh = pressure_qnh * ChPaTOmmHG;
        break;
    default:
        break;
    }

    const float resultQNH = pressure_qnh;
    return resultQNH;
}

// Calculate QFE by hPa and feet by default.
// [1] Meteorology Specification A2669 Version 4.00 Page 100
float UR_Atmosphere::calculate_qfe(float alt_qfe, float pressure_qfe, float temp, altitude_unit_t alt_qfe_unit) const
{
    uint8_t pQFEUnit = PRESSURE_UNIT::HECTO_PASCAL;
    uint8_t resultQFEUnit = PRESSURE_UNIT::HECTO_PASCAL;
    float hQNH = 0;
    float pQNH = 0;

    switch (alt_qfe_unit) {
    case FEET:                                 // [ft] feet
        hQNH = alt_qfe * CftTOm;
        break;
    case METER:                                 // [m]  meters
        hQNH = alt_qfe;
        break;
    default:
        break;
    }

    switch (pQFEUnit) {
    case HECTO_PASCAL:                      // [hPa]    hectoPascal
        pQNH = pressure_qfe;
        break;
    case INCH_MERCURY:                      // [inHg]   inches mercury
        pQNH = pressure_qfe / ChPaTOinHG;
        break;
    case MILLIMETER_MERCURY:                // [mmHg]   millimiters mercury
        pQNH = pressure_qfe / ChPaTOmmHG;
        break;
    default:
        break;
    }

    // Calculations
    alt_qfe = p0hPa * powf(powf((pQNH / p0hPa), (-1 * (CRGasSI * dTdh0SI)) / CgSI) + ((hQNH * dTdh0SI) / (Temp0 + temp - 15)), (-1 * CgSI) / (CRGasSI * dTdh0SI));

    // Unit Conversions
    switch (resultQFEUnit) {
    case HECTO_PASCAL:                      // [hPa]    hectoPascal
        break;
    case INCH_MERCURY:                      // [inHg]   inches mercury
        alt_qfe = alt_qfe * ChPaTOinHG;
        break;
    case MILLIMETER_MERCURY:                // [mmHg]   millimiters mercury
        alt_qfe = alt_qfe * ChPaTOmmHG;
        break;
    default:
        break;
    }

    const float resultQFE = alt_qfe;
    return resultQFE;
}

void UR_Atmosphere::consume_updated(void)
{
    atm_status.updated = false;
}

float UR_Atmosphere::get_altitude_amsl() const
{
    return atm_status.altitude_amsl;
}

float UR_Atmosphere::get_qnh() const
{
    return atm_status.qnh_pressure;
}
