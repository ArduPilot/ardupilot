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

#pragma once

#include <AP_HAL/AP_HAL.h>

#include <cmath>

#define INHG_TO_PA      3386.383613f
#define METERS_TO_FEET  3.280839895f

namespace ISA_MATH_CONST {

using namespace std;

    const float p0      = 101325.0f;                    // [N/m^2] = [Pa]
    const float p1      = 22632.05545875171f;           // [N/m^2] = [Pa] Calculated @ 11000.0000000000001 [ft]
    const float p2      = 5474.884659730908f;           // [N/m^2] = [Pa] Calculated @ 20000.000000000001  [ft]
    const float p3      = 868.0176477556424f;           // [N/m^2] = [Pa] Calculated @ 32000.000000000001  [ft]

    const float p0hPa   = 1013.25f;                     // [hPa]
    const float p0inHG  = 29.9213f;                     // [inHG] Truncated 29.9213
    const float p0mmHG  = p0inHG * 25.4f;               // [mmHG] w/ 25.4 [mm] = 1.0 [in]

    const float T0      = 288.15f;                      // [K]
    const float T1      = 216.65f;                      // [K]
    const float T2      = 216.65f;                      // [K]

    const float h1      = 36089.238845144355f;          // [ft] = 11000 [m] w/ CftTOm
    const float h2      = 65616.79790026246f;           // [ft] = 20000 [m] w/ CftTOm
    const float h3      = 104986.87664041994f;          // [ft] = 32000 [m] w/ CftTOm

    const float dTdh0   = -0.0019812f;                  // [K/ft] w/ CftTOm
    const float dTdh0SI = -0.0065f;                     // [K/m]

    const float dTdh2   = 0.0003048f;                   // [K/ft] w/ CftTOm
    const float dTdh2SI = 0.001f;                       // [K/m]

    const float CPascalTOPSI   = 1.45037737730209e-04f;
    const float ChPaTOinHG     = p0inHG / p0hPa;
    const float ChPaTOmmHG     = p0mmHG / p0hPa;
    const float CPaTOinHG     = p0inHG / p0;

    const float ClbPft3TOkgPm3 = 16.0184633739601f;     // [lb/ft^3] to [kg/m^3]

    const float CftTOm     = 0.3048f;
    const float CftTOnm    = 1.64578833693305e-04f;

    const float CnmTOm     = 1852.0f;

    const float CftPsTOkn  = CftTOnm * 3600.0f;
    const float CftPsTOmph = 3600.0f / 5280.0f;
    const float CftPsTOkph = CftTOm * 3600.0f / 1000.0f;

    const float CmPsTOkn   = 3600.0f / CnmTOm;

    const float CknTOftPs  = 1.0f / (CftTOnm * 3600.0f);

    const float CRGasSI    = 287.053f;                  // [m^2/(s^2*K)] = [J/(kg*K)]

    const float CgSI       = 9.80665f;                  // [m/s^2]

    const float CgRGas     = (CgSI * CftTOm) / CRGasSI;
    const float CgRGasSI   = CgSI / CRGasSI;

    const float CGamma     = 1.4f;    // [-]
    const float CGammaRGas = (CGamma * CRGasSI) / (CftTOm * CftTOm);    // [ft^2/(s^2*K)]

    const float CaSLSI         = sqrtf(CGamma * CRGasSI * T0);
    const float CPressureSLSI  = 101325.0f;                             // [Pa] = [N/m^2]
    const float CaSLNU         = CaSLSI * CmPsTOkn;                     // [kts] Nautical Unit
    const float CRhoSLSI       = 1.225f;                                // [kg/m^3]

    const float CKelvinTOCelsius   = 273.15f;
    const float CKelvinTORankine   = 1.8f;

    const float CCelsiusTOFahrenheitLinear       =  32.0f;
    const float CCelsiusTOFahrenheitProportional =  1.8f;

    typedef enum ALTITUDE_UNIT {
        INCH = 0,
        FEET,
        MILLIMETER,
        CENTIMETER,
        METER
    } altitude_unit_t;

    typedef enum PRESSURE_UNIT {
        PASCAL = 0,
        HECTO_PASCAL,
        INCH_MERCURY,
        MILLIMETER_MERCURY
    } pressure_unit_t;

};
