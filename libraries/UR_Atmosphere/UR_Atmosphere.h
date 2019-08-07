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
#include <AP_Baro/AP_Baro.h>
#include "UR_Atmosphere/atmosphere_definitions.h"

using namespace ISA_MATH_CONST;

class AP_Baro;

class UR_Atmosphere
{
public:
    UR_Atmosphere();
    ~UR_Atmosphere(void) {};

    void init(void);
    void update(uint8_t sensor);

    float calculate_altitude_difference(float base_pressure, float pressure, float temp, altitude_unit_t alt_diff_unit = METER) const;
    float calculate_qnh(float alt_qnh, float pressure_qnh, float temp, altitude_unit_t alt_qnh_unit);
    float calculate_qfe(float alt_qfe, float pressure_qfe, float temp, altitude_unit_t alt_qfe_unit);

    static bool initialized(void) {
        return _initialized;
    }

    void consume_updated(void);

    float get_altitude_amsl();
    float get_qnh();

    static UR_Atmosphere *get_instance() {
        return _instance;
    }

private:

    struct atm_status_s {
        float altitude_amsl;
        float qfe_pressure;
        float qnh_pressure;
        bool updated;
    } atm_status;

    AP_Baro *_barometer;
    static UR_Atmosphere *_instance;

    static bool _initialized;

};
