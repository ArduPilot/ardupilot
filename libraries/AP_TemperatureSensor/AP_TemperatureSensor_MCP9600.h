/*
 * Copyright (C) 2022  Kraus Hamdani Aerospace Inc. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Tom Pittenger
 */
/*
    Implements I2C driver for Microchip MCP9600 digital thermocouple EMF to Temperature converter
*/

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_MCP9600_ENABLED
    #define AP_TEMPERATURE_SENSOR_MCP9600_ENABLED AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_MCP9600_ENABLED


class AP_TemperatureSensor_MCP9600 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;
public:

    enum class ThermocoupleType : uint8_t {
        K = 0,
        J = 1,
        T = 2,
        N = 3,
        S = 4,
        E = 5,
        B = 6,
        R = 7,
    };

    void init(void) override;

    void update(void) override {};

private:

    void _timer(void);

    bool read_temperature(float &temperature);

    bool set_config(const ThermocoupleType type, const uint8_t filter);
};

#endif // AP_TEMPERATURE_SENSOR_MCP9600_ENABLED
