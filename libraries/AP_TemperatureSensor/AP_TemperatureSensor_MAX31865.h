/*
 * Copyright (C) 2023 Kraus Hamdani Aerospace Inc. All rights reserved.
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
    Implements SPI driver for MAX31865 digital RTD Temperature converter
*/

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#if AP_TEMPERATURE_SENSOR_MAX31865_ENABLED

class AP_TemperatureSensor_MAX31865 : public AP_TemperatureSensor_Backend {
public:
    AP_TemperatureSensor_MAX31865(AP_TemperatureSensor &front, AP_TemperatureSensor::TemperatureSensor_State &state, AP_TemperatureSensor_Params &params);

    __INITFUNC__ void init(void) override;

    void update(void) override {};

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Vbias = ON (must be on for automatic mode)
    // Conversion mode: Auto
    // Fault Status: 1set 1 to Clear all latched faults
    uint8_t config_register = 0b11000010;

    AP_Float nominal_resistance;
    AP_Float reference_resistance;

    // Convert raw value in to temperature in deg celsius
    float calculate_temperature(const uint16_t raw) const;

    void thread_tick(void);
};

#endif // AP_TEMPERATURE_SENSOR_MAX31865_ENABLED
