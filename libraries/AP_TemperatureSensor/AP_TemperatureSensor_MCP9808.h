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
 * I2C driver for Microchip MCP9808 high-accuracy digital temperature sensor.
 *
 * The MCP9808 is a digital temperature sensor with 0.0625°C resolution.
 * It communicates over I2C and provides a 16-bit ambient temperature register.
 */

#pragma once

#include "AP_TemperatureSensor_Backend.h"

#if AP_TEMPERATURE_SENSOR_MCP9808_ENABLED

#ifndef AP_TEMPERATURE_SENSOR_MCP9808_DEFAULT_I2C_ADDR
#define AP_TEMPERATURE_SENSOR_MCP9808_DEFAULT_I2C_ADDR 0x18
#endif


class AP_TemperatureSensor_MCP9808 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

public:

    __INITFUNC__ void init(void) override;


    void update() override {};


private:

    // update the temperature, called at 20Hz
    void _timer(void);


    // read a 16-bit big-endian register
    bool read_registers(uint8_t reg, uint16_t &value) const;


    // write a 16-bit big-endian register
    bool write_register(uint8_t reg, uint16_t value) const;
};


#endif // AP_TEMPERATURE_SENSOR_MCP9808_ENABLED