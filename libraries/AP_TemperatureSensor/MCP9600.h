/*
 * Copyright (C) 2020  Kraus Hamdani Aerospace Inc. All rights reserved.
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

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_TEMP_SENSOR_MCP9600_ENABLE
    #define HAL_TEMP_SENSOR_MCP9600_ENABLE (!HAL_MINIMIZE_FEATURES || BOARD_FLASH_SIZE > 1024 || defined(HAL_PERIPH_ENABLE_TEMP_SENSOR_MCP9600))
#endif

#if HAL_TEMP_SENSOR_MCP9600_ENABLE

#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>

class MCP9600 {
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
    } ;

    enum class ID_Src : uint8_t {
        ESC = 0,
        Battery = 1,
    };

    MCP9600() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    bool init(uint8_t bus, uint8_t address);
    bool init() { return init(_bus, _address); }

    // get temperature in degrees C
     float get_temperature(void) {
        WITH_SEMAPHORE(sem);
        return _temperature;
    }

    bool healthy(void) const { // do we have a valid temperature reading?
        return _healthy;
    }
    uint8_t get_bus() const { return _bus; }
    uint8_t get_address() const { return _address; }
    ID_Src get_id_src() const { return _id_src; }
    int32_t get_id() const { return _id; }

    bool set_config(const ThermocoupleType type, const uint8_t filter);
    bool set_Type(const ThermocoupleType type) { return set_config(type, _filter); }
    bool set_Filter(const uint8_t filter) { return set_config(_thermoType, filter); }

    static const struct AP_Param::GroupInfo var_info[];

private:

    bool read_temperature(float &temperature);
    void _timer(void); // update the temperature, called at 20Hz

    HAL_Semaphore sem;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint32_t _last_reading_ms;
    float _temperature; // degrees C
    bool _healthy; // we have a valid temperature reading to report


    int8_t _filter;
    AP_Int8 _filter_param;
    ThermocoupleType _thermoType;
    AP_Enum<ThermocoupleType> _thermoType_param;
    AP_Int8 _bus;
    AP_Int8 _address;
    AP_Enum<ID_Src> _id_src;
    AP_Int32 _id;
};

#endif // HAL_TEMP_SENSOR_MCP9600_ENABLE
