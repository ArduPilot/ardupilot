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
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_TEMPERATURE_SENSOR_ENABLED
#define AP_TEMPERATURE_SENSOR_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
#include "AP_TemperatureSensor_Params.h"

// maximum number of Temperature Sensors
#ifndef AP_TEMPERATURE_SENSOR_MAX_INSTANCES
#define AP_TEMPERATURE_SENSOR_MAX_INSTANCES       3
#endif

// first sensor is always the primary sensor
#define AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE            0

// declare backend class
class AP_TemperatureSensor_Backend;
class AP_TemperatureSensor_TSYS01;

class AP_TemperatureSensor
{
    friend class AP_TemperatureSensor_Backend;
    friend class AP_TemperatureSensor_TSYS01;

public:

    // temperature sensor types
    enum class Type {
        NONE                       = 0,
        TSYS01                     = 1,
    };

    // Constructor
    AP_TemperatureSensor();

    CLASS_NO_COPY(AP_TemperatureSensor);

    static AP_TemperatureSensor *get_singleton() { return _singleton; }

    // The TemperatureSensor_State structure is filled in by the backend driver
    struct TemperatureSensor_State {
        const struct AP_Param::GroupInfo *var_info;
        uint32_t    last_time_micros;          // time when the sensor was last read in microseconds
        float       temperature;               // temperature (deg C)
        bool        healthy;                   // temperature sensor is communicating correctly
        uint8_t     instance;                  // instance number
    };

    // Return the number of temperature sensors instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available temperature sensors
    void init();

    // Update the temperature for all temperature sensors
    void update();

    // get_type - returns temperature sensor type
    Type get_type() const { return get_type(AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE); }
    Type get_type(uint8_t instance) const;

    // temperature
    bool temperature(float &temp) const { return temperature(temp, AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE); }
    bool temperature(float &temp, const uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // parameters
    AP_TemperatureSensor_Params _params[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];

private:
    static AP_TemperatureSensor *_singleton;

    TemperatureSensor_State _state[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];
    AP_TemperatureSensor_Backend *drivers[AP_TEMPERATURE_SENSOR_MAX_INSTANCES];

    uint8_t     _num_instances;         // number of temperature sensors

    // Parameters
    AP_Int8 _log_flag;                  // log_flag: true if we should log all sensors data
};

namespace AP {
    AP_TemperatureSensor &temperature_sensor();
};

#endif // AP_TEMPERATURE_SENSOR_ENABLED
