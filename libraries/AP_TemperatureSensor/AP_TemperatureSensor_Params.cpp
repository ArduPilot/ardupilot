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

#include "AP_TemperatureSensor_Params.h"
#include "AP_TemperatureSensor.h"

#if AP_TEMPERATURE_SENSOR_ENABLED

#ifndef AP_TEMPERATURE_SENSOR_TYPE_DEFAULT
#define AP_TEMPERATURE_SENSOR_TYPE_DEFAULT 0
#endif

#ifndef AP_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT
#define AP_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT 0
#endif

#ifndef AP_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT
#define AP_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT 0
#endif

#ifndef AP_TEMPERATURE_SENSOR_SOURCE_ID_DEFAULT
#define AP_TEMPERATURE_SENSOR_SOURCE_ID_DEFAULT -1
#endif

#ifndef AP_TEMPERATURE_SENSOR_SOURCE_DEFAULT
#define AP_TEMPERATURE_SENSOR_SOURCE_DEFAULT (int8_t)AP_TemperatureSensor::Source::None
#endif

const AP_Param::GroupInfo AP_TemperatureSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Temperature Sensor Type
    // @Description: Enables temperature sensors
    // @Values: 0:Disabled, 1:TSYS01, 2:MCP9600
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_TemperatureSensor_Params, type, AP_TEMPERATURE_SENSOR_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: BUS
    // @DisplayName: Temperature sensor bus
    // @Description: Temperature sensor bus number, typically used to select from multiple I2C buses
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BUS", 2, AP_TemperatureSensor_Params, bus, AP_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT),

    // @Param: ADDR
    // @DisplayName: Temperature sensor address
    // @Description: Temperature sensor address, typically used for I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ADDR", 3, AP_TemperatureSensor_Params, bus_address, AP_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT),

    AP_GROUPEND
};

AP_TemperatureSensor_Params::AP_TemperatureSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_TEMPERATURE_SENSOR_ENABLED
