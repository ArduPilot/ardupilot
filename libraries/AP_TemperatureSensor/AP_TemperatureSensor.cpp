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

#include "AP_TemperatureSensor.h"

#if AP_TEMPERATURE_SENSOR_ENABLED
#include "AP_TemperatureSensor_TSYS01.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

AP_TemperatureSensor *AP_TemperatureSensor::_singleton;

const AP_Param::GroupInfo AP_TemperatureSensor::var_info[] = {

    // SKIP INDEX 0

    // @Param: _LOG
    // @DisplayName: Logging
    // @Description: Enables temperature sensor logging
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_LOG", 1, AP_TemperatureSensor, _log_flag, 0),

    // SKIP Index 2-9 to be for parameters that apply to every sensor

    // @Group: 1_
    // @Path: AP_TemperatureSensor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 10, AP_TemperatureSensor, AP_TemperatureSensor_Params),

#if AP_TEMPERATURE_SENSOR_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_TemperatureSensor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 11, AP_TemperatureSensor, AP_TemperatureSensor_Params),
#endif

#if AP_TEMPERATURE_SENSOR_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_TemperatureSensor_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 12, AP_TemperatureSensor, AP_TemperatureSensor_Params),
#endif

    AP_GROUPEND
};

// Default Constructor
AP_TemperatureSensor::AP_TemperatureSensor()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_TemperatureSensor must be singleton");
    }
    _singleton = this;
}

// init - instantiate the temperature sensors
void AP_TemperatureSensor::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

 // For Sub set the Default: Type to TSYS01 and I2C_ADDR of 0x77
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
    AP_Param::set_default_by_name("TEMP1_TYPE", (int8_t)AP_TemperatureSensor::Type::TSYS01);
    AP_Param::set_default_by_name("TEMP1_ADDR", TSYS01_ADDR_CSB0);
#endif

    // create each instance
    for (uint8_t instance = 0; instance < AP_TEMPERATURE_SENSOR_MAX_INSTANCES; instance++) {

        switch (get_type(instance)) {
#if AP_TEMPERATURE_SENSOR_TSYS01_ENABLE
            case Type::TSYS01:
                drivers[instance] = new AP_TemperatureSensor_TSYS01(*this, _state[instance], _params[instance]);
                break;
#endif
            case Type::NONE:
            default:
                break;
        }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            drivers[instance]->init();
            // _num_instances is actually the index for looping over instances
            // the user may have TEMP_TYPE=0 and TEMP2_TYPE=7, in which case
            // there will be a gap, but as we always check for drivers[instances] being nullptr
            // this is safe
            _num_instances = instance + 1;
        }
    }
}

// update: - For all active instances update temperature and log TEMP
void AP_TemperatureSensor::update()
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != nullptr && get_type(i) != Type::NONE) {
            drivers[i]->update();

#if HAL_LOGGING_ENABLED
            const AP_Logger *logger = AP_Logger::get_singleton();
            if (logger != nullptr && _log_flag) {
                const uint64_t time_us = AP_HAL::micros64();
                drivers[i]->Log_Write_TEMP(time_us);
            }
#endif
        }
    }
}

AP_TemperatureSensor::Type AP_TemperatureSensor::get_type(uint8_t instance) const
{
    if (instance >= AP_TEMPERATURE_SENSOR_MAX_INSTANCES) {
        return Type::NONE;
    }
    return (Type)_params[instance]._type.get();
}

// returns true if there is a temperature reading
bool AP_TemperatureSensor::temperature(float &temp, const uint8_t instance) const
{
    if (instance >= AP_TEMPERATURE_SENSOR_MAX_INSTANCES || drivers[instance] == nullptr) {
        return false;
    }

    temp = _state[instance].temperature;

    return drivers[instance]->healthy();
}

namespace AP {

AP_TemperatureSensor &temperature_sensor()
{
    return *AP_TemperatureSensor::get_singleton();
}

};

#endif // AP_TEMPERATURE_SENSOR_ENABLED
