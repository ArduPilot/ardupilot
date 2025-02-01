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

#include "AP_AirSensor.h"

#if AP_AIRSENSOR_ENABLED

#include <GCS_MAVLink/GCS.h>
#include "AP_AirSensor_Backend.h"
#include "AP_AirSensor_Scripting.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_AirSensor::var_info[] = {
    // 0 is reserved for possible addition of an ENABLED parameter

    // @Group: 1
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[0], "1", 1, AP_AirSensor, AP_AirSensor_Params),

#if AP_AIR_SENSOR_MAX_SENSORS > 1
    // @Group: 2
    // @Path: AP_Proximity_Params.cpp
    // TODO change to the correct number
    AP_SUBGROUPINFO(params[1], "2", 99, AP_AirSensor, AP_AirSensor_Params),
#endif
};
static_assert(AP_AIR_SENSOR_MAX_SENSORS < 2, "Add more parameter groups before adding more sensors!");

AP_AirSensor::AP_AirSensor()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_AirSensor must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void AP_AirSensor::init()
{
  // TODO add param for enabling,
  allocate();
}

void AP_AirSensor::update()
{
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (!valid_instance(i)) {
            continue;
        }
        sensors[i]->update();
    }
}

void AP_AirSensor::allocate()
{
    for (size_t i = 0; i < AP_AIR_SENSOR_MAX_SENSORS; i++) {
        switch ((AP_AirSensor::Type)params[i].type.get()) {
            case AP_AirSensor::Type::NONE:
                // nothing to do
                break;
#if AP_AIRSENSOR_SCRIPTING_ENABLED
            case AP_AirSensor::Type::SCRIPTING:
                sensors[i] = NEW_NOTHROW AP_AirSensor_Scripting(*this, state[i], params[i]);
                break;
#endif
            default:
                break;
                // TODO
        }

        if (sensors[i] && !sensors[i]->init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AirSensor %zu init failed", i + 1);
            delete sensors[i];
            sensors[i] = nullptr;
        }
        if (sensors[i] != nullptr) {
            _num_sensors = i+1;
        }
    }
}

// return air sensor backend for Lua scripting
AP_AirSensor_Backend *AP_AirSensor::get_backend(uint8_t id) const
{
    if (!valid_instance(id)) {
        return nullptr;
    }
    return sensors[id];
}

AP_AirSensor::Type AP_AirSensor::get_type(uint8_t instance) const
{
    if (instance < AP_AIR_SENSOR_MAX_SENSORS) {
        return (Type)((uint8_t)params[instance].type);
    }
    return Type::NONE;
}


// return true if the given instance exists
bool AP_AirSensor::valid_instance(uint8_t i) const
{
    if (i >= AP_AIR_SENSOR_MAX_SENSORS) {
        return false;
    }

    if (sensors[i] == nullptr) {
        return false;
    }
    return (Type)params[i].type.get() != Type::NONE; // TODO
}

AP_AirSensor *AP_AirSensor::_singleton;


#endif // AP_AIRSENSOR_ENABLED
