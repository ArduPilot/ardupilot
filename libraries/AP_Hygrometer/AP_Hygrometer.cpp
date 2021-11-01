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

#include "AP_Hygrometer.h"

#if HAL_HYGROMETER_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Hygrometer_Backend.h"
#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_Hygrometer_UAVCAN.h"
#endif

#define HYGRO_DEFAULT_TYPE 0

// table of user settable parameters
const AP_Param::GroupInfo AP_Hygrometer::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Hygrometer type
    // @Description: Type of hygrometer sensor
    // @Values: 0:None,1:UAVCAN
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_Hygrometer, param[0].type, HYGRO_DEFAULT_TYPE, AP_PARAM_FLAG_ENABLE),


#if HYGROMETER_MAX_SENSORS > 1
    // @Param: _PRIMARY
    // @DisplayName: Primary hygrometer sensor
    // @Description: This selects which hygrometer sensor will be the primary if multiple sensors are found
    // @Values: 0:FirstSensor,1:2ndSensor
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 2, AP_Hygrometer, primary_sensor, 0),
#endif

#if HYGROMETER_MAX_SENSORS > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Hygrometer type
    // @Description: Type of 2nd hygrometer sensor
    // @Values: 0:None,1:UAVCAN
    // @User: Standard
    AP_GROUPINFO_FLAGS("2_TYPE", 3, AP_Hygrometer, param[1].type, 0, AP_PARAM_FLAG_ENABLE),


#endif // HYGROMETER_MAX_SENSORS

    AP_GROUPEND
};

void AP_Hygrometer::init(void)
{
    for (uint8_t i=0; i<HYGROMETER_MAX_SENSORS; i++) {
         switch ((enum Type)param[i].type.get()) {
             case Type::NONE:
                // nothing to do
                break;
             case Type::UAVCAN:
#if HAL_ENABLE_LIBUAVCAN_DRIVERS    
    sensor[i] = AP_Hygrometer_UAVCAN::probe(*this, i, state[i], param[i]);
#endif
                break;
         }
        if (sensor[i] && !sensor[i]->init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hygrometer %u init failed", i + 1);
            delete sensor[i];
            sensor[i] = nullptr;
        }
        if (sensor[i] != nullptr) {
            num_sensors = i+1;
        }
    }
}

AP_Hygrometer::AP_Hygrometer()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Hygrometer must be singleton");
    }
    _singleton = this;
}

bool AP_Hygrometer::get_humidity(uint8_t i, float &humidity)
{
    if (!enabled(i)) {
        return false;
    }
    if (sensor[i]) {
        return sensor[i]->get_humidity(humidity);
    }

    return false;
}

bool AP_Hygrometer::get_temperature(uint8_t i, float &temperature)
{
    if (!enabled(i)) {
        return false;

    }
    if (sensor[i]) {
        return sensor[i]->get_temperature(temperature);
    }

    return false;
}

bool AP_Hygrometer::get_id(uint8_t i, uint8_t &id)
{
    if (!enabled(i)) {
        return false;
    }
    if (sensor[i]) {
        return sensor[i]->get_id(id);
    }
    return false;
}

inline void AP_Hygrometer::Log_HYGR()
{
    const uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<HYGROMETER_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }
        float temperature,humidity;
        if (!get_temperature(i, temperature)) {
            temperature = 0;
        }
        if (!get_humidity(i, humidity)) {
            humidity = 0;
        }

        state[i].temperature = temperature;
        state[i].humidity = humidity;

        const struct log_HYGRO pkt{
            LOG_PACKET_HEADER_INIT(LOG_HYGRO_MSG),
            time_us       : now,
            instance      : i,
            temperature   : state[i].temperature,
            humidity      : state[i].humidity,
            primary       : get_primary()
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

//update the log
void AP_Hygrometer::update()
{
#if HAL_LOGGING_ENABLED
        Log_HYGR();
#endif
}

AP_Hygrometer_Backend *AP_Hygrometer::get_backend(uint8_t id) const 
{
    if (id >= HYGROMETER_MAX_SENSORS) {
        return nullptr;
    }
    if (sensor[id] == nullptr) {
        return nullptr;
    }
    if (sensor[id]->type() == Type::NONE) {
        return nullptr;
    }

    return sensor[id];
};

// singleton instance
AP_Hygrometer *AP_Hygrometer::_singleton;

namespace AP {

AP_Hygrometer *hygrometer()
{
    return AP_Hygrometer::get_singleton();
}

};

#endif // HAL_HYGROMETER_ENABLED