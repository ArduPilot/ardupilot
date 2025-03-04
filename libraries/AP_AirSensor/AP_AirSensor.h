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

#include "AP_AirSensor_config.h"

#if AP_AIRSENSOR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include "AP_AirSensor_Params.h"
#include <AP_Param/AP_Param.h>

class AP_AirSensor_Backend;

/// @class AP_AirSensor
/// Air data sensing class that provides a common interface for 1D AP_Airspeed probes
/// 2D WindVane sensors, and other 1D-3D capable air data sensor interfaces.
class AP_AirSensor
{
public:
    friend class AP_AirSensor_Backend;  
    static const struct AP_Param::GroupInfo var_info[];

    AP_AirSensor();

    // return the number of air sensor backends
    uint8_t num_sensors() const { return _num_sensors; }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AirSensor);

    enum class Type {
        NONE,
#if AP_AIRSENSOR_SCRIPTING_ENABLED
        SCRIPTING
#endif
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        Good
    };

    // The State structure is filled in by the backend driver
    struct State {
        uint8_t instance; // the instance number of this proximity sensor
        Status status; // sensor status

        // const struct AP_Param::GroupInfo *var_info; // stores extra parameter information for the sensor (if it exists)
    };

    // detect and initialise any available air sensors
    void init();
    // update state of all air sensors. Should be called at high rate from main loop
    void update();
    static AP_AirSensor *get_singleton(void) { return _singleton; };

    // return backend object for Lua scripting
    AP_AirSensor_Backend *get_backend(uint8_t id) const;

    // Returns status of first good sensor. If no good sensor found, returns status of last instance sensor 
    Status get_status() const;

    // return sensor type of a given instance
    Type get_type(uint8_t instance) const;

private:

    void allocate();

    // return true if the given instance exists
    bool valid_instance(uint8_t i) const;

    static AP_AirSensor *_singleton;
    State state[AP_AIR_SENSOR_MAX_SENSORS];
    AP_AirSensor_Params params[AP_AIR_SENSOR_MAX_SENSORS];
    AP_AirSensor_Backend *sensors[AP_AIR_SENSOR_MAX_SENSORS];
    uint8_t _num_sensors;
};

#endif // AP_AIRSENSOR_ENABLED
