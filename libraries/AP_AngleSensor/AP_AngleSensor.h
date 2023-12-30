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

#include "AP_AngleSensor_config.h"

#if AP_ANGLESENSOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_AngleSensor_Params.h"

#define ANGLE_SENSOR_TYPE_NONE 0

class AP_AngleSensor_Backend; 
class AP_AngleSensor_AS5048B;
 
class AP_AngleSensor
{
public:
    friend class AP_AngleSensor_Backend;
    friend class AP_AngleSensor_AS5048B;

    AP_AngleSensor(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AngleSensor);

    // get singleton instance
    static AP_AngleSensor *get_singleton() {
        return _singleton;
    }

    // AngleSensor driver types
    enum AngleSensor_Type : uint8_t {
        ANGLESENSOR_TYPE_NONE          =   ANGLE_SENSOR_TYPE_NONE,
        ANGLESENSOR_TYPE_AS5048B       =   1,
    };

    // The AngleSensor_State structure is filled in by the backend driver
    struct AngleSensor_State {
        uint8_t                instance;        // the instance number of this Angle Sensor
        float                  angle_radians;   // current angle measured by the sensor, in radians
        uint8_t                quality;         // sensor quality as a percent
        uint32_t               last_reading_ms; // time of last reading
    };

    // detect and initialise any available angle sensor devices
    void init(void);

    // update state of all sensors. Should be called from main loop
    void update(void);

    // log data to logger
    void Log_Write() const;

    // return the number of angle sensor sensor instances
    uint8_t num_sensors(void) const { return _num_instances; }

    // return true if healthy
    bool healthy(uint8_t instance) const;

    // return true if the instance is enabled
    bool enabled(uint8_t instance) const;

    // get the type of sensor for a specific instance
    uint8_t get_type(uint8_t instance) const;

    //get the most recent angle measurement of the sensor
    float get_angle_radians(uint8_t instance) const;

    // get the signal quality for a sensor (0 = extremely poor quality, 100 = extremely good quality)
    uint8_t get_signal_quality(uint8_t instance) const;

    // get the system time (in milliseconds) of the last update
    uint32_t get_last_reading_ms(uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    /// parameters
    AP_AngleSensor_Params _params[ANGLE_SENSOR_MAX_INSTANCES];


private:

    AngleSensor_State state[ANGLE_SENSOR_MAX_INSTANCES];
    AP_AngleSensor_Backend *drivers[ANGLE_SENSOR_MAX_INSTANCES];
    uint8_t _num_instances;

    static AP_AngleSensor *_singleton;
};

namespace AP {
    AP_AngleSensor *ANGLE_SENSOR();
}

#endif  // AP_ANGLESENSOR_ENABLED
