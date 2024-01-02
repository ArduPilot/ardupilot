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

#include "AP_AngleSensor.h"

#if AP_ANGLESENSOR_ENABLED

#include "AP_AngleSensor_AS5048B.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_AngleSensor::var_info[] = {
    // @Group: _
    // @Path: AP_AngleSensor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "_", 0, AP_AngleSensor, AP_AngleSensor_Params),

    #if ANGLE_SENSOR_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_AngleSensor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 1, AP_AngleSensor, AP_AngleSensor_Params),
    #endif
    AP_GROUPEND
};

AP_AngleSensor::AP_AngleSensor(void)
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_AngleSensor class.
void AP_AngleSensor::init(void)
{
    if (_num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<ANGLE_SENSOR_MAX_INSTANCES; i++) {
        switch ((AngleSensor_Type)_params[i]._type.get()) {

        case ANGLESENSOR_TYPE_AS5048B:
            drivers[i] = new AP_AngleSensor_AS5048B(*this, i, state[i]);
            break;
            
        case ANGLESENSOR_TYPE_NONE:
            break;
        }

        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            _num_instances = i+1;  // num_instances is a high-water-mark
        }
    }
}

// update Angle Sensor state for all instances. This should be called by main loop
void AP_AngleSensor::update(void)
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != nullptr && _params[i]._type != ANGLESENSOR_TYPE_NONE) {
            drivers[i]->update();
        }
    }
    Log_Write();
}

// log angle sensor information
void AP_AngleSensor::Log_Write() const
{
    // return immediately if no angle sensors are enabled
    if (!enabled(0) && !enabled(1)) {
        return;
    }

    struct log_AngleSensor pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ANGLESENSOR_MSG),
        time_us     : AP_HAL::micros64(),
        angle_0     : (float)get_angle_radians(0),
        quality_0   : (uint8_t)get_signal_quality(0),
        angle_1     : (float)get_angle_radians(1),
        quality_1   : (uint8_t)get_signal_quality(1),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// check if an instance is healthy
bool AP_AngleSensor::healthy(uint8_t instance) const
{
    return enabled(instance) && get_signal_quality(instance) > 0;
}

// check if an instance is activated
bool AP_AngleSensor::enabled(uint8_t instance) const
{
    if (instance >= _num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    return _params[instance]._type != ANGLESENSOR_TYPE_NONE; 
}

uint8_t AP_AngleSensor::get_type(uint8_t instance) const
{
    if (instance >= _num_instances) {
        return 0;
    }
    // if no sensor type is selected, the sensor is not activated.
    return (uint8_t) _params[instance]._type;
}

// //get the most recent angle measurement of the sensor (in radians)
float AP_AngleSensor::get_angle_radians(uint8_t instance) const
{
    
    // for invalid instances return zero
    if (instance >= ANGLE_SENSOR_MAX_INSTANCES) {
        return 0;
    }

    float raw_angle = state[instance].angle_radians;
    float offset_rad = radians(_params[instance]._offset);
    int8_t direction = _params[instance]._direction;

    return wrap_2PI(raw_angle*(float)direction + offset_rad);
}

// get the signal quality for a sensor
uint8_t AP_AngleSensor::get_signal_quality(uint8_t instance) const
{
   // for invalid instances return zero
    if (instance >= ANGLE_SENSOR_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].quality;
}

// get the system time (in milliseconds) of the last update
uint32_t AP_AngleSensor::get_last_reading_ms(uint8_t instance) const
{
    // for invalid instances return zero
    if (instance >= ANGLE_SENSOR_MAX_INSTANCES) {
        return 0;
    }
    return state[instance].last_reading_ms;
}

// singleton instance
AP_AngleSensor *AP_AngleSensor::_singleton;

namespace AP {

AP_AngleSensor *ANGLE_SENSOR()
{
    return AP_AngleSensor::get_singleton();
}

}

#endif  //AP_ANGLESENSOR_ENABLED

