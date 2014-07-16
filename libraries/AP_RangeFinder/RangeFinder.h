// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#ifndef __RANGEFINDER_H__
#define __RANGEFINDER_H__

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Param.h>

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 2

class AP_RangeFinder_Backend; 
 
class RangeFinder
{
public:
    RangeFinder(void) :
    num_instances(0)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4
    };

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };


    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint8_t                instance;    // the instance number of this RangeFinder
        uint16_t               distance_cm; // distance: in cm
        uint16_t               voltage_mv;  // voltage in millivolts,
                                            // if applicable, otherwise 0
        bool                   healthy;     // sensor is communicating correctly
    };

    // parameters for each instance
    AP_Int8  _type[RANGEFINDER_MAX_INSTANCES];
    AP_Int8  _pin[RANGEFINDER_MAX_INSTANCES];
    AP_Int8  _ratiometric[RANGEFINDER_MAX_INSTANCES];
    AP_Int8  _stop_pin[RANGEFINDER_MAX_INSTANCES];
    AP_Int16 _settle_time_ms[RANGEFINDER_MAX_INSTANCES];
    AP_Float _scaling[RANGEFINDER_MAX_INSTANCES];
    AP_Float _offset[RANGEFINDER_MAX_INSTANCES];
    AP_Int8  _function[RANGEFINDER_MAX_INSTANCES];
    AP_Int16 _min_distance_cm[RANGEFINDER_MAX_INSTANCES];
    AP_Int16 _max_distance_cm[RANGEFINDER_MAX_INSTANCES];

    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available rangefinders
    void init(void);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);
    
#define _RangeFinder_STATE(instance) state[instance]

    uint16_t distance_cm(uint8_t instance) const {
        return _RangeFinder_STATE(instance).distance_cm;
    }
    uint16_t distance_cm() const {
        return distance_cm(primary_instance);
    }

    uint16_t voltage_mv(uint8_t instance) const {
        return _RangeFinder_STATE(instance).voltage_mv;
    }
    uint16_t voltage_mv() const {
        return voltage_mv(primary_instance);
    }

    int16_t max_distance_cm(uint8_t instance) const {
        return _max_distance_cm[instance];
    }
    int16_t max_distance_cm() const {
        return max_distance_cm(primary_instance);
    }

    int16_t min_distance_cm(uint8_t instance) const {
        return _min_distance_cm[instance];
    }
    int16_t min_distance_cm() const {
        return min_distance_cm(primary_instance);
    }
    
    bool healthy(uint8_t instance) const {
        return instance < num_instances && _RangeFinder_STATE(instance).healthy;
    }
    bool healthy() const {
        return healthy(primary_instance);
    }
    
private:
    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t primary_instance:2;
    uint8_t num_instances:2;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  
};
#endif // __RANGEFINDER_H__
