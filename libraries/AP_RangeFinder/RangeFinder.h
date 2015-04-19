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
#include <AP_Math.h>

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 2

class AP_RangeFinder_Backend; 
 
class RangeFinder
{
public:
    friend class AP_RangeFinder_Backend;
    RangeFinder(void) :
    primary_instance(0),
    num_instances(0),
    estimated_terrain_height(0)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4,
        RangeFinder_TYPE_PX4_PWM= 5
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
    AP_Int16 _powersave_range;

    AP_Int8 _mix_enable; // effect and r_a, r_b needed for the mixing.
    float effect[2];
    uint8_t r_a, r_b;

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

    inline uint16_t distance_cm(uint8_t instance) const {
        return _RangeFinder_STATE(instance).distance_cm;
    }
    uint16_t distance_cm() {
	if(!_mix_enable) {
            return distance_cm(primary_instance);
        } else {
            // We need to know which one of two range finders is upper (A) and lower (B)
            if(max_distance_cm(0) > max_distance_cm(1))
            {
                r_a = 0;
                r_b = 1;
            } else
            {
                r_a = 1;
                r_b = 0;
            }

            // Calculate overlapping distance
            float overlapp = max_distance_cm(r_b) - min_distance_cm(r_a);

            // The finders have more effect on the result as their values are approaching inside their range
            // and have less effect outside. We compare this only inside overlapping region.
            effect[r_a] = ((float) (distance_cm(r_a) - min_distance_cm(r_a))) / overlapp;
            effect[r_b] = ((float) (max_distance_cm(r_b) - distance_cm(r_b))) / overlapp;

            // The sum of effects may not be equal to 100%. We need to scale them to match 100% in total.
            float eff_k = 100.0f / (effect[r_a] + effect[r_b]);
            effect[r_a] *= eff_k;
            effect[r_b] *= eff_k;

            // New range will be combined distance using scale of effect.
            return (((float) distance_cm(r_a)) * effect[r_a] + ((float) distance_cm(r_b)) * effect[r_b]);
        }
    }

    inline uint16_t voltage_mv(uint8_t instance) const {
        return _RangeFinder_STATE(instance).voltage_mv;
    }
    uint16_t voltage_mv() const {
        if(!_mix_enable) {
            return voltage_mv(primary_instance);
        } else {
            // With 2 range finders working in parallel this is obsolete. There is no use of it right now apart from Rover, but that logic will not use mixing anyway.
            // Return the sensor number that gives bigger influence for debug and setup purpose as it goes to mission planner.
            return ((effect[r_a] > effect[r_b])?r_a*1000:r_b*1000)+1000;
        }
    }

    inline int16_t max_distance_cm(uint8_t instance) const {
        return _max_distance_cm[instance];
    }
    int16_t max_distance_cm() const {
        if(!_mix_enable) {
            return max_distance_cm(primary_instance);
        } else {
            return max(max_distance_cm(0), max_distance_cm(1));
        }
    }

    inline int16_t min_distance_cm(uint8_t instance) const {
        return _min_distance_cm[instance];
    }
    int16_t min_distance_cm() const {
        if(!_mix_enable) {
            return min_distance_cm(primary_instance);
        } else {
            return min(min_distance_cm(0), min_distance_cm(1));
        }
    }
    
    inline bool healthy(uint8_t instance) const {
        return instance < num_instances && _RangeFinder_STATE(instance).healthy;
    }
    bool healthy() const {
        if(!_mix_enable) {
            return healthy(primary_instance);
        } else {
            return (healthy(0) && healthy(1));
        }
    }

    /*
      set an externally estimated terrain height. Used to enable power
      saving (where available) at high altitudes.
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }
    
private:
    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t primary_instance:2;
    uint8_t num_instances:2;
    float estimated_terrain_height;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  
};
#endif // __RANGEFINDER_H__
