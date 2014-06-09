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
#include <Filter.h> // Filter library
#include <rotations.h>

// Maximum number of range finder instances available on this platform
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define RANGEFINDER_MAX_INSTANCES 2
#else
#define RANGEFINDER_MAX_INSTANCES 1
#endif
 
class AP_RangeFinder_Backend;
 
class RangeFinder
{
public:
    RangeFinder(FilterInt16 *filter) :
        _mode_filter(filter)    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE  = 0,
        RangeFinder_TYPE_AUTO  = 1,
        RangeFinder_TYPE_ANALOG = 2,
        RangeFinder_TYPE_MBI2C  = 3,
        RangeFinder_TYPE_PLI2C = 4
    };

    enum RangeFinder_Location {
        RangeFinder_LOCATION_FRONT  = 0,
        RangeFinder_LOCATION_RIGHT  = 1,
        RangeFinder_LOCATION_LEFT = 2,
        RangeFinder_LOCATION_BACK  = 3
    };
    
    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint8_t                instance; // the instance number of this RangeFinder
        int16_t                distance; // distance: in cm
        int16_t                max_distance; // maximum measurable distance: in cm
        int16_t                min_distance; // minimum measurable distance: in cm
        bool                   healthy; // sensor is communicating correctly
        Rotation               orientation; // none would imply that it is pointing out the craft front
        RangeFinder_Location   location; // generic approximation of the sensor's location on the craft
    };

    AP_Int8                                 _type[RANGEFINDER_MAX_INSTANCES];
    AP_Int8                                 _pin[RANGEFINDER_MAX_INSTANCES];
    FilterInt16 *                           _mode_filter;
    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    void init(void);
    void update(void);
    
#if RANGEFINDER_MAX_INSTANCES == 1
#	define _RangeFinder_STATE(instance) state[0]
#else
#	define _RangeFinder_STATE(instance) state[instance]
#endif

    int16_t distance(uint8_t instance) const {
        return _RangeFinder_STATE(instance).distance;
    }
    int16_t distance() const {
        return distance(primary_instance);
    }

    int16_t max_distance(uint8_t instance) const {
        return _RangeFinder_STATE(instance).max_distance;
    }
    int16_t max_distance() const {
        return max_distance(primary_instance);
    }

    int16_t min_distance(uint8_t instance) const {
        return _RangeFinder_STATE(instance).min_distance;
    }
    int16_t min_distance() const {
        return min_distance(primary_instance);
    }
    
    bool healthy(uint8_t instance) const {
        return _RangeFinder_STATE(instance).healthy;
    }
    bool healthy() const {
        return healthy(primary_instance);
    }
    
    Rotation orientation(uint8_t instance) const {
        return _RangeFinder_STATE(instance).orientation;
    }
    Rotation orientation() const {
        return orientation(primary_instance);
    }
    
    RangeFinder_Location location(uint8_t instance) const {
        return _RangeFinder_STATE(instance).location;
    }
    RangeFinder_Location location() const {
        return location(primary_instance);
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
