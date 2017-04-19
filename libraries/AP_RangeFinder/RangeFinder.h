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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of range finder instances available on this platform
#define RANGEFINDER_MAX_INSTANCES 2
#define RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT 10
#define RANGEFINDER_PREARM_ALT_MAX_CM           200
#define RANGEFINDER_PREARM_REQUIRED_CHANGE_CM   50

class AP_RangeFinder_Backend; 
 
class RangeFinder
{
public:
    friend class AP_RangeFinder_Backend;

    RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default);

    // RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_ANALOG = 1,
        RangeFinder_TYPE_MBI2C  = 2,
        RangeFinder_TYPE_PLI2C  = 3,
        RangeFinder_TYPE_PX4    = 4,
        RangeFinder_TYPE_PX4_PWM= 5,
        RangeFinder_TYPE_BBB_PRU= 6,
        RangeFinder_TYPE_LWI2C  = 7,
        RangeFinder_TYPE_LWSER  = 8,
        RangeFinder_TYPE_BEBOP  = 9,
        RangeFinder_TYPE_MAVLink = 10,
        RangeFinder_TYPE_ULANDING= 11,
        RangeFinder_TYPE_LEDDARONE = 12,
        RangeFinder_TYPE_MBSER  = 13,
        RangeFinder_TYPE_TRONE  = 14,
        RangeFinder_TYPE_PLI2CV3= 15,
    };

    enum RangeFinder_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum RangeFinder_Status {
        RangeFinder_NotConnected = 0,
        RangeFinder_NoData,
        RangeFinder_OutOfRangeLow,
        RangeFinder_OutOfRangeHigh,
        RangeFinder_Good
    };

    // The RangeFinder_State structure is filled in by the backend driver
    struct RangeFinder_State {
        uint8_t                instance;    // the instance number of this RangeFinder
        uint16_t               distance_cm; // distance: in cm
        uint16_t               voltage_mv;  // voltage in millivolts,
                                            // if applicable, otherwise 0
        enum RangeFinder_Status status;     // sensor status
        uint8_t                range_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        uint16_t               pre_arm_distance_min;    // min distance captured during pre-arm checks
        uint16_t               pre_arm_distance_max;    // max distance captured during pre-arm checks
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
    AP_Int8  _ground_clearance_cm[RANGEFINDER_MAX_INSTANCES];
    AP_Int8  _address[RANGEFINDER_MAX_INSTANCES];
    AP_Int16 _powersave_range;
    AP_Vector3f _pos_offset[RANGEFINDER_MAX_INSTANCES]; // position offset in body frame
    AP_Int8  _orientation[RANGEFINDER_MAX_INSTANCES];

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

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
    void handle_msg(mavlink_message_t *msg);

#define _RangeFinder_STATE(instance) state[instance]

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // find first range finder instance with the specified orientation
    bool find_instance(enum Rotation orientation, uint8_t &instance) const;

    // get orientation of a given range finder
    enum Rotation get_orientation(uint8_t instance) const {
        return (instance<num_instances? (enum Rotation)_orientation[instance].get() : ROTATION_NONE);
    }

    uint16_t distance_cm(uint8_t instance) const {
        return (instance<num_instances? _RangeFinder_STATE(instance).distance_cm : 0);
    }
    uint16_t distance_cm_orient(enum Rotation orientation) const;

    uint16_t voltage_mv(uint8_t instance) const {
        return _RangeFinder_STATE(instance).voltage_mv;
    }
    uint16_t voltage_mv_orient(enum Rotation orientation) const;

    int16_t max_distance_cm(uint8_t instance) const {
        return _max_distance_cm[instance];
    }
    int16_t max_distance_cm_orient(enum Rotation orientation) const;

    int16_t min_distance_cm(uint8_t instance) const {
        return _min_distance_cm[instance];
    }
    int16_t min_distance_cm_orient(enum Rotation orientation) const;

    int16_t ground_clearance_cm(uint8_t instance) const {
        return _ground_clearance_cm[instance];
    }
    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;

    // query status
    RangeFinder_Status status(uint8_t instance) const;
    RangeFinder_Status status_orient(enum Rotation orientation) const;

    // true if sensor is returning data
    bool has_data(uint8_t instance) const;
    bool has_data_orient(enum Rotation orientation) const;

    // returns count of consecutive good readings
    uint8_t range_valid_count(uint8_t instance) const {
        return _RangeFinder_STATE(instance).range_valid_count;
    }
    uint8_t range_valid_count_orient(enum Rotation orientation) const;

    /*
      set an externally estimated terrain height. Used to enable power
      saving (where available) at high altitudes.
     */
    void set_estimated_terrain_height(float height) {
        estimated_terrain_height = height;
    }

    /*
      returns true if pre-arm checks have passed for all range finders
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;

    // return a 3D vector defining the position offset of the sensor in metres relative to the body frame origin
    const Vector3f &get_pos_offset(uint8_t instance) const {
        return _pos_offset[instance];
    }
    const Vector3f &get_pos_offset_orient(enum Rotation orientation) const;

private:
    RangeFinder_State state[RANGEFINDER_MAX_INSTANCES];
    AP_RangeFinder_Backend *drivers[RANGEFINDER_MAX_INSTANCES];
    uint8_t num_instances:3;
    float estimated_terrain_height;
    AP_SerialManager &serial_manager;
    Vector3f pos_offset_zero;   // allows returning position offsets of zero for invalid requests

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    void update_pre_arm_check(uint8_t instance);
    bool _add_backend(AP_RangeFinder_Backend *driver);
};
