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

#include "AP_Proximity.h"

#if HAL_PROXIMITY_ENABLED
#include "AP_Proximity_RPLidarA2.h"
#include "AP_Proximity_TeraRangerTower.h"
#include "AP_Proximity_TeraRangerTowerEvo.h"
#include "AP_Proximity_RangeFinder.h"
#include "AP_Proximity_MAV.h"
#include "AP_Proximity_LightWareSF40C.h"
#include "AP_Proximity_LightWareSF45B.h"
#include "AP_Proximity_SITL.h"
#include "AP_Proximity_AirSimSITL.h"
#include "AP_Proximity_Cygbot_D1.h"
#include "AP_Proximity_MR72_CAN.h"


#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity::var_info[] = {
    // 0 is reserved for possible addition of an ENABLED parameter

    // 1 was _TYPE
    // 2 was _ORIENT
    // 3 was _YAW_CORR
    // 4 to 15 was _IGN_ANG1 to _IGN_WID6

    // @Param{Copter}: _IGN_GND
    // @DisplayName: Proximity sensor land detection
    // @Description: Ignore proximity data that is within 1 meter of the ground below the vehicle. This requires a downward facing rangefinder
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO_FRAME("_IGN_GND", 16, AP_Proximity, _ign_gnd_enable, 0, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    // @Param: _LOG_RAW
    // @DisplayName: Proximity raw distances log
    // @Description: Set this parameter to one if logging unfiltered(raw) distances from sensor should be enabled
    // @Values: 0:Off, 1:On
    // @User: Advanced
    AP_GROUPINFO("_LOG_RAW", 17, AP_Proximity, _raw_log_enable, 0),

    // @Param: _FILT
    // @DisplayName: Proximity filter cutoff frequency
    // @Description: Cutoff frequency for low pass filter applied to each face in the proximity boundary
    // @Units: Hz
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("_FILT", 18, AP_Proximity, _filt_freq, 0.25f),

    // 19 was _MIN
    // 20 was _MAX

    // @Group: 1
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[0], "1", 21, AP_Proximity, AP_Proximity_Params),

    // @Group: 1_
    // @Path: AP_Proximity_MR72_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  25, AP_Proximity, backend_var_info[0]),

#if PROXIMITY_MAX_INSTANCES > 1
    // @Group: 2
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[1], "2", 22, AP_Proximity, AP_Proximity_Params),

    // @Group: 2_
    // @Path: AP_Proximity_MR72_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  26, AP_Proximity, backend_var_info[1]),
#endif

#if PROXIMITY_MAX_INSTANCES > 2
    // @Group: 3
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[2], "3", 23, AP_Proximity, AP_Proximity_Params),

    // @Group: 3_
    // @Path: AP_Proximity_MR72_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  27, AP_Proximity, backend_var_info[2]),
#endif

#if PROXIMITY_MAX_INSTANCES > 3
    // @Group: 4
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[3], "4", 24, AP_Proximity, AP_Proximity_Params),

    // @Group: 4_
    // @Path: AP_Proximity_MR72_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_",  28, AP_Proximity, backend_var_info[3]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *AP_Proximity::backend_var_info[PROXIMITY_MAX_INSTANCES];

AP_Proximity::AP_Proximity()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Proximity must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

// initialise the Proximity class. We do detection of attached sensors here
// we don't allow for hot-plugging of sensors (i.e. reboot required)
void AP_Proximity::init()
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    // instantiate backends
    uint8_t serial_instance = 0;
    for (uint8_t instance=0; instance<PROXIMITY_MAX_INSTANCES; instance++) {
        switch (get_type(instance)) {
        case Type::None:
            break;
        case Type::RPLidarA2:
            if (AP_Proximity_RPLidarA2::detect(serial_instance)) {
                state[instance].instance = instance;
                drivers[instance] = new AP_Proximity_RPLidarA2(*this, state[instance], params[instance], serial_instance);
                serial_instance++;
            }
            break;
        case Type::MAV:
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_MAV(*this, state[instance], params[instance]);
            break;

        case Type::TRTOWER:
            if (AP_Proximity_TeraRangerTower::detect(serial_instance)) {
                state[instance].instance = instance;
                drivers[instance] = new AP_Proximity_TeraRangerTower(*this, state[instance], params[instance], serial_instance);
                serial_instance++;
            }
            break;
        case Type::TRTOWEREVO:
            if (AP_Proximity_TeraRangerTowerEvo::detect(serial_instance)) {
                state[instance].instance = instance;
                drivers[instance] = new AP_Proximity_TeraRangerTowerEvo(*this, state[instance], params[instance], serial_instance);
                serial_instance++;
            }
            break;

        case Type::RangeFinder:
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_RangeFinder(*this, state[instance], params[instance]);
            break;

        case Type::SF40C:
            if (AP_Proximity_LightWareSF40C::detect(serial_instance)) {
                state[instance].instance = instance;
                drivers[instance] = new AP_Proximity_LightWareSF40C(*this, state[instance], params[instance], serial_instance);
                serial_instance++;
            }
            break;

        case Type::SF45B:
            if (AP_Proximity_LightWareSF45B::detect(serial_instance)) {
                state[instance].instance = instance;
                drivers[instance] = new AP_Proximity_LightWareSF45B(*this, state[instance], params[instance], serial_instance);
                serial_instance++;
            }
            break;

        case Type::CYGBOT_D1:
#if AP_PROXIMITY_CYGBOT_ENABLED
        if (AP_Proximity_Cygbot_D1::detect(serial_instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_Cygbot_D1(*this, state[instance], params[instance], serial_instance);
            serial_instance++;
        }
# endif
        break;
        case Type::MR72:
#if AP_PROXIMITY_MR72_ENABLED
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_MR72_CAN(*this, state[instance], params[instance]);
# endif
            break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case Type::SITL:
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_SITL(*this, state[instance], params[instance]);
            break;

        case Type::AirSimSITL:
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_AirSimSITL(*this, state[instance], params[instance]);
            break;
#endif
        }

        if (drivers[instance] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = instance+1;
        }

        // initialise status
        state[instance].status = Status::NotConnected;

        // if the backend has some local parameters then make those available in the tree
        if (drivers[instance] && state[instance].var_info) {
            backend_var_info[instance] = state[instance].var_info;
            AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

            // param count could have changed
            AP_Param::invalidate_count();
        }
    }
}

// update Proximity state for all instances. This should be called at a high rate by the main loop
void AP_Proximity::update()
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (!valid_instance(i)) {
            continue;
        }
        drivers[i]->update();
    }

    // set boundary cutoff freq for low pass filter
    boundary.set_filter_freq(get_filter_freq());

    // check if any face has valid distance when it should not
    boundary.check_face_timeout();
}

AP_Proximity::Type AP_Proximity::get_type(uint8_t instance) const
{
    if (instance < PROXIMITY_MAX_INSTANCES) {
        return (Type)((uint8_t)params[instance].type);
    }
    return Type::None;
}

// return sensor health
AP_Proximity::Status AP_Proximity::get_instance_status(uint8_t instance) const
{
    // sanity check instance number
    if (!valid_instance(instance)) {
        return Status::NotConnected;
    }

    return state[instance].status;
}

AP_Proximity::Status AP_Proximity::get_status() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        const Status sensors_status = get_instance_status(i);
        if (sensors_status != Status::Good) {
            // return first bad status
            return sensors_status;
        }
    }
    // All valid sensors seem to be working
    return Status::Good;
}

// prearm checks
bool AP_Proximity::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        switch (get_instance_status(i)) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "PRX%d: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "PRX%d: Not Connected", i + 1);
            return false;
        case Status::Good:
            break;
        }
    }
    return true;
}

// get maximum and minimum distances (in meters)
float AP_Proximity::distance_max() const
{
    float dist_max = 0;

    // return longest distance from all backends
    for (uint8_t i=0; i<num_instances; i++) {
        if (valid_instance(i)) {
            dist_max = MAX(dist_max, drivers[i]->distance_max());
        }
    }
    return dist_max;
}
float AP_Proximity::distance_min() const
{
    float dist_min = 0;
    bool found_dist_min = false;

    // calculate shortest distance from all backends
    for (uint8_t i=0; i<num_instances; i++) {
        if (valid_instance(i)) {
            const float disti_min = drivers[i]->distance_min();
            if (!found_dist_min || (disti_min <= dist_min)) {
                dist_min = disti_min;
                found_dist_min = true;
            }
        }
    }

    if (found_dist_min) {
        return dist_min;
    }
    return 0;
}


// get distances in 8 directions. used for sending distances to ground station
bool AP_Proximity::get_horizontal_distances(Proximity_Distance_Array &prx_dist_array) const
{
    Proximity_Distance_Array prx_filt_dist_array; // unused
    return boundary.get_layer_distances(PROXIMITY_MIDDLE_LAYER, distance_max(), prx_dist_array, prx_filt_dist_array);
}

// get total number of obstacles, used in GPS based Simple Avoidance
uint8_t AP_Proximity::get_obstacle_count() const
{
    return boundary.get_obstacle_count();
}

// get vector to obstacle based on obstacle_num passed, used in GPS based Simple Avoidance
bool AP_Proximity::get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const
{
    return boundary.get_obstacle(obstacle_num, vec_to_obstacle);
}

// returns shortest distance to "obstacle_num" obstacle, from a line segment formed between "seg_start" and "seg_end"
// returns FLT_MAX if it's an invalid instance.
bool AP_Proximity::closest_point_from_segment_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{
    return boundary.closest_point_from_segment_to_obstacle(obstacle_num , seg_start, seg_end, closest_point);
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity::get_closest_object(float& angle_deg, float &distance) const
{
    return boundary.get_closest_object(angle_deg, distance);
}

// get number of objects, angle and distance - used for non-GPS avoidance
uint8_t AP_Proximity::get_object_count() const
{
    return boundary.get_horizontal_object_count();
}

// get number of objects, angle and distance - used for non-GPS avoidance
bool AP_Proximity::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    return boundary.get_horizontal_object_angle_and_distance(object_number, angle_deg, distance);
}

// handle mavlink messages
void AP_Proximity::handle_msg(const mavlink_message_t &msg)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (valid_instance(i)) {
            drivers[i]->handle_msg(msg);
        }
    }
}

// methods for mavlink SYS_STATUS message (send_sys_status)
bool AP_Proximity::sensor_present() const
{
    return get_status() != Status::NotConnected;
}

bool AP_Proximity::sensor_enabled() const
{
    // check atleast one sensor is enabled
    return (num_instances > 0);
}

bool AP_Proximity::sensor_failed() const
{
    return get_status() != Status::Good;
}

// get distance in meters upwards, returns true on success
bool AP_Proximity::get_upward_distance(uint8_t instance, float &distance) const
{
    if (!valid_instance(instance)) {
        return false;
    }
    // get upward distance from backend
    return drivers[instance]->get_upward_distance(distance);
}

bool AP_Proximity::get_upward_distance(float &distance) const
{
    // get upward distance from backend
    for (uint8_t i=0; i<num_instances; i++) {
        // return first good upward distance
        if (get_upward_distance(i, distance)) {
            return true;
        }
    }
    return false;
}

#if HAL_LOGGING_ENABLED
// Write proximity sensor distances
void AP_Proximity::log()
{
    // exit immediately if no sensors
    if (num_sensors() == 0) {
        return;
    }

    Proximity_Distance_Array dist_array{}; // raw distances stored here
    Proximity_Distance_Array filt_dist_array{}; //filtered distances stored here
    auto &logger { AP::logger() };
    for (uint8_t i = 0; i < boundary.get_num_layers(); i++) {
        const bool active = boundary.get_layer_distances(i, distance_max(), dist_array, filt_dist_array);
        if (!active) {
            // nothing on this layer
            continue;
        }
        float dist_up;
        if (!get_upward_distance(dist_up)) {
            dist_up = 0.0f;
        }

        float closest_ang = 0.0f;
        float closest_dist = 0.0f;
        get_closest_object(closest_ang, closest_dist);

        const struct log_Proximity pkt_proximity{
                LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_MSG),
                time_us         : AP_HAL::micros64(),
                instance        : i,
                health          : (uint8_t)get_status(),
                dist0           : filt_dist_array.distance[0],
                dist45          : filt_dist_array.distance[1],
                dist90          : filt_dist_array.distance[2],
                dist135         : filt_dist_array.distance[3],
                dist180         : filt_dist_array.distance[4],
                dist225         : filt_dist_array.distance[5],
                dist270         : filt_dist_array.distance[6],
                dist315         : filt_dist_array.distance[7],
                distup          : dist_up,
                closest_angle   : closest_ang,
                closest_dist    : closest_dist
        };
        logger.WriteBlock(&pkt_proximity, sizeof(pkt_proximity));

        if (_raw_log_enable) {
            const struct log_Proximity_raw pkt_proximity_raw{
                LOG_PACKET_HEADER_INIT(LOG_RAW_PROXIMITY_MSG),
                time_us         : AP_HAL::micros64(),
                instance        : i,
                raw_dist0       : dist_array.distance[0],
                raw_dist45      : dist_array.distance[1],
                raw_dist90      : dist_array.distance[2],
                raw_dist135     : dist_array.distance[3],
                raw_dist180     : dist_array.distance[4],
                raw_dist225     : dist_array.distance[5],
                raw_dist270     : dist_array.distance[6],
                raw_dist315     : dist_array.distance[7],
            };
            logger.WriteBlock(&pkt_proximity_raw, sizeof(pkt_proximity_raw));
        }
    }
}
#endif

// return true if the given instance exists
bool AP_Proximity::valid_instance(uint8_t i) const
{
    if (i >= PROXIMITY_MAX_INSTANCES) {
        return false;
    }

    if (drivers[i] == nullptr) {
        return false;
    }
    return (Type)params[i].type.get() != Type::None;
}

AP_Proximity *AP_Proximity::_singleton;

namespace AP {

AP_Proximity *proximity()
{
    return AP_Proximity::get_singleton();
}

}

#endif // HAL_PROXIMITY_ENABLED
