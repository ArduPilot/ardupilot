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

#if PROXIMITY_MAX_INSTANCES > 1
    // @Group: 2
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[1], "2", 22, AP_Proximity, AP_Proximity_Params),
#endif

#if PROXIMITY_MAX_INSTANCES > 2
    // @Group: 3
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[2], "3", 23, AP_Proximity, AP_Proximity_Params),
#endif

#if PROXIMITY_MAX_INSTANCES > 3
    // @Group: 4
    // @Path: AP_Proximity_Params.cpp
    AP_SUBGROUPINFO(params[3], "4", 24, AP_Proximity, AP_Proximity_Params),
#endif

    AP_GROUPEND
};

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
    for (uint8_t i=0; i<PROXIMITY_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = Status::NotConnected;
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
        drivers[i]->boundary_3D_checks();
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != nullptr && (state[i].status == Status::Good)) {
            primary_instance = i;
        }
    }
}

AP_Proximity::Type AP_Proximity::get_type(uint8_t instance) const
{
    if (instance < PROXIMITY_MAX_INSTANCES) {
        return (Type)((uint8_t)params[instance].type);
    }
    return Type::None;
}

// return sensor orientation
uint8_t AP_Proximity::get_orientation(uint8_t instance) const
{
    if (!valid_instance(instance)) {
        return 0;
    }

    return params[instance].orientation.get();
}

// return sensor yaw correction
int16_t AP_Proximity::get_yaw_correction(uint8_t instance) const
{
    if (!valid_instance(instance)) {
        return 0;
    }

    return params[instance].yaw_correction.get();
}

// return sensor health
AP_Proximity::Status AP_Proximity::get_status(uint8_t instance) const
{
    // sanity check instance number
    if (!valid_instance(instance)) {
        return Status::NotConnected;
    }

    return state[instance].status;
}

AP_Proximity::Status AP_Proximity::get_status() const
{
    return get_status(primary_instance);
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity::distance_max() const
{
    if (!valid_instance(primary_instance)) {
        return 0.0f;
    }
    // get maximum distance from backend
    return drivers[primary_instance]->distance_max();
}
float AP_Proximity::distance_min() const
{
    if (!valid_instance(primary_instance)) {
        return 0.0f;
    }
    // get minimum distance from backend
    return drivers[primary_instance]->distance_min();
}


// get distances in 8 directions. used for sending distances to ground station
bool AP_Proximity::get_horizontal_distances(Proximity_Distance_Array &prx_dist_array) const
{
    if (!valid_instance(primary_instance)) {
        return false;
    }
    // get distances from backend
    return drivers[primary_instance]->get_horizontal_distances(prx_dist_array);
}

// get number of layers.
uint8_t AP_Proximity::get_num_layers() const
{
    if (!valid_instance(primary_instance)) {
        return 0;
    }
    return drivers[primary_instance]->get_num_layers();
}

// get raw and filtered distances in 8 directions per layer. used for logging
bool AP_Proximity::get_active_layer_distances(uint8_t layer, AP_Proximity::Proximity_Distance_Array &prx_dist_array, AP_Proximity::Proximity_Distance_Array &prx_filt_dist_array) const
{
    if (!valid_instance(primary_instance)) {
        return false;
    }
    // get distances from backend
    return drivers[primary_instance]->get_active_layer_distances(layer, prx_dist_array, prx_filt_dist_array);
}

// get total number of obstacles, used in GPS based Simple Avoidance
uint8_t AP_Proximity::get_obstacle_count() const
{   
    if (!valid_instance(primary_instance)) {
        return 0;
    }
    return drivers[primary_instance]->get_obstacle_count();
}

// get vector to obstacle based on obstacle_num passed, used in GPS based Simple Avoidance
bool AP_Proximity::get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const
{
    if (!valid_instance(primary_instance)) {
        return false;
    }
    return drivers[primary_instance]->get_obstacle(obstacle_num, vec_to_obstacle);
}

// returns shortest distance to "obstacle_num" obstacle, from a line segment formed between "seg_start" and "seg_end"
// used in GPS based Simple Avoidance
bool AP_Proximity::closest_point_from_segment_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{
    if (!valid_instance(primary_instance)) {
        return false;
    }
    return drivers[primary_instance]->closest_point_from_segment_to_obstacle(obstacle_num, seg_start, seg_end, closest_point);
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity::get_closest_object(float& angle_deg, float &distance) const
{
    if (!valid_instance(primary_instance)) {
        return false;
    }
    // get closest object from backend
    return drivers[primary_instance]->get_closest_object(angle_deg, distance);
}

// get number of objects, used for non-GPS avoidance
uint8_t AP_Proximity::get_object_count() const
{
    if (!valid_instance(primary_instance)) {
        return 0;
    }
    // get count from backend
    return drivers[primary_instance]->get_horizontal_object_count();
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if (!valid_instance(primary_instance)) {
        return false;
    }
    // get angle and distance from backend
    return drivers[primary_instance]->get_horizontal_object_angle_and_distance(object_number, angle_deg, distance);
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity::handle_msg(const mavlink_message_t &msg)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (valid_instance(i)) {
            drivers[i]->handle_msg(msg);
        }
    }
}

// methods for mavlink SYS_STATUS message (send_sys_status)
// these methods cover only the primary instance
bool AP_Proximity::sensor_present() const
{
    return get_status() != Status::NotConnected;
}
bool AP_Proximity::sensor_enabled() const
{
    return get_type(primary_instance) != Type::None;
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
    return get_upward_distance(primary_instance, distance);
}

// set alt as read from dowward facing rangefinder. Tilt is already adjusted for.
void AP_Proximity::set_rangefinder_alt(bool use, bool healthy, float alt_cm)
{
    if (!valid_instance(primary_instance)) {
        return;
    }
    // store alt at the backend
    drivers[primary_instance]->set_rangefinder_alt(use, healthy, alt_cm);
}

#if HAL_LOGGING_ENABLED
// Write proximity sensor distances
void AP_Proximity::log()
{
    // exit immediately if not enabled
    if (get_status() == AP_Proximity::Status::NotConnected) {
        return;
    }

    Proximity_Distance_Array dist_array{}; // raw distances stored here
    Proximity_Distance_Array filt_dist_array{}; //filtered distances stored here
    auto &logger { AP::logger() };
    for (uint8_t i = 0; i < get_num_layers(); i++) {
        const bool active = get_active_layer_distances(i, dist_array, filt_dist_array);
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

//  detect if an instance of a proximity sensor is connected.
void AP_Proximity::detect_instance(uint8_t instance)
{
    switch (get_type(instance)) {
    case Type::None:
        return;
    case Type::RPLidarA2:
        if (AP_Proximity_RPLidarA2::detect(instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_RPLidarA2(*this, state[instance], params[instance]);
            return;
        }
        break;
    case Type::MAV:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_MAV(*this, state[instance], params[instance]);
        return;

    case Type::TRTOWER:
        if (AP_Proximity_TeraRangerTower::detect(instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTower(*this, state[instance], params[instance]);
            return;
        }
        break;
    case Type::TRTOWEREVO:
        if (AP_Proximity_TeraRangerTowerEvo::detect(instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTowerEvo(*this, state[instance], params[instance]);
            return;
        }
        break;

    case Type::RangeFinder:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_RangeFinder(*this, state[instance], params[instance]);
        return;

    case Type::SF40C:
        if (AP_Proximity_LightWareSF40C::detect(instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C(*this, state[instance], params[instance]);
            return;
        }
        break;

    case Type::SF45B:
        if (AP_Proximity_LightWareSF45B::detect(instance)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF45B(*this, state[instance], params[instance]);
            return;
        }
        break;

    case Type::CYGBOT_D1:
#if AP_PROXIMITY_CYGBOT_ENABLED
    if (AP_Proximity_Cygbot_D1::detect(instance)) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_Cygbot_D1(*this, state[instance], params[instance]);
        return;
    }
# endif
    break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case Type::SITL:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_SITL(*this, state[instance], params[instance]);
        return;

    case Type::AirSimSITL:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_AirSimSITL(*this, state[instance], params[instance]);
        return;

#endif
    }
}



AP_Proximity *AP_Proximity::_singleton;

namespace AP {

AP_Proximity *proximity()
{
    return AP_Proximity::get_singleton();
}

}

#endif // HAL_PROXIMITY_ENABLED
