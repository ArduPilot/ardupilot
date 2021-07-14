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
#include "AP_Proximity_LightWareSF40C_v09.h"
#include "AP_Proximity_RPLidarA2.h"
#include "AP_Proximity_TeraRangerTower.h"
#include "AP_Proximity_TeraRangerTowerEvo.h"
#include "AP_Proximity_RangeFinder.h"
#include "AP_Proximity_MAV.h"
#include "AP_Proximity_LightWareSF40C.h"
#include "AP_Proximity_LightWareSF45B.h"
#include "AP_Proximity_SITL.h"
#include "AP_Proximity_AirSimSITL.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity::var_info[] = {
    // 0 is reserved for possible addition of an ENABLED parameter

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,7:LightwareSF40c,1:LightWareSF40C-legacy,2:MAVLink,3:TeraRangerTower,4:RangeFinder,5:RPLidarA2,6:TeraRangerTowerEvo,8:LightwareSF45B,10:SITL,12:AirSimSITL
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_TYPE",   1, AP_Proximity, _type[0], 0),

    // @Param: _ORIENT
    // @DisplayName: Proximity sensor orientation
    // @Description: Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("_ORIENT", 2, AP_Proximity, _orientation[0], 0),

    // @Param: _YAW_CORR
    // @DisplayName: Proximity sensor yaw correction
    // @Description: Proximity sensor yaw correction
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_YAW_CORR", 3, AP_Proximity, _yaw_correction[0], 0),

    // @Param: _IGN_ANG1
    // @DisplayName: Proximity sensor ignore angle 1
    // @Description: Proximity sensor ignore angle 1
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG1", 4, AP_Proximity, _ignore_angle_deg[0], 0),

    // @Param: _IGN_WID1
    // @DisplayName: Proximity sensor ignore width 1
    // @Description: Proximity sensor ignore width 1
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID1", 5, AP_Proximity, _ignore_width_deg[0], 0),

    // @Param: _IGN_ANG2
    // @DisplayName: Proximity sensor ignore angle 2
    // @Description: Proximity sensor ignore angle 2
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG2", 6, AP_Proximity, _ignore_angle_deg[1], 0),

    // @Param: _IGN_WID2
    // @DisplayName: Proximity sensor ignore width 2
    // @Description: Proximity sensor ignore width 2
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID2", 7, AP_Proximity, _ignore_width_deg[1], 0),

    // @Param: _IGN_ANG3
    // @DisplayName: Proximity sensor ignore angle 3
    // @Description: Proximity sensor ignore angle 3
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG3", 8, AP_Proximity, _ignore_angle_deg[2], 0),

    // @Param: _IGN_WID3
    // @DisplayName: Proximity sensor ignore width 3
    // @Description: Proximity sensor ignore width 3
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID3", 9, AP_Proximity, _ignore_width_deg[2], 0),

    // @Param: _IGN_ANG4
    // @DisplayName: Proximity sensor ignore angle 4
    // @Description: Proximity sensor ignore angle 4
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG4", 10, AP_Proximity, _ignore_angle_deg[3], 0),

    // @Param: _IGN_WID4
    // @DisplayName: Proximity sensor ignore width 4
    // @Description: Proximity sensor ignore width 4
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID4", 11, AP_Proximity, _ignore_width_deg[3], 0),

    // @Param: _IGN_ANG5
    // @DisplayName: Proximity sensor ignore angle 5
    // @Description: Proximity sensor ignore angle 5
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG5", 12, AP_Proximity, _ignore_angle_deg[4], 0),

    // @Param: _IGN_WID5
    // @DisplayName: Proximity sensor ignore width 5
    // @Description: Proximity sensor ignore width 5
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID5", 13, AP_Proximity, _ignore_width_deg[4], 0),

    // @Param: _IGN_ANG6
    // @DisplayName: Proximity sensor ignore angle 6
    // @Description: Proximity sensor ignore angle 6
    // @Units: deg
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG6", 14, AP_Proximity, _ignore_angle_deg[5], 0),

    // @Param: _IGN_WID6
    // @DisplayName: Proximity sensor ignore width 6
    // @Description: Proximity sensor ignore width 6
    // @Units: deg
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("_IGN_WID6", 15, AP_Proximity, _ignore_width_deg[5], 0),

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
void AP_Proximity::init(void)
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
void AP_Proximity::update(void)
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

// return sensor orientation
uint8_t AP_Proximity::get_orientation(uint8_t instance) const
{
    if (!valid_instance(instance)) {
        return 0;
    }

    return _orientation[instance].get();
}

// return sensor yaw correction
int16_t AP_Proximity::get_yaw_correction(uint8_t instance) const
{
    if (!valid_instance(instance)) {
        return 0;
    }

    return _yaw_correction[instance].get();
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

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity::handle_msg(const mavlink_message_t &msg)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (valid_instance(i)) {
            drivers[i]->handle_msg(msg);
        }
    }
}

//  detect if an instance of a proximity sensor is connected.
void AP_Proximity::detect_instance(uint8_t instance)
{
    switch (get_type(instance)) {
    case Type::None:
        return;
    case Type::SF40C_v09:
        if (AP_Proximity_LightWareSF40C_v09::detect()) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C_v09(*this, state[instance]);
            return;
        }
        break;
    case Type::RPLidarA2:
        if (AP_Proximity_RPLidarA2::detect()) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_RPLidarA2(*this, state[instance]);
            return;
        }
        break;
    case Type::MAV:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_MAV(*this, state[instance]);
        return;

    case Type::TRTOWER:
        if (AP_Proximity_TeraRangerTower::detect()) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTower(*this, state[instance]);
            return;
        }
        break;
    case Type::TRTOWEREVO:
        if (AP_Proximity_TeraRangerTowerEvo::detect()) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTowerEvo(*this, state[instance]);
            return;
        }
        break;

    case Type::RangeFinder:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_RangeFinder(*this, state[instance]);
        return;

    case Type::SF40C:
        if (AP_Proximity_LightWareSF40C::detect()) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C(*this, state[instance]);
            return;
        }
        break;

    case Type::SF45B:
        if (AP_Proximity_LightWareSF45B::detect()) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF45B(*this, state[instance]);
            return;
        }
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case Type::SITL:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_SITL(*this, state[instance]);
        return;

    case Type::AirSimSITL:
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_AirSimSITL(*this, state[instance]);
        return;
#endif
    }
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

// get number of layers.
uint8_t AP_Proximity::get_num_layers() const
{
    if (!valid_instance(primary_instance)) {
        return 0;
    }
    return drivers[primary_instance]->get_num_layers();
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

AP_Proximity::Type AP_Proximity::get_type(uint8_t instance) const
{
    if (instance < PROXIMITY_MAX_INSTANCES) {
        return (Type)((uint8_t)_type[instance]);
    }
    return Type::None;
}

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

// set alt as read from dowward facing rangefinder. Tilt is already adjusted for.
void AP_Proximity::set_rangefinder_alt(bool use, bool healthy, float alt_cm)
{
    if (!valid_instance(primary_instance)) {
        return;
    }
    // store alt at the backend
    drivers[primary_instance]->set_rangefinder_alt(use, healthy, alt_cm);
}


AP_Proximity *AP_Proximity::_singleton;

namespace AP {

AP_Proximity *proximity()
{
    return AP_Proximity::get_singleton();
}

}

#endif // HAL_PROXIMITY_ENABLED
