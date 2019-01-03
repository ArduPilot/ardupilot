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
#include "AP_Proximity_LightWareSF40C.h"
#include "AP_Proximity_RPLidarA2.h"
#include "AP_Proximity_TeraRangerTower.h"
#include "AP_Proximity_TeraRangerTowerEvo.h"
#include "AP_Proximity_RangeFinder.h"
#include "AP_Proximity_MAV.h"
#include "AP_Proximity_SITL.h"
#include "AP_Proximity_MorseSITL.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity::var_info[] = {
    // 0 is reserved for possible addition of an ENABLED parameter

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C,2:MAVLink,3:TeraRangerTower,4:RangeFinder,5:RPLidarA2M6-R3,6:TeraRangerTowerEvo,10:SITL,11:MorseSITL,12:RPLidarA2M6-R4,13:RPLidarA2M4,14:RPLidarA2M8-R3,15:RPLidarA2M8-R4,16:RPLidarA1M8-R4,17:RPLidarA1M8-R5,18:RPLidarA3M1
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
    AP_GROUPINFO("_YAW_CORR", 3, AP_Proximity, _yaw_correction[0], PROXIMITY_YAW_CORRECTION_DEFAULT),

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
    // @Range: 0 45
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
    // @Range: 0 45
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
    // @Range: 0 45
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
    // @Range: 0 45
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
    // @Range: 0 45
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
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID6", 15, AP_Proximity, _ignore_width_deg[5], 0),

    // @Param: _DISTMAX_M
    // @DisplayName: maximum distance (in meters) of sensor
    // @Description: maximum distance (in meters) of sensor
    // @Units: m
    // @Range: 0.00 25.00
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_DISTMAX_M", 19, AP_Proximity, _distance_max[0], 16.00f),

    // @Param: __DISTMIN_M
    // @DisplayName: minimum distance (in meters) of sensor
    // @Description: minimum distance (in meters) of sensor
    // @Units: m
    // @Range: 0.00 25.00
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("__DISTMIN_M", 20, AP_Proximity, _distance_min[0], 0.20f),

#if PROXIMITY_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C,2:MAVLink,3:TeraRangerTower,4:RangeFinder,5:RPLidarA2,6:TeraRangerTowerEvo
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("2_TYPE", 16, AP_Proximity, _type[1], 0),

    // @Param: 2_ORIENT
    // @DisplayName: Second Proximity sensor orientation
    // @Description: Second Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("2_ORIENT", 17, AP_Proximity, _orientation[1], 0),

    // @Param: 2_YAW_CORR
    // @DisplayName: Second Proximity sensor yaw correction
    // @Description: Second Proximity sensor yaw correction
    // @Units: deg
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("2_YAW_CORR", 18, AP_Proximity, _yaw_correction[1], PROXIMITY_YAW_CORRECTION_DEFAULT),

    // @Param: _DISTMAX_M
    // @DisplayName: maximum distance (in meters) of sensor
    // @Description: maximum distance (in meters) of sensor
    // @Units: m
    // @Range: 0.00 25.00
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_DISTMAX_M", 21, AP_Proximity, _distance_max[1], 16.00f),

    // @Param: _DISTMIN_M
    // @DisplayName: minimum distance (in meters) of sensor
    // @Description: minimum distance (in meters) of sensor
    // @Units: m
    // @Range: 0.00 25.00
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_DISTMIN_M", 22, AP_Proximity, _distance_min[1], 0.20f),
#endif

    AP_GROUPEND
};

AP_Proximity::AP_Proximity(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager)
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
        state[i].status = Proximity_NotConnected;
    }
}

// update Proximity state for all instances. This should be called at a high rate by the main loop
void AP_Proximity::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_type[i] == Proximity_Type_None) {
                // allow user to disable a proximity sensor at runtime
                state[i].status = Proximity_NotConnected;
                continue;
            }
            drivers[i]->update();
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != nullptr && (state[i].status == Proximity_Good)) {
            primary_instance = i;
        }
    }
}

// return sensor orientation
uint8_t AP_Proximity::get_orientation(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0;
    }

    return _orientation[instance].get();
}

// return sensor yaw correction
int16_t AP_Proximity::get_yaw_correction(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0;
    }

    return _yaw_correction[instance].get();
}

// return sensor maximum distance (in meters)
float AP_Proximity::get_distance_max(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 16.0f;
    }

    return _distance_max[instance].get();
}

// return sensor minimum distance (in meters)
float AP_Proximity::get_distance_min(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0.20f;
    }

    return _distance_min[instance].get();
}

// return sensor health
AP_Proximity::Proximity_Status AP_Proximity::get_status(uint8_t instance) const
{
    // sanity check instance number
    if (instance >= num_instances) {
        return Proximity_NotConnected;
    }

    return state[instance].status;
}

AP_Proximity::Proximity_Status AP_Proximity::get_status() const
{
    return get_status(primary_instance);
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity::handle_msg(mavlink_message_t *msg)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (_type[i] != Proximity_Type_None)) {
            drivers[i]->handle_msg(msg);
        }
    }
}

//  detect if an instance of a proximity sensor is connected.
void AP_Proximity::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];
    if (type == Proximity_Type_SF40C) {
        if (AP_Proximity_LightWareSF40C::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C(*this, state[instance], serial_manager);
            return;
        }
    }
    switch (type) {
    case Proximity_Type_RPLidarA2M6R3:
    case Proximity_Type_RPLidarA2M6R4:
    case Proximity_Type_RPLidarA2M4:
    case Proximity_Type_RPLidarA2M8R3:
    case Proximity_Type_RPLidarA2M8R4:
    case Proximity_Type_RPLidarA1M8R4:
    case Proximity_Type_RPLidarA1M8R5:
    case Proximity_Type_RPLidarA3M1:
        if (AP_Proximity_RPLidarA2::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_RPLidarA2(*this, state[instance], serial_manager);
            return;
        }
        break;
    default:
        break;
    }
    if (type == Proximity_Type_MAV) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_MAV(*this, state[instance]);
        return;
    }
    if (type == Proximity_Type_TRTOWER) {
        if (AP_Proximity_TeraRangerTower::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTower(*this, state[instance], serial_manager);
            return;
        }
    }
    if (type == Proximity_Type_TRTOWEREVO) {
        if (AP_Proximity_TeraRangerTowerEvo::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTowerEvo(*this, state[instance], serial_manager);
            return;
        }
    }
    if (type == Proximity_Type_RangeFinder) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_RangeFinder(*this, state[instance]);
        return;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (type == Proximity_Type_SITL) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_SITL(*this, state[instance]);
        return;
    }
    if (type == Proximity_Type_MorseSITL) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_MorseSITL(*this, state[instance]);
        return;
    }
#endif
}

// get distance in meters in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance
bool AP_Proximity::get_horizontal_distance(uint8_t instance, float angle_deg, float &distance) const
{
    if ((drivers[instance] == nullptr) || (_type[instance] == Proximity_Type_None)) {
        return false;
    }
    // get distance from backend
    return drivers[instance]->get_horizontal_distance(angle_deg, distance);
}

// get distance in meters in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance
bool AP_Proximity::get_horizontal_distance(float angle_deg, float &distance) const
{
    return get_horizontal_distance(primary_instance, angle_deg, distance);
}

// get distances in 8 directions. used for sending distances to ground station
bool AP_Proximity::get_horizontal_distances(Proximity_Distance_Array &prx_dist_array) const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return false;
    }
    // get distances from backend
    return drivers[primary_instance]->get_horizontal_distances(prx_dist_array);
}

// get boundary points around vehicle for use by avoidance
//   returns nullptr and sets num_points to zero if no boundary can be returned
const Vector2f* AP_Proximity::get_boundary_points(uint8_t instance, uint16_t& num_points) const
{
    if ((drivers[instance] == nullptr) || (_type[instance] == Proximity_Type_None)) {
        num_points = 0;
        return nullptr;
    }
    // get boundary from backend
    return drivers[instance]->get_boundary_points(num_points);
}

const Vector2f* AP_Proximity::get_boundary_points(uint16_t& num_points) const
{
    return get_boundary_points(primary_instance, num_points);
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity::get_closest_object(float& angle_deg, float &distance) const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return false;
    }
    // get closest object from backend
    return drivers[primary_instance]->get_closest_object(angle_deg, distance);
}

// get number of objects, used for non-GPS avoidance
uint8_t AP_Proximity::get_object_count() const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return 0;
    }
    // get count from backend
    return drivers[primary_instance]->get_object_count();
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return false;
    }
    // get angle and distance from backend
    return drivers[primary_instance]->get_object_angle_and_distance(object_number, angle_deg, distance);
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity::distance_max() const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return 0.0f;
    }
    // get maximum distance from backend
    return drivers[primary_instance]->distance_max();
}
float AP_Proximity::distance_min() const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return 0.0f;
    }
    // get minimum distance from backend
    return drivers[primary_instance]->distance_min();
}

// get distance in meters upwards, returns true on success
bool AP_Proximity::get_upward_distance(uint8_t instance, float &distance) const
{
    if ((drivers[instance] == nullptr) || (_type[instance] == Proximity_Type_None)) {
        return false;
    }
    // get upward distance from backend
    return drivers[instance]->get_upward_distance(distance);
}

bool AP_Proximity::get_upward_distance(float &distance) const
{
    return get_upward_distance(primary_instance, distance);
}

AP_Proximity::Proximity_Type AP_Proximity::get_type(uint8_t instance) const
{
    if (instance < PROXIMITY_MAX_INSTANCES) {
        return (Proximity_Type)((uint8_t)_type[instance]);
    }
    return Proximity_Type_None;
}

bool AP_Proximity::sensor_present() const
{
    return get_status() != Proximity_NotConnected;
}
bool AP_Proximity::sensor_enabled() const
{
    return _type[primary_instance] != Proximity_Type_None;
}
bool AP_Proximity::sensor_failed() const
{
    return get_status() != Proximity_Good;
}

AP_Proximity *AP_Proximity::_singleton;
