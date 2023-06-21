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

#include "AP_Follow_config.h"

#if AP_FOLLOW_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_LocationDB/AP_LocationDB.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // position estimate timeout after 1 second

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // offsets are in north-east-down frame
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  1   // offsets are relative to lead vehicle's heading

#define AP_FOLLOW_ALTITUDE_TYPE_RELATIVE  1 // relative altitude is used by default   

#define AP_FOLLOW_POS_P_DEFAULT 0.1f    // position error gain default

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_FOLLOW_ALT_TYPE_DEFAULT 0
#else
#define AP_FOLLOW_ALT_TYPE_DEFAULT AP_FOLLOW_ALTITUDE_TYPE_RELATIVE
#endif

AP_Follow *AP_Follow::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_Follow::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // 2 is reserved for TYPE parameter

    // @Param: _SYSID
    // @DisplayName: Follow target's mavlink system id
    // @Description: Follow target's mavlink system id
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_SYSID", 3, AP_Follow, _sysid, 0),

    // 4 is reserved for MARGIN parameter

    // @Param: _DIST_MAX
    // @DisplayName: Follow distance maximum
    // @Description: Follow distance maximum.  targets further than this will be ignored
    // @Units: m
    // @Range: 1 10000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 1000),

    // @Param: _OFS_TYPE
    // @DisplayName: Follow offset type
    // @Description: Follow offset type
    // @Values: 0:North-East-Down, 1:Relative to lead vehicle heading
    // @User: Standard
    AP_GROUPINFO("_OFS_TYPE", 6, AP_Follow, _offset_type, AP_FOLLOW_OFFSET_TYPE_NED),

    // @Param: _OFS_X
    // @DisplayName: Follow offsets in meters north/forward
    // @Description: Follow offsets in meters north/forward.  If positive, this vehicle fly ahead or north of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Y
    // @DisplayName: Follow offsets in meters east/right
    // @Description: Follow offsets in meters east/right.  If positive, this vehicle will fly to the right or east of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Z
    // @DisplayName: Follow offsets in meters down
    // @Description: Follow offsets in meters down.  If positive, this vehicle will fly below the lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset, 0),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover))
    // @Param: _YAW_BEHAVE
    // @DisplayName: Follow yaw behaviour
    // @Description: Follow yaw behaviour
    // @Values: 0:None,1:Face Lead Vehicle,2:Same as Lead vehicle,3:Direction of Flight
    // @User: Standard
    AP_GROUPINFO("_YAW_BEHAVE", 8, AP_Follow, _yaw_behave, 1),
#endif

    // @Param: _POS_P
    // @DisplayName: Follow position error P gain
    // @Description: Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 9, AP_Follow, AC_P),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover)) 
    // @Param: _ALT_TYPE
    // @DisplayName: Follow altitude type
    // @Description: Follow altitude type
    // @Values: 0:absolute, 1:relative
    // @User: Standard
    AP_GROUPINFO("_ALT_TYPE", 10, AP_Follow, _alt_type, AP_FOLLOW_ALT_TYPE_DEFAULT),
#endif

    // @Param: _OPTIONS
    // @DisplayName: Follow options
    // @Description: Follow options bitmask
    // @Values: 0:None,1: Mount Follows lead vehicle on mode enter
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 11, AP_Follow, _options, 0),

    // @Param: _SRC
    // @DisplayName: Follow vehicle information source
    // @Description: Follow vehicle information source
    // @Values: 0:MAVLINK SYSID, 1:LocationDB Key
    // @User: Standard
    AP_GROUPINFO("_SRC", 12, AP_Follow, _src, 0),

    // @Param: _DBITM_KEY
    // @DisplayName: LocationDB item key for the object to be followed
    // @Description: LocationDB item key for the object to be followed
    // @Range: 0 4294967295
    // @User: Standard
    AP_GROUPINFO("_DBITM_KEY", 13, AP_Follow, _locdb_key_param, 0),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow::AP_Follow() :
        _p_pos(AP_FOLLOW_POS_P_DEFAULT)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Follow::update() {
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    if (!have_target_key() || target_mismatch()) {
        refresh_target_key();
        return;
    }

    // get estimated location and velocity
    Location loc_estimate{};
    Vector3f vel_estimate;
    AP_LocationDB_Item target;
    Vector3f target_pos_NEU;
    Vector3f target_vel_NEU;
    if (!AP::locationdb()->get_item(_locdb_key, target) ||
         !target.get_pos_cm_NEU(target_pos_NEU) ||
         !target.get_vel_cm_NEU(target_vel_NEU) ||
         !get_target_location_and_velocity(loc_estimate, vel_estimate)) {
        // not able to retrive information about the target
        // try resetting it
        _locdb_key = 0;
        return;
    }
    Location target_location(target_pos_NEU, Location::AltFrame::ABOVE_ORIGIN);
    IGNORE_RETURN(target_location.change_alt_frame(Location::AltFrame::ABSOLUTE));

    // log lead's estimated vs reported position
// @LoggerMessage: FOLL
// @Description: Follow library diagnostic data
// @Field: TimeUS: Time since system startup
// @Field: Lat: Target latitude
// @Field: Lon: Target longitude
// @Field: Alt: Target absolute altitude
// @Field: VelN: Target earth-frame velocity, North
// @Field: VelE: Target earth-frame velocity, East
// @Field: VelD: Target earth-frame velocity, Down
// @Field: LatE: Vehicle latitude
// @Field: LonE: Vehicle longitude
// @Field: AltE: Vehicle absolute altitude
    AP::logger().WriteStreaming("FOLL",
                                "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE",  // labels
                                "sDUmnnnDUm",    // units
                                "F--B000--B",    // mults
                                "QLLifffLLi",    // fmt
                                AP_HAL::micros64(),
                                target_location.lat,
                                target_location.lng,
                                target_location.alt,
                                (double)target_vel_NEU.x,
                                (double)target_vel_NEU.y,
                                (double)-target_vel_NEU.z,
                                loc_estimate.lat,
                                loc_estimate.lng,
                                loc_estimate.alt
                                );
}

// restore offsets to zero if necessary, should be called when vehicle exits follow mode
void AP_Follow::clear_offsets_if_required()
{
    if (_offsets_were_zero) {
        _offset.set(Vector3f());
    }
    _offsets_were_zero = false;
}

// reconstruct target key from parameters
void AP_Follow::refresh_target_key()
{
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    switch(_src.get()) {
    case uint8_t(TargetSource::MAVLINK_ONLY) :
    {
        uint8_t sysid = _sysid.get();
        // check if we are getting FOLLOW_TARGET or GLOBAL_POSITION_INT message from any of the componenets of given sysid
        for (uint16_t compid = 0; compid <= 255U; compid++) {
            if (AP::locationdb()->item_exists(AP_LocationDB::construct_key_mavlink(sysid, compid, MAVLINK_MSG_ID_FOLLOW_TARGET))) {
                // we are getting a FOLLOW_TARGET message
                _locdb_key = AP_LocationDB::construct_key_mavlink(sysid, compid, MAVLINK_MSG_ID_FOLLOW_TARGET);
                return;
            } else if (AP::locationdb()->item_exists(AP_LocationDB::construct_key_mavlink(sysid, compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT))) {
                // we are getting a GLOBAL_POSITION_INT message
                _locdb_key = AP_LocationDB::construct_key_mavlink(sysid, compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
                return;
            }
        }
        break;
    }
    case uint8_t(TargetSource::LOCATIONDB):
        if (AP::locationdb()->item_exists(_locdb_key_param.get())) {
            _locdb_key = _locdb_key_param.get();
            return;
        }
        break;
    };

    // no key constructed
    _locdb_key = 0;
}

// return true if we are following a target other than the specified one
bool AP_Follow::target_mismatch() {
    switch(_src.get()) {
    case uint8_t(TargetSource::MAVLINK_ONLY) :
        // location db keys for mavlink items have structure: DOMAIN (8 bits) | SYSID (8 bits) | COMPID (8bits) | SHORT MSD ID (8 bits)
        // we check if the set key has matching domain and sysid with the target we are following
        if ((uint8_t(_locdb_key >> 24) != uint8_t(AP_LocationDB::KeyDomain::MAVLINK)) || (uint8_t(_locdb_key >> 16) != _sysid.get())) {
            return true;
        }
        break;
    case uint8_t(TargetSource::LOCATIONDB):
        if (_locdb_key != uint32_t(_locdb_key_param.get())) {
            return true;
        }
        break;
    };

    return false;
}

// returns true if we have the target key
bool AP_Follow::have_target_key() const {
    if (_locdb_key == 0) {
        return false;
    }

    return true;
}

// get target's estimated position and velocity (in NED)
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) const {
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    Vector3f target_pos_ned_m;
    Vector3f target_vel_ned_m;

    if (!get_target_position_and_velocity(target_pos_ned_m, target_vel_ned_m)) {
        return false;
    }

    // Location() accepts ekf offset in NEU cms
    target_pos_ned_m = target_pos_ned_m * 100;
    target_pos_ned_m.z = -target_pos_ned_m.z;

    loc = Location(target_pos_ned_m, Location::AltFrame::ABOVE_ORIGIN);

    if (!loc.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return false;
    }

    return true;
}

// get target's estimated position and velocity
bool AP_Follow::get_target_position_and_velocity(Vector3f &pos_ned_m, Vector3f &vel_ned) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // return false if we do not have the target key or we fail to retrive the target with that key
    AP_LocationDB_Item target;
    if (!have_target_key() || !AP::locationdb()->get_item(_locdb_key, target)) {
        return false;
    }

    // check for timeout
    if (AP_HAL::millis() - target.get_timestamp_ms() > AP_FOLLOW_TIMEOUT_MS) {
        return false;
    }

    // calculate time since last actual position update
    const float dt = (AP_HAL::millis() - target.get_timestamp_ms()) * 0.001f;

    Vector3f target_pos, target_vel, target_accel;
    if (!target.get_pos_cm_NEU(target_pos) || !target.get_vel_cm_NEU(target_vel)) {
        return false;
    }

    // calculate velocity estimate if we are able to get target acceleration
    if (target.get_acc_cm_NEU(target_accel)) {
        target_vel += (target_accel * dt);
    }

    // project the vehicle position
    target_pos += (target_vel * dt);

    // NEU to NED
    target_pos.z = -target_pos.z;
    target_vel.z = -target_vel.z;

    // return latest position estimate
    pos_ned_m = target_pos * 0.01; // in m
    vel_ned = target_vel * 0.01; // in m
    return true;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
bool AP_Follow::get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // get our location
    Vector3f our_pos_from_origin_ned;
    if (!AP::ahrs().get_relative_position_NED_origin(our_pos_from_origin_ned)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // get target location and velocity
    Vector3f veh_pos_ned;
    Vector3f veh_vel_ned;
    if (!get_target_position_and_velocity(veh_pos_ned, veh_vel_ned)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // calculate difference
    const Vector3f dist_vec = veh_pos_ned - our_pos_from_origin_ned;

    // fail if too far
    if (is_positive(_dist_max.get()) && (dist_vec.length() > _dist_max)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // initialise offsets from distance vector if required
    init_offsets_if_required(dist_vec);

    // get offsets
    Vector3f offsets;
    if (!get_offsets_ned(offsets)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // calculate results
    dist_ned = dist_vec;
    dist_with_offs = dist_vec + offsets;
    vel_ned = veh_vel_ned;

    // record distance and heading for reporting purposes
    if (is_zero(dist_with_offs.x) && is_zero(dist_with_offs.y)) {
        clear_dist_and_bearing_to_target();
    } else {
        _dist_to_target = safe_sqrt(sq(dist_with_offs.x) + sq(dist_with_offs.y));
        _bearing_to_target = degrees(atan2f(dist_with_offs.y, dist_with_offs.x));
    }

    return true;
}

// get target's heading in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading_deg(float &heading) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    AP_LocationDB_Item target;
    if (!have_target_key() || !AP::locationdb()->get_item(_locdb_key, target)) {
        return false;
    }

    // check for timeout
    if (AP_HAL::millis() - target.get_timestamp_ms() > AP_FOLLOW_TIMEOUT_MS) {
        return false;
    }

    float target_heading;
    if (!target.get_heading_cdeg(target_heading) || target_heading > 36000) {
        return false;
    }

    // return heading estimate in degrees
    heading = target_heading * 0.01f;
    return true;
}

// initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if required
void AP_Follow::init_offsets_if_required(const Vector3f &dist_vec_ned)
{
    // return immediately if offsets have already been set
    if (!_offset.get().is_zero()) {
        return;
    }
    _offsets_were_zero = true;

    float target_heading_deg;
    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) && get_target_heading_deg(target_heading_deg)) {
        // rotate offsets from north facing to vehicle's perspective
        _offset.set(rotate_vector(-dist_vec_ned, -target_heading_deg));
        gcs().send_text(MAV_SEVERITY_INFO, "Relative follow offset loaded");
    } else {
        // initialise offset in NED frame
        _offset.set(-dist_vec_ned);
        // ensure offset_type used matches frame of offsets saved
        _offset_type.set(AP_FOLLOW_OFFSET_TYPE_NED);
        gcs().send_text(MAV_SEVERITY_INFO, "N-E-D follow offset loaded");
    }
}

// get offsets in meters in NED frame
bool AP_Follow::get_offsets_ned(Vector3f &offset) const
{
    const Vector3f &off = _offset.get();

    // if offsets are zero or type is NED, simply return offset vector
    if (off.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        offset = off;
        return true;
    }

    // offset type is relative, exit if we cannot get vehicle's heading
    float target_heading_deg;
    if (!get_target_heading_deg(target_heading_deg)) {
        return false;
    }

    // rotate offsets from vehicle's perspective to NED
    offset = rotate_vector(off, target_heading_deg);
    return true;
}

// rotate 3D vector clockwise by specified angle (in degrees)
Vector3f AP_Follow::rotate_vector(const Vector3f &vec, float angle_deg) const
{
    // rotate roll, pitch input from north facing to vehicle's perspective
    const float cos_yaw = cosf(radians(angle_deg));
    const float sin_yaw = sinf(radians(angle_deg));
    return Vector3f((vec.x * cos_yaw) - (vec.y * sin_yaw), (vec.y * cos_yaw) + (vec.x * sin_yaw), vec.z);
}

// set recorded distance and bearing to target to zero
void AP_Follow::clear_dist_and_bearing_to_target()
{
    _dist_to_target = 0.0f;
    _bearing_to_target = 0.0f;
}

// get target's estimated location and velocity (in NED), with offsets added
bool AP_Follow::get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned) const
{
    Vector3f ofs;
    if (!get_offsets_ned(ofs) ||
        !get_target_location_and_velocity(loc, vel_ned)) {
        return false;
    }
    // apply offsets
    loc.offset(ofs.x, ofs.y);
    loc.alt -= ofs.z*100;
    return true;
}

// return true if we have a target
bool AP_Follow::have_target(void) const
{
    if (!_enabled) {
        return false;
    }

    AP_LocationDB_Item target;
    if (!AP::locationdb()->get_item(_locdb_key, target)) {
        return false;
    }

    // check for timeout
    if (AP_HAL::millis() - target.get_timestamp_ms() > AP_FOLLOW_TIMEOUT_MS) {
        return false;
    }
    return true;
}

uint32_t AP_Follow::get_last_update_ms() const {
    AP_LocationDB_Item target;
    if (AP::locationdb() && AP::locationdb()->get_item(_locdb_key, target)) {
        return target.get_timestamp_ms();
    }

    // target not retrieved
    return 0;
}

namespace AP {

AP_Follow &follow()
{
    return *AP_Follow::get_singleton();
}

}

#endif
