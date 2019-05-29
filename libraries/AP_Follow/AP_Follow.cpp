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

#include <AP_HAL/AP_HAL.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // position estimate timeout after 1 second
#define AP_FOLLOW_SYSID_TIMEOUT_MS 10000 // forget sysid we are following if we haave not heard from them in 10 seconds

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // offsets are in north-east-down frame
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  1   // offsets are relative to lead vehicle's heading

#define AP_FOLLOW_ALTITUDE_TYPE_RELATIVE  1 // relative altitude is used by default   

#define AP_FOLLOW_POS_P_DEFAULT 0.1f    // position error gain default

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
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 100),

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

    // @Param: _YAW_BEHAVE
    // @DisplayName: Follow yaw behaviour
    // @Description: Follow yaw behaviour
    // @Values: 0:None,1:Face Lead Vehicle,2:Same as Lead vehicle,3:Direction of Flight
    // @User: Standard
    AP_GROUPINFO("_YAW_BEHAVE", 8, AP_Follow, _yaw_behave, 1),

    // @Param: _POS_P
    // @DisplayName: Follow position error P gain
    // @Description: Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 9, AP_Follow, AC_P),

    // @Param: _ALT_TYPE
    // @DisplayName: Follow altitude type
    // @Description: Follow altitude type
    // @Values: 0:absolute, 1: relative
    // @User: Standard
    AP_GROUPINFO("_ALT_TYPE", 10, AP_Follow, _alt_type, AP_FOLLOW_ALTITUDE_TYPE_RELATIVE),

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
    AP_Param::setup_object_defaults(this, var_info);
}

// get target's estimated location
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // calculate time since last actual position update
    const float dt = (AP_HAL::millis() - _last_location_update_ms) * 0.001f;

    // get velocity estimate
    if (!get_velocity_ned(vel_ned, dt)) {
        return false;
    }

    // project the vehicle position
    Location last_loc = _target_location;
    last_loc.offset(vel_ned.x * dt, vel_ned.y * dt);
    last_loc.alt -= vel_ned.z * 100.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    loc = last_loc;
    return true;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
bool AP_Follow::get_target_dist_and_vel_ned(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    // get our location
    Location current_loc;
    if (!AP::ahrs().get_position(current_loc)) {
        clear_dist_and_bearing_to_target();
         return false;
    }

    // get target location and velocity
    Location target_loc;
    Vector3f veh_vel;
    if (!get_target_location_and_velocity(target_loc, veh_vel)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // change to altitude above home if relative altitude is being used
    if (target_loc.relative_alt == 1) {
        current_loc.alt -= AP::ahrs().get_home().alt;
    }

    // calculate difference
    const Vector3f dist_vec = current_loc.get_distance_NED(target_loc);

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
    vel_ned = veh_vel;

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

    // check for timeout
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // return latest heading estimate
    heading = _target_heading;
    return true;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    // skip our own messages
    if (msg.sysid == mavlink_system.sysid) {
        return;
    }

    // skip message if not from our target
    if (_sysid != 0 && msg.sysid != _sysid) {
        if (_automatic_sysid) {
            // maybe timeout who we were following...
            if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_SYSID_TIMEOUT_MS)) {
                _sysid.set(0);
            }
        }
        return;
    }

    // decode global-position-int message
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        // get estimated location and velocity (for logging)
        Location loc_estimate{};
        Vector3f vel_estimate;
        UNUSED_RESULT(get_target_location_and_velocity(loc_estimate, vel_estimate));

        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        // ignore message if lat and lon are (exactly) zero
        if ((packet.lat == 0 && packet.lon == 0)) {
            return;
        }

        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;

        // select altitude source based on FOLL_ALT_TYPE param 
        if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
            // relative altitude
            _target_location.alt = packet.relative_alt / 10;        // convert millimeters to cm
            _target_location.relative_alt = 1;                // set relative_alt flag
        } else {
            // absolute altitude
            _target_location.alt = packet.alt / 10;                 // convert millimeters to cm
            _target_location.relative_alt = 0;                // reset relative_alt flag
        }

        _target_velocity_ned.x = packet.vx * 0.01f; // velocity north
        _target_velocity_ned.y = packet.vy * 0.01f; // velocity east
        _target_velocity_ned.z = packet.vz * 0.01f; // velocity down

        // get a local timestamp with correction for transport jitter
        _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_boot_ms, AP_HAL::millis());
        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            _target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
            _last_heading_update_ms = _last_location_update_ms;
        }
        // initialise _sysid if zero to sender's id
        if (_sysid == 0) {
            _sysid.set(msg.sysid);
            _automatic_sysid = true;
        }

        // log lead's estimated vs reported position
        AP::logger().Write("FOLL",
                                               "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE",  // labels
                                               "sDUmnnnDUm",    // units
                                               "F--B000--B",    // mults
                                               "QLLifffLLi",    // fmt
                                               AP_HAL::micros64(),
                                               _target_location.lat,
                                               _target_location.lng,
                                               _target_location.alt,
                                               (double)_target_velocity_ned.x,
                                               (double)_target_velocity_ned.y,
                                               (double)_target_velocity_ned.z,
                                               loc_estimate.lat,
                                               loc_estimate.lng,
                                               loc_estimate.alt
                                               );
    }
}

// get velocity estimate in m/s in NED frame using dt since last update
bool AP_Follow::get_velocity_ned(Vector3f &vel_ned, float dt) const
{
    vel_ned = _target_velocity_ned + (_target_accel_ned * dt);
    return true;
}

// initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if required
void AP_Follow::init_offsets_if_required(const Vector3f &dist_vec_ned)
{
    // return immediately if offsets have already been set
    if (!_offset.get().is_zero()) {
        return;
    }

    float target_heading_deg;
    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) && get_target_heading_deg(target_heading_deg)) {
        // rotate offsets from north facing to vehicle's perspective
        _offset = rotate_vector(-dist_vec_ned, -target_heading_deg);
    } else {
        // initialise offset in NED frame
        _offset = -dist_vec_ned;
        // ensure offset_type used matches frame of offsets saved
        _offset_type = AP_FOLLOW_OFFSET_TYPE_NED;
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
