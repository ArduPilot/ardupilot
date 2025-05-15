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

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // position estimate timeout after 1 second
#define AP_FOLLOW_SYSID_TIMEOUT_MS 10000 // forget sysid we are following if we have not heard from them in 10 seconds

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
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max_m, 100),

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
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset_ned_m, 0),

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

// restore offsets to zero if necessary, should be called when vehicle exits follow mode
void AP_Follow::clear_offsets_if_required()
{
    if (_offsets_were_zero) {
        _offset_ned_m.set(Vector3f());
        _offsets_were_zero = false;
    }
}

// get target's estimated location
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned_ms) const
{
    Vector3p pos_ned_m;
    Vector3f local_vel_ned_m;  // so we either change both return parameters or neither
    if (!get_target_position_and_velocity_NED_m(pos_ned_m, local_vel_ned_m)) {
        return false;
    }
    if (!AP::ahrs().get_location_from_origin_offset_NED(loc, pos_ned_m)) {
        return false;
    }
    vel_ned_ms = local_vel_ned_m;

    if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
        loc.change_alt_frame(Location::AltFrame::ABOVE_HOME);
    }

    return true;
}

// get target's estimated location and velocity, both NED SI from origin
bool AP_Follow::get_target_position_and_velocity_NED_m(Vector3p &pos_ned_m, Vector3f &vel_ned_ms) const
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
    if (!get_velocity_NED_ms(vel_ned_ms, dt)) {
        return false;
    }

    // project the vehicle position
    const Vector3p vel_ned_p { vel_ned_ms.x, vel_ned_ms.y, vel_ned_ms.z };
    pos_ned_m = _target_position_ned_m + vel_ned_p * dt;

    return true;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
bool AP_Follow::get_target_dist_and_vel_NED_m(Vector3f &dist_ned_m, Vector3f &dist_with_offs_m, Vector3f &vel_ned_ms)
{
    Vector3f current_position_NED;
    if (!AP::ahrs().get_relative_position_NED_origin(current_position_NED)) {
        clear_dist_and_bearing_to_target();
        return false;
    }
    Vector3p target_position_ned_m;
    Vector3f veh_vel_ned_ms;  // NED
    if (!get_target_position_and_velocity_NED_m(target_position_ned_m, veh_vel_ned_ms)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // convert from Vector3p to Vector3f as they can't be subtracted:
    Vector3f target_position_NED_f;
    target_position_NED_f.x = target_position_ned_m.x;
    target_position_NED_f.y = target_position_ned_m.y;
    target_position_NED_f.z = target_position_ned_m.z;

    const Vector3f dist_vec_ned_m = target_position_NED_f - current_position_NED;

    // fail if too far
    if (is_positive(_dist_max_m.get()) && (dist_vec_ned_m.length() > _dist_max_m)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // initialise offsets_ned_m from distance vector if required
    init_offsets_if_required(dist_vec_ned_m);

    // get offsets_ned_m
    Vector3f offsets_ned_m;
    if (!get_offsets_NED_m(offsets_ned_m)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // calculate results
    dist_ned_m = dist_vec_ned_m;
    dist_with_offs_m = dist_vec_ned_m + offsets_ned_m;
    vel_ned_ms = veh_vel_ned_ms;

    // record distance and heading_deg for reporting purposes
    if (is_zero(dist_with_offs_m.x) && is_zero(dist_with_offs_m.y)) {
        clear_dist_and_bearing_to_target();
    } else {
        _dist_to_target_m = safe_sqrt(sq(dist_with_offs_m.x) + sq(dist_with_offs_m.y));
        _bearing_to_target_deg = degrees(atan2f(dist_with_offs_m.y, dist_with_offs_m.x));
    }

    return true;
}

// get target's heading_deg in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading_deg(float &heading_deg) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // return latest heading_deg estimate
    heading_deg = _target_heading_deg;
    return true;
}

// returns true if we should extract information from msg
bool AP_Follow::should_handle_message(const mavlink_message_t &msg) const
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // skip our own messages
    if (msg.sysid == mavlink_system.sysid) {
        return false;
    }

    // skip message if not from our target
    if (_sysid != 0 && msg.sysid != _sysid) {
        return false;
    }

    return true;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // this method should be called from an "update()" method:
    if (_automatic_sysid) {
        // maybe timeout who we were following...
        if ((_last_location_update_ms == 0) ||
            (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_SYSID_TIMEOUT_MS)) {
            _sysid.set(0);
        }
    }

    if (!should_handle_message(msg)) {
        return;
    }

    // decode global-position-int message
    bool updated = false;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        updated = handle_global_position_int_message(msg);
        break;
    }
    case MAVLINK_MSG_ID_FOLLOW_TARGET: {
        updated = handle_follow_target_message(msg);
        break;
    }
    }

    if (updated) {
#if HAL_LOGGING_ENABLED
        Log_Write_FOLL();
#endif
    }
}

bool AP_Follow::handle_global_position_int_message(const mavlink_message_t &msg)
{
        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        // ignore message if lat and lon are (exactly) zero
        if ((packet.lat == 0 && packet.lon == 0)) {
            return false;
        }

        Location _target_location;
        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;

        // select altitude source based on FOLL_ALT_TYPE param 
        if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
            // above home alt
            _target_location.set_alt_cm(packet.relative_alt / 10, Location::AltFrame::ABOVE_HOME);
        } else {
            // absolute altitude
            _target_location.set_alt_cm(packet.alt / 10, Location::AltFrame::ABSOLUTE);
        }
        if (!_target_location.get_vector_from_origin_NEU(_target_position_ned_m)) {
            return false;
        }
        _target_position_ned_m.z = -_target_position_ned_m.z; // NEU->NED
        _target_position_ned_m *= 0.01;  // cm -> m

        _target_velocity_ned_ms.x = packet.vx * 0.01f; // velocity north
        _target_velocity_ned_ms.y = packet.vy * 0.01f; // velocity east
        _target_velocity_ned_ms.z = packet.vz * 0.01f; // velocity down

        // get a local timestamp with correction for transport jitter
        _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_boot_ms, AP_HAL::millis());
        if (packet.hdg <= 36000) {                  // heading_deg (UINT16_MAX if unknown)
            _target_heading_deg = packet.hdg * 0.01f;   // convert centi-degrees to degrees
            _last_heading_update_ms = _last_location_update_ms;
        }
        // initialise _sysid if zero to sender's id
        if (_sysid == 0) {
            _sysid.set(msg.sysid);
            _automatic_sysid = true;
        }
        return true;
}

bool AP_Follow::handle_follow_target_message(const mavlink_message_t &msg)
{
        // decode message
        mavlink_follow_target_t packet;
        mavlink_msg_follow_target_decode(&msg, &packet);

        // ignore message if lat and lon are (exactly) zero
        if ((packet.lat == 0 && packet.lon == 0)) {
            return false;
        }
        // require at least position
        if ((packet.est_capabilities & (1<<0)) == 0) {
            return false;
        }

        const Location new_loc {
            packet.lat,
            packet.lon,
            int32_t(packet.alt * 100),  // m -> cm
            Location::AltFrame::ABSOLUTE
        };

        if (!new_loc.get_vector_from_origin_NEU(_target_position_ned_m)) {
            return false;
        }
        _target_position_ned_m.z = -_target_position_ned_m.z; // NEU->NED
        _target_position_ned_m *= 0.01;  // cm -> m

        if (packet.est_capabilities & (1<<1)) {
            _target_velocity_ned_ms.x = packet.vel[0]; // velocity north
            _target_velocity_ned_ms.y = packet.vel[1]; // velocity east
            _target_velocity_ned_ms.z = packet.vel[2]; // velocity down
        } else {
            _target_velocity_ned_ms.zero();
        }

        // get a local timestamp with correction for transport jitter
        _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.timestamp, AP_HAL::millis());

        if (packet.est_capabilities & (1<<3)) {
            Quaternion q{packet.attitude_q[0], packet.attitude_q[1], packet.attitude_q[2], packet.attitude_q[3]};
            float r, p, y;
            q.to_euler(r,p,y);
            _target_heading_deg = degrees(y);
            _last_heading_update_ms = _last_location_update_ms;
        }

        // initialise _sysid if zero to sender's id
        if (_sysid == 0) {
            _sysid.set(msg.sysid);
            _automatic_sysid = true;
        }

        return true;
}

// write out an onboard-log message to help diagnose follow problems:
#if HAL_LOGGING_ENABLED
void AP_Follow::Log_Write_FOLL()
{
        // get estimated location and velocity
        Location loc_estimate{};
        Vector3f vel_estimate_ned_ms;
        UNUSED_RESULT(get_target_location_and_velocity(loc_estimate, vel_estimate_ned_ms));

        Location _target_location;
        UNUSED_RESULT(AP::ahrs().get_location_from_origin_offset_NED(_target_location, _target_position_ned_m * 0.01));
        if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
            _target_location.change_alt_frame(Location::AltFrame::ABOVE_HOME);
        }

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
                                               _target_location.lat,
                                               _target_location.lng,
                                               _target_location.alt,
                                               (double)_target_velocity_ned_ms.x,
                                               (double)_target_velocity_ned_ms.y,
                                               (double)_target_velocity_ned_ms.z,
                                               loc_estimate.lat,
                                               loc_estimate.lng,
                                               loc_estimate.alt
                                               );
}
#endif  // HAL_LOGGING_ENABLED

// get velocity estimate in m/s in NED frame using dt since last update
bool AP_Follow::get_velocity_NED_ms(Vector3f &vel_ned_ms, float dt) const
{
    vel_ned_ms = _target_velocity_ned_ms + (_target_accel_ned_mss * dt);
    return true;
}

// initialise offsets_ned_m to provided distance vector to other vehicle (in meters in NED frame) if required
void AP_Follow::init_offsets_if_required(const Vector3f &dist_vec_ned)
{
    // return immediately if offsets_ned_m have already been set
    if (!_offset_ned_m.get().is_zero()) {
        return;
    }
    _offsets_were_zero = true;

    float target_heading_deg;
    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) && get_target_heading_deg(target_heading_deg)) {
        // rotate offsets_ned_m from north facing to vehicle's perspective
        _offset_ned_m.set(rotate_vector(-dist_vec_ned, -target_heading_deg));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Relative follow offset loaded");
    } else {
        // initialise offset in NED frame
        _offset_ned_m.set(-dist_vec_ned);
        // ensure offset_type used matches frame of offsets_ned_m saved
        _offset_type.set(AP_FOLLOW_OFFSET_TYPE_NED);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "N-E-D follow offset loaded");
    }
}

// get offsets_ned_m in meters in NED frame
bool AP_Follow::get_offsets_NED_m(Vector3f &offset) const
{
    const Vector3f &off = _offset_ned_m.get();

    // if offsets_ned_m are zero or type is NED, simply return offset vector
    if (off.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        offset = off;
        return true;
    }

    // offset type is relative, exit if we cannot get vehicle's heading_deg
    float target_heading_deg;
    if (!get_target_heading_deg(target_heading_deg)) {
        return false;
    }

    // rotate offsets_ned_m from vehicle's perspective to NED
    offset = rotate_vector(off, target_heading_deg);
    return true;
}

// rotate 3D vector clockwise by specified angle (in degrees)
Vector3f AP_Follow::rotate_vector(const Vector3f &vec, float angle_deg) const
{
    // rotate roll, pitch input from north facing to vehicle's perspective
    Vector3f ret = vec;
    ret.xy().rotate(radians(angle_deg));

    return ret;
}

// set recorded distance and bearing to target to zero
void AP_Follow::clear_dist_and_bearing_to_target()
{
    _dist_to_target_m = 0.0f;
    _bearing_to_target_deg = 0.0f;
}

// get target's estimated origin-relative-position and velocity (both in SI NED), with offsets_ned_m added
bool AP_Follow::get_target_position_and_velocity_ofs_NED_m(Vector3p &pos_ned_m, Vector3f &vel_ned_ms) const
{
    Vector3f ofs_ned;
    if (!get_offsets_NED_m(ofs_ned)) {
        return false;
    }
    if (!get_target_position_and_velocity_NED_m(pos_ned_m, vel_ned_ms)) {
        return false;
    }
    // can't add Vector3p and Vector3f directly:
    pos_ned_m.x += ofs_ned.x;
    pos_ned_m.y += ofs_ned.y;
    pos_ned_m.z += ofs_ned.z;
    return true;
}

// get target's estimated location and velocity (in NED), with offsets_ned_m added
bool AP_Follow::get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned_ms) const
{
    Vector3p pos_ned_m;
    Vector3f local_vel_ned_m;  // so we either change both return parameters or neither
    if (!get_target_position_and_velocity_ofs_NED_m(pos_ned_m, local_vel_ned_m)) {
        return false;
    }
    if (!AP::ahrs().get_location_from_origin_offset_NED(loc, pos_ned_m)) {
        return false;
    }
    if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
        loc.change_alt_frame(Location::AltFrame::ABOVE_HOME);
    }

    vel_ned_ms = local_vel_ned_m;
    return true;
}

// return true if we have a target
bool AP_Follow::have_target(void) const
{
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

namespace AP {

AP_Follow &follow()
{
    return *AP_Follow::get_singleton();
}

}

#endif  // AP_FOLLOW_ENABLED
