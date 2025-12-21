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

//==============================================================================
// AP_Follow Library
// Target Following Logic for ArduPilot Vehicles
//==============================================================================


//==============================================================================
// Includes
//==============================================================================

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
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;


//==============================================================================
// Constants and Definitions
//==============================================================================

#define AP_FOLLOW_TIMEOUT          3     // position estimate timeout in seconds
#define AP_FOLLOW_SYSID_TIMEOUT_MS 10000 // forget sysid we are following if we have not heard from them in 10 seconds

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // offsets are in north-east-down frame
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  1   // offsets are relative to lead vehicle's heading

#define AP_FOLLOW_POS_P_DEFAULT 0.1f    // position error gain default

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 #define AP_FOLLOW_ALT_TYPE_DEFAULT static_cast<float>(Location::AltFrame::ABSOLUTE)
 #define AP_FOLLOW_DIST_MAX_DEFAULT 0    // zero = ignored
#else
 #define AP_FOLLOW_ALT_TYPE_DEFAULT static_cast<float>(Location::AltFrame::ABOVE_HOME)
 #define AP_FOLLOW_DIST_MAX_DEFAULT 100
#endif


//==============================================================================
// Constructor
//==============================================================================

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
    // @Description: Follow distance maximum. If exceeded, the follow estimate will be considered invalid. If zero there is no maximum.
    // @Units: m
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max_m, AP_FOLLOW_DIST_MAX_DEFAULT),

    // @Param: _OFS_TYPE
    // @DisplayName: Follow offset type
    // @Description: Follow offset type
    // @Values: 0:North-East-Down, 1:Relative to lead vehicle heading
    // @User: Standard
    AP_GROUPINFO("_OFS_TYPE", 6, AP_Follow, _offset_type, AP_FOLLOW_OFFSET_TYPE_NED),

    // @Param: _OFS_X
    // @DisplayName: Follow offsets in meters north/forward
    // @Description: Follow offsets in meters north/forward. If positive, this vehicle fly ahead or north of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Y
    // @DisplayName: Follow offsets in meters east/right
    // @Description: Follow offsets in meters east/right. If positive, this vehicle will fly to the right or east of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Z
    // @DisplayName: Follow offsets in meters down
    // @Description: Follow offsets in meters down. If positive, this vehicle will fly below the lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset_m, 0),

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
    // @Description: Follow position error P gain. Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 9, AP_Follow, AC_P),

#if !(APM_BUILD_TYPE(APM_BUILD_Rover)) 
    // @Param: _ALT_TYPE
    // @DisplayName: Follow altitude type
    // @Description: Follow altitude type
    // @Values: 0:absolute, 1:relative, 3:terrain
    // @User: Standard
    AP_GROUPINFO("_ALT_TYPE", 10, AP_Follow, _alt_type, AP_FOLLOW_ALT_TYPE_DEFAULT),
#endif

    // @Param: _OPTIONS
    // @DisplayName: Follow options
    // @Description: Follow options bitmask
    // @Values: 0:None,1: Mount Follows lead vehicle on mode enter
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 11, AP_Follow, _options, 0),

    // @Param: _ACCEL_NE
    // @DisplayName: Acceleration limit for the horizontal kinematic input shaping
    // @Description: Acceleration limit of the horizontal kinematic path generation used to determine how quickly the estimate varies in velocity
    // @Range: 0 5
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("_ACCEL_NE", 12, AP_Follow, _accel_max_ne_mss, 2.5),

    // @Param: _JERK_NE
    // @DisplayName: Jerk limit for the horizontal kinematic input shaping
    // @Description: Jerk limit of the horizontal kinematic path generation used to determine how quickly the estimate varies in acceleration
    // @Range: 0 20
    // @Units: m/s/s/s
    // @User: Advanced
    AP_GROUPINFO("_JERK_NE", 13, AP_Follow, _jerk_max_ne_msss, 5.0),

    // @Param: _ACCEL_D
    // @DisplayName: Acceleration limit for the vertical kinematic input shaping
    // @Description: Acceleration limit of the vertical kinematic path generation used to determine how quickly the estimate varies in velocity
    // @Range: 0 2.5
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("_ACCEL_D", 14, AP_Follow, _accel_max_d_mss, 2.5),

    // @Param: _JERK_D
    // @DisplayName: Jerk limit for the vertical kinematic input shaping
    // @Description: Jerk limit of the vertical kinematic path generation used to determine how quickly the estimate varies in acceleration
    // @Range: 0 5
    // @Units: m/s/s/s
    // @User: Advanced
    AP_GROUPINFO("_JERK_D", 15, AP_Follow, _jerk_max_d_msss, 5.0),

    // @Param: _ACCEL_H
    // @DisplayName: Angular acceleration limit for the heading kinematic input shaping
    // @Description: Angular acceleration limit of the heading kinematic path generation used to determine how quickly the estimate varies in angular velocity
    // @Range: 0 90
    // @Units: deg/s/s
    // @User: Advanced
    AP_GROUPINFO("_ACCEL_H", 16, AP_Follow, _accel_max_h_degss, 90.0),

    // @Param: _JERK_H
    // @DisplayName: Angular jerk limit for the heading kinematic input shaping
    // @Description: Angular jerk limit of the heading kinematic path generation used to determine how quickly the estimate varies in angular acceleration
    // @Range: 0 360
    // @Units: deg/s/s/s
    // @User: Advanced
    AP_GROUPINFO("_JERK_H", 17, AP_Follow, _jerk_max_h_degsss, 360.0),

    // @Param: _TIMEOUT
    // @DisplayName: Follow timeout
    // @Description: Follow position update from lead - timeout after x seconds
    // @User: Standard
    // @Units: s
    AP_GROUPINFO("_TIMEOUT", 18, AP_Follow, _timeout, AP_FOLLOW_TIMEOUT),

    AP_GROUPEND
};

// Constructor for AP_Follow. Initializes the position P-controller and sets up parameter defaults.
AP_Follow::AP_Follow() :
    _p_pos(AP_FOLLOW_POS_P_DEFAULT)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}


//==============================================================================
// Target Estimation Update Functions
//==============================================================================

// Projects and updates the estimated target position, velocity, and heading based on last known data and configured input shaping.
void AP_Follow::update_estimates()
{
    WITH_SEMAPHORE(_follow_sem);

    // check for target: if no valid target, invalidate estimate
    if (!have_target()) {
        clear_dist_and_bearing_to_target();
        _estimate_valid = false;
        return;
    }

    // if sysid changed, reset the estimation state
    if (_sysid != _sysid_used) {
        _sysid_used = _sysid;
        _estimate_valid = false;
    }

    const uint32_t now = AP_HAL::millis();

    // calculate time since last location update in seconds
    const float dt = (now - _last_location_update_ms) * 0.001f;

    // project target's position and velocity forward using simple kinematics
    Vector3f delta_pos_m = _target_vel_ned_ms * dt + _target_accel_ned_mss * 0.5f * sq(dt);
    Vector3f delta_vel_ms = _target_accel_ned_mss * dt;
    float delta_heading_rad = radians(_target_heading_rate_degs) * dt;

    // calculate time since last estimation update
    const float e_dt = (now - _last_estimation_update_ms) * 0.001f;

    const bool valid_kinematic_params = (_accel_max_ne_mss > 0.0f) && (_jerk_max_ne_msss > 0.0f) &&
                                (_accel_max_d_mss > 0.0f) && (_jerk_max_d_msss > 0.0f) &&
                                (_accel_max_h_degss > 0.0f) && (_jerk_max_h_degsss > 0.0f);
    
    if (_estimate_valid && valid_kinematic_params) {
        // update X/Y position, velocity, acceleration with shaping
        update_pos_vel_accel_xy(_estimate_pos_ned_m.xy(), _estimate_vel_ned_ms.xy(), _estimate_accel_ned_mss.xy(), e_dt, Vector2f(), Vector2f(), Vector2f());

        // update Z axis position, velocity, acceleration without shaping (direct update)
        update_pos_vel_accel(_estimate_pos_ned_m.z, _estimate_vel_ned_ms.z, _estimate_accel_ned_mss.z, e_dt, 0.0, 0.0, 0.0);

        // apply horizontal shaping to refine estimate toward projected target state
        shape_pos_vel_accel_xy(_target_pos_ned_m.xy() + delta_pos_m.xy().topostype(), _target_vel_ned_ms.xy() + delta_vel_ms.xy(), _target_accel_ned_mss.xy(),
                               _estimate_pos_ned_m.xy(), _estimate_vel_ned_ms.xy(), _estimate_accel_ned_mss.xy(),
                               0.0, _accel_max_ne_mss, _jerk_max_ne_msss, e_dt, false);

        // apply angular shaping for heading estimate
        shape_angle_vel_accel(radians(_target_heading_deg) + delta_heading_rad, radians(_target_heading_rate_degs), 0.0,
                              _estimate_heading_rad, _estimate_heading_rate_rads, _estimate_heading_accel_radss,
                              0.0, radians(_accel_max_h_degss),
                              radians(_jerk_max_h_degsss), e_dt, false);

        // update heading angle separately to maintain proper wrapping [-PI, PI]
        postype_t estimate_heading_rad = _estimate_heading_rad;
        update_pos_vel_accel(estimate_heading_rad, _estimate_heading_rate_rads, _estimate_heading_accel_radss, e_dt, 0.0, 0.0, 0.0);
        _estimate_heading_rad = wrap_PI(estimate_heading_rad);
    } else {
        // no valid estimate yet: initialise from latest target position
        _estimate_pos_ned_m = _target_pos_ned_m + delta_pos_m.topostype();
        _estimate_vel_ned_ms = _target_vel_ned_ms + delta_vel_ms;
        _estimate_accel_ned_mss = _target_accel_ned_mss;
        _estimate_heading_rad = radians(_target_heading_deg) + delta_heading_rad;  
        _estimate_heading_rate_rads = radians(_target_heading_rate_degs);  
        _estimate_valid = true;
    }

    Vector3f offset_m = _offset_m.get();

    // calculate estimated position and velocity with offsets applied
    if (offset_m.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        // offsets are in NED frame: simple addition
        _ofs_estimate_pos_ned_m = _estimate_pos_ned_m + offset_m.topostype();
        _ofs_estimate_vel_ned_ms = _estimate_vel_ned_ms;
        _ofs_estimate_accel_ned_mss = _estimate_accel_ned_mss;
    } else {
        // offsets are in FRD frame: rotate by heading
        offset_m.xy().rotate(_estimate_heading_rad);
        _ofs_estimate_pos_ned_m = _estimate_pos_ned_m + offset_m.topostype();
        _ofs_estimate_vel_ned_ms = _estimate_vel_ned_ms;
        _ofs_estimate_accel_ned_mss = _estimate_accel_ned_mss;
        // with kinematic shaping of heading we can improve our offset velocity and acceleration of the offset
        if (valid_kinematic_params) {
            Vector3f offset_cross = offset_m.cross(Vector3f{0.0, 0.0, 1.0});
            float offset_length_m = offset_m.length();
            _ofs_estimate_vel_ned_ms += offset_cross * offset_length_m * _estimate_heading_rate_rads;
            _ofs_estimate_accel_ned_mss += offset_cross * offset_length_m * _estimate_heading_accel_radss;
        }
    }

    // update the distance and bearing to the target
    update_dist_and_bearing_to_target();

    _last_estimation_update_ms = now;

    // Check if the target is within the maximum distance
    Vector3p current_position_ned_m;
    if (!AP::ahrs().get_relative_position_NED_origin(current_position_ned_m)) {
        // no idea where we are; knowing where other things are won't help.
        _estimate_valid = false;
        return;
    }
    const Vector3p dist_vec_ned_m = _target_pos_ned_m - current_position_ned_m;
    // If _dist_max_m is not positive, we don't check the distance
    if (is_positive(_dist_max_m.get()) && (dist_vec_ned_m.length() > _dist_max_m)) {
        // target is too far away, mark the estimate invalid
        _estimate_valid = false;
    }
}


//==============================================================================
// Target Information Retrieval Functions
//==============================================================================

// Retrieves the estimated target position, velocity, and acceleration in the NED frame (relative to origin).
bool AP_Follow::get_target_pos_vel_accel_NED_m(Vector3p &pos_ned_m, Vector3f &vel_ned_ms, Vector3f &accel_ned_mss) const
{
    if (!_estimate_valid) {
        return false;
    }

    pos_ned_m = _estimate_pos_ned_m;
    vel_ned_ms = _estimate_vel_ned_ms;
    accel_ned_mss = _estimate_accel_ned_mss;

    return true;
}

// Retrieves the estimated target position, velocity, and acceleration in the NED frame, including configured offsets.
bool AP_Follow::get_ofs_pos_vel_accel_NED_m(Vector3p &pos_ofs_ned_m, Vector3f &vel_ofs_ned_ms, Vector3f &accel_ofs_ned_mss) const
{
    if (!_estimate_valid) {
        return false;
    }

    pos_ofs_ned_m = _ofs_estimate_pos_ned_m;
    vel_ofs_ned_ms = _ofs_estimate_vel_ned_ms;
    accel_ofs_ned_mss = _ofs_estimate_accel_ned_mss;

    return true;
}

// Retrieves distance vectors (with and without configured offsets) and the target’s velocity, all in the NED frame.
bool AP_Follow::get_target_dist_and_vel_NED_m(Vector3f &dist_ned, Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    WITH_SEMAPHORE(_follow_sem);
    
    if (!_estimate_valid) {
        return false;
    }

    Vector3p current_position_ned_m;
    if (!AP::ahrs().get_relative_position_NED_origin(current_position_ned_m)) {
        return false;
    }

    const Vector3p dist_vec_ned_m = _estimate_pos_ned_m - current_position_ned_m;
    const Vector3p ofs_dist_vec = _ofs_estimate_pos_ned_m - current_position_ned_m;
    dist_ned = dist_vec_ned_m.tofloat();
    dist_with_offs = ofs_dist_vec.tofloat();
    vel_ned = _ofs_estimate_vel_ned_ms;

    return true;
}

// Retrieves the estimated target heading and heading rate in radians.
bool AP_Follow::get_heading_heading_rate_rad(float &heading_rad, float &heading_rate_rads) const
{
    if (!_estimate_valid) {
        return false;
    }

    // return latest heading estimate
    heading_rad = _estimate_heading_rad;
    heading_rate_rads = _estimate_heading_rate_rads;
    return true;
}

// Retrieves the target's estimated global location and estimated velocity
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned)
{
    WITH_SEMAPHORE(_follow_sem);

    if (!_estimate_valid) {
        return false;
    }

    if (!AP::ahrs().get_location_from_origin_offset_NED(loc, _estimate_pos_ned_m)) {
        return false;
    }
    vel_ned = _estimate_vel_ned_ms;

    // The frame requested by FOLL_ALT_TYPE may not be the frame of location returned by ahrs. 
    // Make sure we give the caller the frame they have asked for.
    return loc.change_alt_frame(_alt_type);
}

// Retrieves the target's estimated global location including configured offsets, and estimated velocity,  for LUA bindings.
bool AP_Follow::get_target_location_and_velocity_ofs(Location &loc, Vector3f &vel_ned)
{
    WITH_SEMAPHORE(_follow_sem);

    if (!_estimate_valid) {
        return false;
    }
    if (!AP::ahrs().get_location_from_origin_offset_NED(loc, _ofs_estimate_pos_ned_m)) {
        return false;
    }

    vel_ned = _ofs_estimate_vel_ned_ms;
    return true;
}

// Retrieves the estimated target heading in degrees (0° = North, 90° = East) for LUA bindings.
bool AP_Follow::get_target_heading_deg(float &heading_deg)
{
    WITH_SEMAPHORE(_follow_sem);
    
    if (!_estimate_valid) {
        return false;
    }

    // return latest heading estimate
    heading_deg = degrees(_estimate_heading_rad);
    return true;
}

// Retrieves the estimated target heading in degrees (0° = North, 90° = East) for LUA bindings.
bool AP_Follow::get_target_heading_rate_degs(float &heading_rate_degs)
{
    WITH_SEMAPHORE(_follow_sem);
    
    if (!_estimate_valid) {
        return false;
    }

    // return latest heading estimate
    heading_rate_degs = degrees(_estimate_heading_rate_rads);
    return true;
}


//==============================================================================
// MAVLink Message Handling
//==============================================================================

// Handles incoming MAVLink messages to update the target's position, velocity, and heading.
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // Invalidate the estimate if no position update has been received within the timeout period.
    // If using automatic sysid tracking, clear the sysid and reset tracking state.
    if ((_last_location_update_ms == 0) ||
        (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_SYSID_TIMEOUT_MS)) {
        if (_automatic_sysid) {
            _sysid.set(0);         // clear target system ID
            _sysid_used = 0;       // reset used sysid tracking
        }
        _estimate_valid = false;   // mark estimate as invalid
        _using_follow_target = false; // reset follow-target usage flag
    }

    if (!should_handle_message(msg)) {
        // ignore message if filtering rules reject it (e.g., wrong sysid)
        return;
    }

    // decode MAVLink message
    bool updated = false;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        // handle standard global position messages
        updated = handle_global_position_int_message(msg);
        break;
    }
    case MAVLINK_MSG_ID_FOLLOW_TARGET: {
        // handle follow-target specific messages
        updated = handle_follow_target_message(msg);
        break;
    }
    }

    if (updated) {
        // Check if estimate needs reset based on position and velocity errors
        if (estimate_error_too_large()) {
            _estimate_valid = false;
        }

#if HAL_LOGGING_ENABLED
        // log current follow diagnostic data
        Log_Write_FOLL();
#endif
    }
}

// Returns true if the incoming MAVLink message should be processed for target updates.
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

// Checks whether the current estimate should be reset based on position and velocity errors.
bool AP_Follow::estimate_error_too_large() const
{
    const float timeout_sec = _timeout;

    // Calculate position thresholds based on maximum acceleration then deceleration for the timeout duration
    const float pos_thresh_horiz_m = _accel_max_ne_mss.get() * sq(timeout_sec * 0.5);
    const float pos_thresh_vert_m  = _accel_max_d_mss.get()  * sq(timeout_sec * 0.5);

    // Calculate velocity thresholds using the new helper function
    const float vel_thresh_horiz_ms = calc_max_velocity_change(_accel_max_ne_mss.get(), _jerk_max_ne_msss.get(), timeout_sec);
    const float vel_thresh_vert_ms  = calc_max_velocity_change(_accel_max_d_mss.get(), _jerk_max_d_msss.get(), timeout_sec);

    // Calculate current position and velocity errors
    const Vector3f pos_error = _estimate_pos_ned_m.tofloat() - _target_pos_ned_m.tofloat();
    const Vector3f vel_error = _estimate_vel_ned_ms - _target_vel_ned_ms;

    const Vector2f pos_error_xy = pos_error.xy();
    const float pos_error_z = pos_error.z;
    const Vector2f vel_error_xy = vel_error.xy();
    const float vel_error_z = vel_error.z;

    // Check horizontal and vertical separately
    const bool pos_horiz_bad = pos_error_xy.length() > pos_thresh_horiz_m;
    const bool vel_horiz_bad = vel_error_xy.length() > vel_thresh_horiz_ms;
    const bool pos_vert_bad = fabsf(pos_error_z) > pos_thresh_vert_m;
    const bool vel_vert_bad = fabsf(vel_error_z) > vel_thresh_vert_ms;

    return pos_horiz_bad || vel_horiz_bad || pos_vert_bad || vel_vert_bad;
}

// Calculates max velocity change under trapezoidal or triangular acceleration profile (jerk-limited).
float AP_Follow::calc_max_velocity_change(float accel_max, float jerk_max, float timeout_sec) const
{
    const float t_jerk = accel_max / jerk_max;
    const float t_total_jerk = 2.0f * t_jerk;

    if (timeout_sec >= t_total_jerk) {
        // time to ramp up, constant accel phase, and ramp down
        const float t_const = timeout_sec - t_total_jerk;
        const float delta_v_jerk = 0.5f * accel_max * t_jerk;
        const float delta_v_const = accel_max * t_const;
        return 2.0f * delta_v_jerk + delta_v_const;
    } else {
        // timeout too short: pure triangle profile
        const float t_half = timeout_sec * 0.5f;
        return 0.5f * jerk_max * sq(t_half);
    }
}

// Decodes a GLOBAL_POSITION_INT MAVLink message to update the target’s position, velocity, and heading.
bool AP_Follow::handle_global_position_int_message(const mavlink_message_t &msg)
{
    // decode GLOBAL_POSITION_INT message into packet struct
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);

    // ignore message if latitude and longitude are exactly zero (invalid GPS fix)
    if ((packet.lat == 0 && packet.lon == 0)) {
        return false;
    }

    if (_using_follow_target) {
        // if we are using follow_target, ignore global_position_int messages
        return false;
    }

    Location target_location;
    target_location.lat = packet.lat;
    target_location.lng = packet.lon;

    switch((Location::AltFrame)_alt_type) {
        case Location::AltFrame::ABSOLUTE:
            target_location.set_alt_cm(packet.alt * 0.1, Location::AltFrame::ABSOLUTE);
            break;
        case Location::AltFrame::ABOVE_HOME:
            target_location.set_alt_cm(packet.relative_alt * 0.1, Location::AltFrame::ABOVE_HOME);
            break;
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduCopter)
        case Location::AltFrame::ABOVE_TERRAIN:
            /// Altitude comes in as AMSL
            target_location.set_alt_cm(packet.alt * 0.1, Location::AltFrame::ABSOLUTE);
            // convert the incoming altitude to terrain altitude, but fail if there is no terrain data available
            if (!target_location.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
                return false;
            };
            break;
#endif
        default:
            // don't process the packet if the _alt_type is invalid
            return false;
    }
    
    // convert global location to local NED frame position
    Vector3p target_pos_neu_m;
    if (!target_location.get_vector_from_origin_NEU_m(target_pos_neu_m)) {
        return false;
    }

    if (packet.hdg <= 36000) {
        // valid heading field available (in centi-degrees)
        _target_heading_deg = packet.hdg * 0.01f;
    } else if (_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) {
        // heading missing but relative offset mode requires heading -> reject
        return false;
    } else {
        // no heading available: set heading rate to zero
        _target_heading_rate_degs = 0.0f;
    }

    _target_pos_ned_m.xy() = target_pos_neu_m.xy(); 
    _target_pos_ned_m.z = -target_pos_neu_m.z;

    // decode target velocity components (cm/s converted to m/s)
    _target_vel_ned_ms.x = packet.vx * 0.01f; // velocity north
    _target_vel_ned_ms.y = packet.vy * 0.01f; // velocity east
    _target_vel_ned_ms.z = packet.vz * 0.01f; // velocity down

    // target acceleration not available in GLOBAL_POSITION_INT
    _target_accel_ned_mss.zero();

    // apply jitter-corrected timestamp to this update
    _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_boot_ms, AP_HAL::millis());

    // if sysid not yet set, adopt sender’s sysid and enable automatic sysid tracking
    if (_sysid == 0) {
        _sysid.set(msg.sysid);
        _sysid_used = 0;
        _estimate_valid = false;
        _automatic_sysid = true;
    }

    return true;
}

// Decodes a FOLLOW_TARGET MAVLink message to update the target’s position, velocity, acceleration, and orientation.
bool AP_Follow::handle_follow_target_message(const mavlink_message_t &msg)
{
    // decode FOLLOW_TARGET message into packet struct
    mavlink_follow_target_t packet;
    mavlink_msg_follow_target_decode(&msg, &packet);

    // ignore message if latitude and longitude are exactly zero (invalid GPS fix)
    if ((packet.lat == 0 && packet.lon == 0)) {
        return false;
    }

    // require that at least position is estimated (bit 0 of est_capabilities)
    if ((packet.est_capabilities & (1<<0)) == 0) {
        return false;
    }

    // build Location object from latitude, longitude, and altitude (alt in meters)
    const Location target_location {
        packet.lat,
        packet.lon,
        int32_t(packet.alt * 100),  // convert meters to centimeters
        Location::AltFrame::ABSOLUTE
    };

    // convert global location to local NED frame position
    Vector3p target_pos_neu_m;
    if (!target_location.get_vector_from_origin_NEU_m(target_pos_neu_m)) {
        return false;
    }

    // adjust Z coordinate to NED frame (NEU altitude -> NED)
    Location origin;
    if (!AP::ahrs().get_origin(origin)) {
        return false;
    }

    // decode attitude if available (bit 3 of est_capabilities)
    if (packet.est_capabilities & (1 << 3)) {
        // reconstruct quaternion from packet
        Quaternion q{packet.attitude_q[0], packet.attitude_q[1], packet.attitude_q[2], packet.attitude_q[3]};
        float roll, pitch, yaw;
        q.to_euler(roll, pitch, yaw);

        // store heading in degrees, wrapped 0–360
        _target_heading_deg = wrap_360(degrees(yaw));

        // transform body rates (roll, pitch, yaw) to earth frame rates
        Matrix3f R;
        q.rotation_matrix(R);
        Vector3f omega_body = Vector3f{packet.rates[0], packet.rates[1], packet.rates[2]};
        Vector3f omega_world = R * omega_body; // rotate rates into earth frame

        // store heading rate (yaw rate in world frame) in degrees/sec
        _target_heading_rate_degs = degrees(omega_world.z);
    } else if (_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) {
        // if attitude unavailable and using relative frame, cannot compute — abort
        return false;
    } else {
        // otherwise, default heading rate to zero
        _target_heading_rate_degs = 0.0f;
    }

    _target_pos_ned_m.xy() = target_pos_neu_m.xy();
    _target_pos_ned_m.z = -packet.alt + origin.alt * 0.01;

    // decode velocity if available (bit 1 of est_capabilities)
    if (packet.est_capabilities & (1<<1)) {
        _target_vel_ned_ms.x = packet.vel[0]; // velocity north
        _target_vel_ned_ms.y = packet.vel[1]; // velocity east
        _target_vel_ned_ms.z = packet.vel[2]; // velocity down
    } else {
        _target_vel_ned_ms.zero();
    }

    // decode acceleration if available (bit 2 of est_capabilities)
    if (packet.est_capabilities & (1 << 2)) {
        _target_accel_ned_mss.x = packet.acc[0]; // acceleration north
        _target_accel_ned_mss.y = packet.acc[1]; // acceleration east
        _target_accel_ned_mss.z = packet.acc[2]; // acceleration down
    } else {
        _target_accel_ned_mss.zero();
    }

    // apply jitter-corrected timestamp to this update
    _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.timestamp, AP_HAL::millis());

    // if sysid not yet assigned, adopt sender's sysid and enable automatic sysid tracking
    if (_sysid == 0) {
        _sysid.set(msg.sysid);
        _automatic_sysid = true;
    }

    // we are using follow_target: set sysid to sender's sysid
    _using_follow_target = true;

    return true;
}


//==============================================================================
// Offset Initialization and Adjustment Functions
//==============================================================================

// Initializes the positional offsets from the target vehicle if not already set.
void AP_Follow::init_offsets_if_required()
{
    // return immediately if offsets have already been set
    if (!_offset_m.get().is_zero()) {
        return;
    }
    _offsets_were_zero = true;

    if (!_estimate_valid) {
        return;
    }

    // Check if the target is within the maximum distance
    Vector3p current_position_ned_m;
    if (!AP::ahrs().get_relative_position_NED_origin(current_position_ned_m)) {
        return;
    }
    const Vector3f dist_vec_ned_m = (_target_pos_ned_m - current_position_ned_m).tofloat();

    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE)) {
        // rotate offset into vehicle-relative frame based on heading
        _offset_m.set(rotate_vector(-dist_vec_ned_m, -degrees(_estimate_heading_rad)));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Relative follow offset loaded");
    } else {
        // initialize offset in NED frame (no heading rotation)
        _offset_m.set(-dist_vec_ned_m);

        // ensure offset type is set to NED frame if initialized this way
        _offset_type.set(AP_FOLLOW_OFFSET_TYPE_NED);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "N-E-D follow offset loaded");
    }
}

// Rotates a 3D vector clockwise by the specified angle (degrees).
Vector3f AP_Follow::rotate_vector(const Vector3f &vec, float angle_deg) const
{
    // rotate roll, pitch input from north facing to vehicle's perspective
    Vector3f ret = vec;
    ret.xy().rotate(radians(angle_deg));

    return ret;
}

// Resets follow mode offsets to zero if they were automatically initialized.
void AP_Follow::clear_offsets_if_required()
{
    if (_offsets_were_zero) {
        _offset_m.set(Vector3f());
        _offsets_were_zero = false;
    }
}


//==============================================================================
// Distance and Bearing Management
//==============================================================================

// Resets the recorded distance and bearing to the target to zero.
void AP_Follow::clear_dist_and_bearing_to_target()
{
    _dist_to_target_m = 0.0f;
    _bearing_to_target_deg = 0.0f;
}

// Updates the recorded distance and bearing to the target to zero.
void AP_Follow::update_dist_and_bearing_to_target()
{
    Vector3p current_position_ned_m;
    if (!AP::ahrs().get_relative_position_NED_origin(current_position_ned_m)) {
        // if unable to retrieve local position, clear distance/bearing info
        clear_dist_and_bearing_to_target();
    } else {
        // convert vehicle position to NED meters (NEU -> NED and cm -> m)
        current_position_ned_m.z = -current_position_ned_m.z; // NEU to NED
        current_position_ned_m *= 0.01;  // convert cm to m

        // calculate distance vectors to target, both with and without offsets
        const Vector3p ofs_dist_vec = _ofs_estimate_pos_ned_m - current_position_ned_m;

        // record distance and bearing to target for reporting/logging
        if (ofs_dist_vec.xy().is_zero()) {
            clear_dist_and_bearing_to_target();
        } else {
            _dist_to_target_m = ofs_dist_vec.xy().length();
            _bearing_to_target_deg = degrees(ofs_dist_vec.xy().angle());
        }
    }
}


//==============================================================================
// Logging
//==============================================================================

// Writes a diagnostic onboard log message containing target and vehicle tracking data for Follow mode.
#if HAL_LOGGING_ENABLED
void AP_Follow::Log_Write_FOLL()
{
    // retrieve latest estimated location and velocity
    Location loc_estimate{};
    Vector3f vel_estimate;
    UNUSED_RESULT(get_target_location_and_velocity(loc_estimate, vel_estimate));

    Location target_location;
    UNUSED_RESULT(AP::ahrs().get_location_from_origin_offset_NED(target_location, _target_pos_ned_m));

    // log the lead target's reported position and vehicle's estimated position
    // @LoggerMessage: FOLL
    // @Description: Follow library diagnostic data
    // @Field: TimeUS: Time since system startup (microseconds)
    // @Field: Lat: Target latitude (degrees * 1E7)
    // @Field: Lon: Target longitude (degrees * 1E7)
    // @Field: Alt: Target absolute altitude (centimeters)
    // @Field: VelN: Target velocity, North (m/s)
    // @Field: VelE: Target velocity, East (m/s)
    // @Field: VelD: Target velocity, Down (m/s)
    // @Field: LatE: Vehicle estimated latitude (degrees * 1E7)
    // @Field: LonE: Vehicle estimated longitude (degrees * 1E7)
    // @Field: AltE: Vehicle estimated altitude (centimeters)
    // @Field: FrmE: Vehicle estimated altitude Frame
    AP::logger().WriteStreaming("FOLL",
                                "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE,FrmE",  // labels
                                "sDUmnnnDUm-",    // units
                                "F--B000--B-",    // mults
                                "QLLifffLLib",    // fmt
                                AP_HAL::micros64(),
                                target_location.lat,
                                target_location.lng,
                                target_location.alt,
                                (double)_target_vel_ned_ms.x,
                                (double)_target_vel_ned_ms.y,
                                (double)_target_vel_ned_ms.z,
                                loc_estimate.lat,
                                loc_estimate.lng,
                                loc_estimate.alt,
                                loc_estimate.get_alt_frame()
                                );
}
#endif  // HAL_LOGGING_ENABLED


//==============================================================================
// Accessors and Helpers
//==============================================================================

// Returns true if following is enabled and a recent target update has been received.
bool AP_Follow::have_target(void) const
{
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || ((AP_HAL::millis() - _last_location_update_ms) > (uint32_t)(_timeout * 1000.0f))) {
        return false;
    }
    return true;
}

//==============================================================================
// AP_Follow Accessor
//==============================================================================

// Accessor for the AP_Follow singleton instance.
namespace AP {
    AP_Follow &follow()
    {
        return *AP_Follow::get_singleton();
    }
}

#endif  // AP_FOLLOW_ENABLED
