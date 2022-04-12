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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_WPNav.h"
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define AR_WPNAV_TIMEOUT_MS             100
#define AR_WPNAV_SPEED_DEFAULT          2.0f
#define AR_WPNAV_SPEED_MIN              0.05f   // minimum speed between waypoints in m/s
#define AR_WPNAV_SPEED_UPDATE_MIN_MS    500     // max speed cannot be updated more than once in this many milliseconds
#define AR_WPNAV_RADIUS_DEFAULT         2.0f
#define AR_WPNAV_OVERSPEED_RATIO_MAX    5.0f    // if _overspeed_enabled the vehicle may travel as quickly as 5x WP_SPEED
#define AR_WPNAV_SNAP_MAX               15.0f   // scurve snap (change in jerk) in m/s/s/s/s
#define AR_WPNAV_ACCEL_MAX              20.0    // acceleration used when user has specified no acceleration limit

const AP_Param::GroupInfo AR_WPNav::var_info[] = {

    // @Param: SPEED
    // @DisplayName: Waypoint speed default
    // @Description: Waypoint speed default
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPEED", 1, AR_WPNav, _speed_max, AR_WPNAV_SPEED_DEFAULT),

    // @Param: RADIUS
    // @DisplayName: Waypoint radius
    // @Description: The distance in meters from a waypoint when we consider the waypoint has been reached. This determines when the vehicle will turn toward the next waypoint.
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RADIUS", 2, AR_WPNav, _radius, AR_WPNAV_RADIUS_DEFAULT),

    // 3 was OVERSHOOT

    // 4 was PIVOT_ANGLE
    // 5 was PIVOT_RATE
    // 6 was SPEED_MIN
    // 7 was PIVOT_DELAY

    // @Group: PIVOT_
    // @Path: AR_PivotTurn.cpp
    AP_SUBGROUPINFO(_pivot, "PIVOT_", 8, AR_WPNav, AR_PivotTurn),

    // @Param: ACCEL
    // @DisplayName: Waypoint acceleration
    // @Description: Waypoint acceleration.  If zero then ATC_ACCEL_MAX is used
    // @Units: m/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ACCEL", 9, AR_WPNav, _accel_max, 0),

    // @Param: JERK
    // @DisplayName: Waypoint jerk
    // @Description: Waypoint jerk (change in acceleration).  If zero then jerk is same as acceleration
    // @Units: m/s/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("JERK", 10, AR_WPNav, _jerk_max, 0),

    AP_GROUPEND
};

AR_WPNav::AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control) :
    _atc(atc),
    _pos_control(pos_control),
    _pivot(atc)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise waypoint controller.  speed_max should be set to the maximum speed in m/s (or left at zero to use the default speed)
void AR_WPNav::init(float speed_max)
{
    // determine max speed, acceleration and jerk
    if (is_positive(speed_max)) {
        _base_speed_max = speed_max;
    } else {
        _base_speed_max = _speed_max;
    }
    _base_speed_max = MAX(AR_WPNAV_SPEED_MIN, _base_speed_max);
    float atc_accel_max = MIN(_atc.get_accel_max(), _atc.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        // accel_max of zero means no limit so use maximum acceleration
        atc_accel_max = AR_WPNAV_ACCEL_MAX;
    }
    const float accel_max = is_positive(_accel_max) ? MIN(_accel_max, atc_accel_max) : atc_accel_max;
    const float jerk_max = is_positive(_jerk_max) ? _jerk_max : accel_max;

    // initialise position controller
    _pos_control.set_limits(_base_speed_max, accel_max, _atc.get_turn_lat_accel_max(), jerk_max);

    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _track_scalar_dt = 1.0f;

    // init some flags
    _reached_destination = false;
    _fast_waypoint = false;

    // ensure pivot turns are deactivated
    _pivot.deactivate();
    _pivot_at_next_wp = false;

    // initialise origin and destination to stopping point
    _orig_and_dest_valid = false;
    set_origin_and_destination_to_stopping_point();

    // initialise nudge speed to zero
    set_nudge_speed_max(0);
}

// update navigation
void AR_WPNav::update(float dt)
{
    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !_orig_and_dest_valid || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _cross_track_error = 0;
        return;
    }

    // if no recent calls initialise desired_speed_limited to current speed
    if (!is_active()) {
        _desired_speed_limited = speed;
    }
    _last_update_ms = AP_HAL::millis();

    update_distance_and_bearing_to_destination();

    // handle change in max speed
    update_speed_max();

    // advance target along path unless vehicle is pivoting
    if (!_pivot.active()) {
        switch (_nav_control_type) {
        case NavControllerType::NAV_SCURVE:
            advance_wp_target_along_track(current_loc, dt);
            break;
        case NavControllerType::NAV_PSC_INPUT_SHAPING:
            update_psc_input_shaping(dt);
            break;
        }
    }

    // update_steering_and_speed
    update_steering_and_speed(current_loc, dt);
}

// set maximum speed in m/s.  returns true on success
// this should not be called at more than 3hz or else SCurve path planning may not advance properly
bool AR_WPNav::set_speed_max(float speed_max)
{
    // range check target speed
    if (speed_max < AR_WPNAV_SPEED_MIN) {
        return false;
    }

    _base_speed_max = speed_max;
    return true;
}

// set speed nudge in m/s.  this will have no effect unless nudge_speed_max > speed_max
// nudge_speed_max should always be positive regardless of whether the vehicle is travelling forward or reversing
void AR_WPNav::set_nudge_speed_max(float nudge_speed_max)
{
    _nudge_speed_max = nudge_speed_max;
}

// set desired location and (optionally) next_destination
// next_destination should be provided if known to allow smooth cornering
bool AR_WPNav::set_desired_location(const struct Location& destination, Location next_destination)
{
    // re-initialise if inactive, previous destination has been interrupted or different controller was used
    if (!is_active() || !_reached_destination || (_nav_control_type != NavControllerType::NAV_SCURVE)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
        // clear scurves
        _scurve_prev_leg.init();
        _scurve_this_leg.init();
        _scurve_next_leg.init();
    }

    // shift this leg to previous leg
    _scurve_prev_leg = _scurve_this_leg;

    // initialise some variables
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    // check if vehicle should pivot if vehicle stopped at previous waypoint
    // or journey to previous waypoint was interrupted or navigation has just started
    if (!_fast_waypoint) {
        _pivot.deactivate();
        _pivot.check_activation((_reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd()) * 0.01, _pivot_at_next_wp);
    }

    // convert origin and destination to offset from EKF origin
    Vector2f origin_NE;
    Vector2f destination_NE;
    if (!_origin.get_vector_xy_from_origin_NE(origin_NE) ||
        !_destination.get_vector_xy_from_origin_NE(destination_NE)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    origin_NE *= 0.01f;
    destination_NE *= 0.01f;

    // calculate track to destination
    if (_fast_waypoint && !_scurve_next_leg.finished()) {
        // skip recalculating this leg by simply shifting next leg
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _scurve_this_leg.calculate_track(Vector3f{origin_NE.x, origin_NE.y, 0.0f},              // origin
                                         Vector3f{destination_NE.x, destination_NE.y, 0.0f},    // destination
                                         _pos_control.get_speed_max(),
                                         _pos_control.get_speed_max(),  // speed up (not used)
                                         _pos_control.get_speed_max(),  // speed down (not used)
                                         _pos_control.get_accel_max(),  // forward back acceleration
                                         _pos_control.get_accel_max(),  // vertical accel (not used)
                                         AR_WPNAV_SNAP_MAX,             // snap
                                         _pos_control.get_jerk_max());
    }

    // handle next destination
    _scurve_next_leg.init();
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    if (next_destination.initialised()) {
        // check if vehicle should pivot at next waypoint
        const float next_wp_yaw_change = get_corner_angle(_origin, destination, next_destination);
        _pivot_at_next_wp = _pivot.would_activate(next_wp_yaw_change);
        if (!_pivot_at_next_wp) {
            // convert next_destination to offset from EKF origin
            Vector2f next_destination_NE;
            if (!next_destination.get_vector_xy_from_origin_NE(next_destination_NE)) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;
            }
            next_destination_NE *= 0.01f;
            _scurve_next_leg.calculate_track(Vector3f{destination_NE.x, destination_NE.y, 0.0f},
                                             Vector3f{next_destination_NE.x, next_destination_NE.y, 0.0f},
                                             _pos_control.get_speed_max(),
                                             _pos_control.get_speed_max(),  // speed up (not used)
                                             _pos_control.get_speed_max(),  // speed down (not used)
                                             _pos_control.get_accel_max(),  // forward back acceleration
                                             _pos_control.get_accel_max(),  // vertical accel (not used)
                                             AR_WPNAV_SNAP_MAX,             // snap
                                             _pos_control.get_jerk_max());

            // next destination provided so fast waypoint
            _fast_waypoint = true;
        }
    }

    // scurves used for navigation to destination
    _nav_control_type = NavControllerType::NAV_SCURVE;

    update_distance_and_bearing_to_destination();

    return true;
}

// set desired location to a reasonable stopping point, return true on success
bool AR_WPNav::set_desired_location_to_stopping_location()
{
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    return set_desired_location(stopping_loc);
}

// set desired location as offset from the EKF origin, return true on success
bool AR_WPNav::set_desired_location_NED(const Vector3f& destination)
{
    // initialise destination to ekf origin
    Location destination_ned;
    if (!AP::ahrs().get_origin(destination_ned)) {
        return false;
    }

    // apply offset
    destination_ned.offset(destination.x, destination.y);
    return set_desired_location(destination_ned);
}

bool AR_WPNav::set_desired_location_NED(const Vector3f &destination, const Vector3f &next_destination)
{
    // initialise destination to ekf origin
    Location dest_loc, next_dest_loc;
    if (!AP::ahrs().get_origin(dest_loc)) {
        return false;
    }
    next_dest_loc = dest_loc;

    // apply offsets
    dest_loc.offset(destination.x, destination.y);
    next_dest_loc.offset(next_destination.x, next_destination.y);
    return set_desired_location(dest_loc, next_dest_loc);
}

// set desired location but expect the destination to be updated again in the near future
// position controller input shaping will be used for navigation instead of scurves
// Note: object avoidance is not supported if this method is used
bool AR_WPNav::set_desired_location_expect_fast_update(const Location &destination)
{
    // initialise if not active
    if (!is_active() || (_nav_control_type != NavControllerType::NAV_PSC_INPUT_SHAPING)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
    }

    // initialise some variables
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    // check if vehicle should pivot
    _pivot.check_activation((_reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd()) * 0.01);

    // position controller input shaping used for navigation to destination
    _nav_control_type = NavControllerType::NAV_PSC_INPUT_SHAPING;
    return true;
}

// calculate vehicle stopping point using current location, velocity and maximum acceleration
bool AR_WPNav::get_stopping_location(Location& stopping_loc)
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // get current velocity vector and speed
    const Vector2f velocity = AP::ahrs().groundspeed_vector();
    const float speed = velocity.length();

    // avoid divide by zero
    if (!is_positive(speed)) {
        stopping_loc = current_loc;
        return true;
    }

    // get stopping distance in meters
    const float stopping_dist = _atc.get_stopping_distance(speed);

    // calculate stopping position from current location in meters
    const Vector2f stopping_offset = velocity.normalized() * stopping_dist;
    stopping_loc = current_loc;
    stopping_loc.offset(stopping_offset.x, stopping_offset.y);

    return true;
}

// true if update has been called recently
bool AR_WPNav::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_WPNAV_TIMEOUT_MS);
}

// move target location along track from origin to destination using SCurves navigation
void AR_WPNav::advance_wp_target_along_track(const Location &current_loc, float dt)
{
    // exit immediately if no current location, destination or disarmed
    Vector2f curr_pos_NE;
    Vector3f curr_vel_NED;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos_NE) || !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        return;
    }

    // exit immediately if we can't convert waypoint origin to offset from ekf origin
    Vector2f origin_NE;
    if (!_origin.get_vector_xy_from_origin_NE(origin_NE)) {
        return;
    }
    // convert from cm to meters
    origin_NE *= 0.01f;

    // use _track_scalar_dt to slow down S-Curve time to prevent target moving too far in front of vehicle
    Vector2f curr_target_vel = _pos_control.get_desired_velocity();
    float track_scaler_dt = 1.0f;
    if (is_positive(curr_target_vel.length())) {
        Vector2f track_direction = curr_target_vel.normalized();
        const float track_error = _pos_control.get_pos_error().tofloat().dot(track_direction);
        float track_velocity = curr_vel_NED.xy().dot(track_direction);
        // set time scaler to be consistent with the achievable vehicle speed with a 5% buffer for short term variation.
        const float time_scaler_dt_max = _overspeed_enabled ? AR_WPNAV_OVERSPEED_RATIO_MAX : 1.0f;
        track_scaler_dt = constrain_float(0.05f + (track_velocity - _pos_control.get_pos_p().kP() * track_error) / curr_target_vel.length(), 0.1f, time_scaler_dt_max);
    }
    // change s-curve time speed with a time constant of maximum acceleration / maximum jerk
    float track_scaler_tc = 1.0f;
    if (is_positive(_pos_control.get_jerk_max())) {
        track_scaler_tc = _pos_control.get_accel_max() / _pos_control.get_jerk_max();
    }
    _track_scalar_dt += (track_scaler_dt - _track_scalar_dt) * (dt / track_scaler_tc);

    // target position, velocity and acceleration from straight line or spline calculators
    Vector3f target_pos_3d_ftype{origin_NE.x, origin_NE.y, 0.0f};
    Vector3f target_vel, target_accel;

    // update target position, velocity and acceleration
    const float wp_radius = MAX(_radius, _turn_radius);
    bool s_finished = _scurve_this_leg.advance_target_along_track(_scurve_prev_leg, _scurve_next_leg, wp_radius, _pos_control.get_lat_accel_max(), _fast_waypoint, _track_scalar_dt * dt, target_pos_3d_ftype, target_vel, target_accel);

    // pass new target to the position controller
    init_pos_control_if_necessary();
    Vector2p target_pos_ptype{target_pos_3d_ftype.x, target_pos_3d_ftype.y};
    _pos_control.set_pos_vel_accel_target(target_pos_ptype, target_vel.xy(), target_accel.xy());

    // check if we've reached the waypoint
    if (!_reached_destination && s_finished) {
        // "fast" waypoints are complete once the intermediate point reaches the destination
        if (_fast_waypoint) {
            _reached_destination = true;
        } else {
            // regular waypoints also require the vehicle to be within the waypoint radius or past the "finish line"
            const bool near_wp = current_loc.get_distance(_destination) <= _radius;
            const bool past_wp = current_loc.past_interval_finish_line(_origin, _destination);
            _reached_destination = near_wp || past_wp;
        }
    }
}

// update psc input shaping navigation controller
void AR_WPNav::update_psc_input_shaping(float dt)
{
    // convert destination location to offset from EKF origin (in meters)
    Vector2f pos_target_cm;
    if (!_destination.get_vector_xy_from_origin_NE(pos_target_cm)) {
        return;
    }

    // initialise position controller if not called recently
    init_pos_control_if_necessary();

    // convert to meters and update target
    const Vector2p pos_target = pos_target_cm.topostype() * 0.01;
    _pos_control.input_pos_target(pos_target, dt);

    // update reached_destination
    if (!_reached_destination) {
        // calculate position difference between destination and position controller input shaped target
        Vector2p pos_target_diff = pos_target - _pos_control.get_pos_target();
        // vehicle has reached destination when the target is within 1cm of the destination and vehicle is within waypoint radius
        _reached_destination = (pos_target_diff.length_squared() < sq(0.01)) && (_pos_control.get_pos_error().length_squared() < sq(_radius));
    }
}

// update distance from vehicle's current position to destination
void AR_WPNav::update_distance_and_bearing_to_destination()
{
    // if no current location leave distance unchanged
    Location current_loc;
    if (!_orig_and_dest_valid || !AP::ahrs().get_location(current_loc)) {
        _distance_to_destination = 0.0f;
        _wp_bearing_cd = 0.0f;
        return;
    }
    _distance_to_destination = current_loc.get_distance(_destination);
    _wp_bearing_cd = current_loc.get_bearing_to(_destination);
}

// calculate steering and speed to drive along line from origin to destination waypoint
void AR_WPNav::update_steering_and_speed(const Location &current_loc, float dt)
{
    _cross_track_error = calc_crosstrack_error(current_loc);

    // handle pivot turns
    if (_pivot.active()) {
        // decelerate to zero
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_heading_cd = _reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd();
        _desired_turn_rate_rads = is_zero(_desired_speed_limited) ? _pivot.get_turn_rate_rads(_desired_heading_cd * 0.01, dt) : 0;
        _desired_lat_accel = 0.0f;
        return;
    }

    _pos_control.set_reversed(_reversed);
    _pos_control.update(dt);
    _desired_speed_limited = _pos_control.get_desired_speed();
    _desired_turn_rate_rads = _pos_control.get_desired_turn_rate_rads();
    _desired_lat_accel = _pos_control.get_desired_lat_accel();
}

// settor to allow vehicle code to provide turn related param values to this library (should be updated regularly)
void AR_WPNav::set_turn_params(float turn_radius, bool pivot_possible)
{
    _turn_radius = pivot_possible ? 0.0 : turn_radius;
    _pivot.enable(pivot_possible);
}

// calculate the crosstrack error
float AR_WPNav::calc_crosstrack_error(const Location& current_loc) const
{
    if (!_orig_and_dest_valid) {
        return 0.0f;
    }

    // get object avoidance adjusted origin and destination
    const Location &orig = get_oa_origin();
    const Location &dest = get_oa_destination();

    // calculate the NE position of destination relative to origin
    Vector2f dest_from_origin = orig.get_distance_NE(dest);

    // return distance to destination if length of track is very small
    if (dest_from_origin.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(dest).length();
    }

    // convert to a vector indicating direction only
    dest_from_origin.normalize();

    // calculate the NE position of the vehicle relative to origin
    const Vector2f veh_from_origin = orig.get_distance_NE(current_loc);

    // calculate distance to target track, for reporting
    return veh_from_origin % dest_from_origin;
}

// calculate yaw change at next waypoint in degrees
// returns zero if the angle cannot be calculated because some points are on top of others
float AR_WPNav::get_corner_angle(const Location& loc1, const Location& loc2, const Location& loc3) const
{
    // sanity check
    if (!loc1.initialised() || !loc2.initialised() || !loc3.initialised()) {
        return 0;
    }
    const float loc1_to_loc2_deg = loc1.get_bearing_to(loc2) * 0.01;
    const float loc2_to_loc3_deg = loc2.get_bearing_to(loc3) * 0.01;
    const float diff_yaw_deg = wrap_180(loc2_to_loc3_deg - loc1_to_loc2_deg);
    return diff_yaw_deg;
}

// helper function to initialise position controller if it hasn't been called recently
// this should be called before updating the position controller with new targets but after the EKF has a good position estimate
void AR_WPNav::init_pos_control_if_necessary()
{
    // initialise position controller if not called recently
    if (!_pos_control.is_active()) {
        if (!_pos_control.init()) {
            // this should never fail because we should always have a valid position estimate at this point
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }
    }
}

// set origin and destination to stopping point
bool AR_WPNav::set_origin_and_destination_to_stopping_point()
{
    // initialise origin and destination to stopping point
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    _origin = _destination = stopping_loc;
    _orig_and_dest_valid = true;
    return true;
}

// check for changes in _base_speed_max or _nudge_speed_max
// updates position controller limits and recalculate scurve path if required
void AR_WPNav::update_speed_max()
{
    const float speed_max = MAX(_base_speed_max, _nudge_speed_max);

    // ignore calls that do not change the speed
    if (is_equal(speed_max, _pos_control.get_speed_max())) {
        return;
    }

    // protect against rapid updates
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_speed_update_ms < AR_WPNAV_SPEED_UPDATE_MIN_MS) {
        return;
    }
    _last_speed_update_ms = now_ms;

    // update position controller max speed
    _pos_control.set_limits(speed_max, _pos_control.get_accel_max(), _pos_control.get_lat_accel_max(), _pos_control.get_jerk_max());

    // change track speed
    _scurve_this_leg.set_speed_max(_pos_control.get_speed_max(), _pos_control.get_speed_max(), _pos_control.get_speed_max());
    _scurve_next_leg.set_speed_max(_pos_control.get_speed_max(), _pos_control.get_speed_max(), _pos_control.get_speed_max());
}
