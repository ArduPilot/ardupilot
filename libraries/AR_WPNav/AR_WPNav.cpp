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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_WPNav.h"

extern const AP_HAL::HAL& hal;

#define AR_WPNAV_TIMEOUT_MS             100
#define AR_WPNAV_SPEED_DEFAULT          2.0f
#define AR_WPNAV_RADIUS_DEFAULT         2.0f
#define AR_WPNAV_OVERSHOOT_DEFAULT      2.0f
#define AR_WPNAV_PIVOT_ANGLE_DEFAULT    60
#define AR_WPNAV_PIVOT_ANGLE_ACCURACY   10      // vehicle will pivot to within this many degrees of destination
#define AR_WPNAV_PIVOT_RATE_DEFAULT     90

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

    // @Param: OVERSHOOT
    // @DisplayName: Waypoint overshoot maximum
    // @Description: Waypoint overshoot maximum in meters.  The vehicle will attempt to stay within this many meters of the track as it completes one waypoint and moves to the next.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("OVERSHOOT", 3, AR_WPNav, _overshoot, AR_WPNAV_OVERSHOOT_DEFAULT),

    // @Param: PIVOT_ANGLE
    // @DisplayName: Waypoint Pivot Angle
    // @Description: Pivot when the difference between the vehicle's heading and its target heading is more than this many degrees. Set to zero to disable pivot turns. Note: This parameter should be greater than 10 degrees for pivot turns to work.
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PIVOT_ANGLE", 4, AR_WPNav, _pivot_angle, AR_WPNAV_PIVOT_ANGLE_DEFAULT),

    // @Param: PIVOT_RATE
    // @DisplayName: Waypoint Pivot Turn Rate
    // @Description: Turn rate during pivot turns
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PIVOT_RATE", 5, AR_WPNav, _pivot_rate, AR_WPNAV_PIVOT_RATE_DEFAULT),

    // @Param: SPEED_MIN
    // @DisplayName: Waypoint speed minimum
    // @Description: Vehicle will not slow below this speed for corners.  Should be set to boat's plane speed.  Does not apply to pivot turns.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN", 6, AR_WPNav, _speed_min, 0),

    // @Param: PIVOT_DELAY
    // @DisplayName: Delay after pivot turn
    // @Description: Waiting time after pivot turn
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PIVOT_DELAY", 7, AR_WPNav, _pivot_delay, 0),

    AP_GROUPEND
};

AR_WPNav::AR_WPNav(AR_AttitudeControl& atc, AP_Navigation& nav_controller) :
    _atc(atc),
    _nav_controller(nav_controller)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update navigation
void AR_WPNav::update(float dt)
{
    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !_orig_and_dest_valid || !AP::ahrs().get_position(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_turn_rate_rads = 0.0f;
        return;
    }

    // if no recent calls initialise desired_speed_limited to current speed
    if (!is_active()) {
        _desired_speed_limited = speed;
    }
    _last_update_ms = AP_HAL::millis();

    // run path planning around obstacles
    bool stop_vehicle = false;
   
    // true if OA has been recently active;
    bool _oa_was_active = _oa_active;

    AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();
    if (oa != nullptr) {
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used;
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc, _origin, _destination, _oa_origin, _oa_destination, path_planner_used);
        switch (oa_retstate) {
        case AP_OAPathPlanner::OA_NOT_REQUIRED:
            _oa_active = false;
            break;
        case AP_OAPathPlanner::OA_PROCESSING:
        case AP_OAPathPlanner::OA_ERROR:
            // during processing or in case of error, slow vehicle to a stop
            stop_vehicle = true;
            _oa_active = false;
            break;
        case AP_OAPathPlanner::OA_SUCCESS:
            _oa_active = true;
            break;
        }
    }
    if (!_oa_active) {
        _oa_origin = _origin;
        _oa_destination = _destination;
    }

    update_distance_and_bearing_to_destination();

    // if object avoidance is active check if vehicle should pivot towards destination
    if (_oa_active) {
        update_pivot_active_flag();
    }

    // check if vehicle is in recovering state after switching off OA
    if (!_oa_active && _oa_was_active) {
        if (oa->get_options() & AP_OAPathPlanner::OA_OPTION_WP_RESET) {
            //reset wp origin to vehicles current location
            _origin = current_loc;
        }
    }

    // check if vehicle has reached the destination
    const bool near_wp = _distance_to_destination <= _radius;
    const bool past_wp = !_oa_active && current_loc.past_interval_finish_line(_origin, _destination);
    if (!_reached_destination && (near_wp || past_wp)) {
       _reached_destination = true;
    }

    // handle stopping vehicle if avoidance has failed
    if (stop_vehicle) {
        // decelerate to speed to zero and set turn rate to zero
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        return;
    }

    // calculate the required turn of the wheels
    update_steering(current_loc, speed);

    // calculate desired speed
    update_desired_speed(dt);
}

// set desired location
bool AR_WPNav::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    // set origin to last destination if waypoint controller active
    if (is_active() && _orig_and_dest_valid && _reached_destination) {
        _origin = _destination;
    } else {
        // otherwise use reasonable stopping point
        if (!get_stopping_location(_origin)) {
            return false;
        }
    }

    // initialise some variables
    _oa_origin = _origin;
    _oa_destination = _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;
    update_distance_and_bearing_to_destination();

    // determine if we should pivot immediately
    update_pivot_active_flag();

    // set final desired speed and whether vehicle should pivot
    _desired_speed_final = 0.0f;
    if (!is_equal(next_leg_bearing_cd, AR_WPNAV_HEADING_UNKNOWN)) {
        const float curr_leg_bearing_cd = _origin.get_bearing_to(_destination);
        const float turn_angle_cd = wrap_180_cd(next_leg_bearing_cd - curr_leg_bearing_cd);
        if (fabsf(turn_angle_cd) < 10.0f) {
            // if turning less than 0.1 degrees vehicle can continue at full speed
            // we use 0.1 degrees instead of zero to avoid divide by zero in calcs below
            _desired_speed_final = _desired_speed;
        } else if (use_pivot_steering_at_next_WP(turn_angle_cd)) {
            // pivoting so we will stop
            _desired_speed_final = 0.0f;
        } else {
            // calculate maximum speed that keeps overshoot within bounds
            const float radius_m = fabsf(_overshoot / (cosf(radians(turn_angle_cd * 0.01f)) - 1.0f));
            _desired_speed_final = MIN(_desired_speed, safe_sqrt(_atc.get_turn_lat_accel_max() * radius_m));
            // ensure speed does not fall below minimum
            apply_speed_min(_desired_speed_final);
        }
    }

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
bool AR_WPNav::set_desired_location_NED(const Vector3f& destination, float next_leg_bearing_cd)
{
    // initialise destination to ekf origin
    Location destination_ned;
    if (!AP::ahrs().get_origin(destination_ned)) {
        return false;
    }

    // apply offset
    destination_ned.offset(destination.x, destination.y);
    return set_desired_location(destination_ned, next_leg_bearing_cd);
}

// calculate vehicle stopping point using current location, velocity and maximum acceleration
bool AR_WPNav::get_stopping_location(Location& stopping_loc)
{
    Location current_loc;
    if (!AP::ahrs().get_position(current_loc)) {
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

// returns true if vehicle should pivot turn at next waypoint
bool AR_WPNav::use_pivot_steering_at_next_WP(float yaw_error_cd) const
{
    // check cases where we clearly cannot use pivot steering
    if (!_pivot_possible || _pivot_angle <= AR_WPNAV_PIVOT_ANGLE_ACCURACY) {
        return false;
    }

    // if error is larger than _pivot_angle then use pivot steering at next WP
    if (fabsf(yaw_error_cd) * 0.01f > _pivot_angle) {
        return true;
    }

    return false;
}

// updates _pivot_active flag based on heading error to destination
// relies on update_distance_and_bearing_to_destination having been called first
// to update _oa_wp_bearing and _reversed variables
void AR_WPNav::update_pivot_active_flag()
{
    // check cases where we clearly cannot use pivot steering
    if (!_pivot_possible || (_pivot_angle <= AR_WPNAV_PIVOT_ANGLE_ACCURACY)) {
        _pivot_active = false;
        return;
    }

    // calc yaw error
    const float yaw_cd = _reversed ? wrap_360_cd(_oa_wp_bearing_cd + 18000) : _oa_wp_bearing_cd;
    const float yaw_error = fabsf(wrap_180_cd(yaw_cd - AP::ahrs().yaw_sensor)) * 0.01f;

    // if error is larger than _pivot_angle start pivot steering
    if (yaw_error > _pivot_angle) {
        _pivot_active = true;
        return;
    }

    uint32_t now = AP_HAL::millis();

    // if within 10 degrees of the target heading, set start time of pivot steering
    if (_pivot_active && yaw_error < AR_WPNAV_PIVOT_ANGLE_ACCURACY && _pivot_start_ms == 0) {
        _pivot_start_ms = now;
    }

    // exit pivot steering after the time set by pivot_delay has elapsed
    if (_pivot_start_ms > 0 && now - _pivot_start_ms >= constrain_float(_pivot_delay.get(), 0.0f, 60.0f) * 1000.0f) {
        _pivot_active = false;
        _pivot_start_ms = 0;
    }
}

// true if update has been called recently
bool AR_WPNav::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_WPNAV_TIMEOUT_MS);
}

// update distance from vehicle's current position to destination
void AR_WPNav::update_distance_and_bearing_to_destination()
{
    // if no current location leave distance unchanged
    Location current_loc;
    if (!_orig_and_dest_valid || !AP::ahrs().get_position(current_loc)) {
        _distance_to_destination = 0.0f;
        _wp_bearing_cd = 0.0f;

        // update OA adjusted values
        _oa_distance_to_destination = 0.0f;
        _oa_wp_bearing_cd = 0.0f;
        return;
    }
    _distance_to_destination = current_loc.get_distance(_destination);
    _wp_bearing_cd = current_loc.get_bearing_to(_destination);

    // update OA adjusted values
    if (_oa_active) {
        _oa_distance_to_destination = current_loc.get_distance(_oa_destination);
        _oa_wp_bearing_cd = current_loc.get_bearing_to(_oa_destination);
    } else {
        _oa_distance_to_destination = _distance_to_destination;
        _oa_wp_bearing_cd = _wp_bearing_cd;
    }
}

// calculate steering output to drive along line from origin to destination waypoint
// relies on update_distance_and_bearing_to_destination being called first so _wp_bearing_cd has been updated
void AR_WPNav::update_steering(const Location& current_loc, float current_speed)
{
    // calculate desired turn rate and update desired heading
    if (_pivot_active) {
        _cross_track_error = calc_crosstrack_error(current_loc);
        _desired_heading_cd = _reversed ? wrap_360_cd(_oa_wp_bearing_cd + 18000) : _oa_wp_bearing_cd;;
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = _atc.get_turn_rate_from_heading(radians(_desired_heading_cd * 0.01f), radians(_pivot_rate));

        // update flag so that it can be cleared
        update_pivot_active_flag();
    } else {
        // run L1 controller
        _nav_controller.set_reverse(_reversed);
        _nav_controller.update_waypoint(_reached_destination ? current_loc : _oa_origin, _oa_destination, _radius);

        // retrieve lateral acceleration, heading back towards line and crosstrack error
        _desired_lat_accel = constrain_float(_nav_controller.lateral_acceleration(), -_atc.get_turn_lat_accel_max(), _atc.get_turn_lat_accel_max());
        _desired_heading_cd = wrap_360_cd(_nav_controller.nav_bearing_cd());
        if (_reversed) {
            _desired_lat_accel *= -1.0f;
            _desired_heading_cd = wrap_360_cd(_desired_heading_cd + 18000);
        }
        _cross_track_error = _nav_controller.crosstrack_error();
        _desired_turn_rate_rads = _atc.get_turn_rate_from_lat_accel(_desired_lat_accel, current_speed);
    }
}

// calculated desired speed(in m/s) based on yaw error and lateral acceleration and/or distance to a waypoint
// relies on update_distance_and_bearing_to_destination and update_steering being run so these internal members
// have been updated: _oa_wp_bearing_cd, _cross_track_error, _oa_distance_to_destination
void AR_WPNav::update_desired_speed(float dt)
{
    // reduce speed to zero during pivot turns
    if (_pivot_active) {
        // decelerate to zero
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        return;
    }

    // accelerate desired speed towards max
    float des_speed_lim = _atc.get_desired_speed_accel_limited(_reversed ? -_desired_speed : _desired_speed, dt);

    // reduce speed to limit overshoot from line between origin and destination
    // calculate number of degrees vehicle must turn to face waypoint
    float ahrs_yaw_sensor = AP::ahrs().yaw_sensor;
    const float heading_cd = is_negative(des_speed_lim) ? wrap_180_cd(ahrs_yaw_sensor + 18000) : ahrs_yaw_sensor;
    const float wp_yaw_diff_cd = wrap_180_cd(_oa_wp_bearing_cd - heading_cd);
    const float turn_angle_rad = fabsf(radians(wp_yaw_diff_cd * 0.01f));

    // calculate distance from vehicle to line + wp_overshoot
    const float line_yaw_diff = wrap_180_cd(_oa_origin.get_bearing_to(_oa_destination) - heading_cd);
    const float dist_from_line = fabsf(_cross_track_error);
    const bool heading_away = is_positive(line_yaw_diff) == is_positive(_cross_track_error);
    const float wp_overshoot_adj = heading_away ? -dist_from_line : dist_from_line;

    // calculate radius of circle that touches vehicle's current position and heading and target position and heading
    float radius_m = 999.0f;
    const float radius_calc_denom = fabsf(1.0f - cosf(turn_angle_rad));
    if (!is_zero(radius_calc_denom)) {
        radius_m = MAX(0.0f, _overshoot + wp_overshoot_adj) / radius_calc_denom;
    }

    // calculate and limit speed to allow vehicle to stay on circle
    // ensure limit does not force speed below minimum
    float overshoot_speed_max = safe_sqrt(_atc.get_turn_lat_accel_max() * MAX(_turn_radius, radius_m));
    apply_speed_min(overshoot_speed_max);
    des_speed_lim = constrain_float(des_speed_lim, -overshoot_speed_max, overshoot_speed_max);

    // limit speed based on distance to waypoint and max acceleration/deceleration
    if (is_positive(_oa_distance_to_destination) && is_positive(_atc.get_decel_max())) {
        const float dist_speed_max = safe_sqrt(2.0f * _oa_distance_to_destination * _atc.get_decel_max() + sq(_desired_speed_final));
        des_speed_lim = constrain_float(des_speed_lim, -dist_speed_max, dist_speed_max);
    }

    _desired_speed_limited = des_speed_lim;
}

// settor to allow vehicle code to provide turn related param values to this library (should be updated regularly)
void AR_WPNav::set_turn_params(float turn_radius, bool pivot_possible)
{
    _turn_radius = turn_radius;
    _pivot_possible = pivot_possible;
}

// adjust speed to ensure it does not fall below value held in SPEED_MIN
// desired_speed should always be positive (or zero)
void AR_WPNav::apply_speed_min(float &desired_speed) const
{
    if (!is_positive(_speed_min) || (_speed_min > _speed_max)) {
        return;
    }

    // ensure speed does not fall below minimum
    desired_speed = MAX(desired_speed, _speed_min);
}

// calculate the crosstrack error (does not rely on L1 controller)
float AR_WPNav::calc_crosstrack_error(const Location& current_loc) const
{
    if (!_orig_and_dest_valid) {
        return 0.0f;
    }

    // calculate the NE position of destination relative to origin
    Vector2f dest_from_origin = _oa_origin.get_distance_NE(_oa_destination);

    // return distance to origin if length of track is very small
    if (dest_from_origin.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(_oa_destination).length();
    }

    // convert to a vector indicating direction only
    dest_from_origin.normalize();

    // calculate the NE position of the vehicle relative to origin
    const Vector2f veh_from_origin = _oa_origin.get_distance_NE(current_loc);

    // calculate distance to target track, for reporting
    return veh_from_origin % dest_from_origin;
}
