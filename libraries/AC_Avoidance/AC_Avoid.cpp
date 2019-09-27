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

#include "AC_Avoid.h"
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Beacon/AP_Beacon.h>

#include <stdio.h>

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_STOP
#else
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_SLIDE
#endif

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable avoidance input sources
    // @Values: 0:None,1:UseFence,2:UseProximitySensor,3:UseFence and UseProximitySensor,4:UseBeaconFence,7:All
    // @Bitmask: 0:UseFence,1:UseProximitySensor,2:UseBeaconFence
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1,  AC_Avoid, _enabled, AC_AVOID_DEFAULT),

    // @Param: ANGLE_MAX
    // @DisplayName: Avoidance max lean angle in non-GPS flight modes
    // @Description: Max lean angle used to avoid obstacles while in non-GPS modes
    // @Units: cdeg
    // @Range: 0 4500
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 2,  AC_Avoid, _angle_max, 1000),

    // @Param: DIST_MAX
    // @DisplayName: Avoidance distance maximum in non-GPS flight modes
    // @Description: Distance from object at which obstacle avoidance will begin in non-GPS modes
    // @Units: m
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO("DIST_MAX", 3,  AC_Avoid, _dist_max, AC_AVOID_NONGPS_DIST_MAX_DEFAULT),

    // @Param: MARGIN
    // @DisplayName: Avoidance distance margin in GPS modes
    // @Description: Vehicle will attempt to stay at least this distance (in meters) from objects while in GPS modes
    // @Units: m
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("MARGIN", 4, AC_Avoid, _margin, 2.0f),

    // @Param: BEHAVE
    // @DisplayName: Avoidance behaviour
    // @Description: Avoidance behaviour (slide or stop)
    // @Values: 0:Slide,1:Stop
    // @User: Standard
    AP_GROUPINFO("BEHAVE", 5, AC_Avoid, _behavior, AP_AVOID_BEHAVE_DEFAULT),

    AP_GROUPEND
};

/// Constructor
AC_Avoid::AC_Avoid()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0) {
        adjust_velocity_circle_fence(kP, accel_cmss_limited, desired_vel_cms, dt);
        adjust_velocity_inclusion_and_exclusion_polygons(kP, accel_cmss_limited, desired_vel_cms, dt);
        adjust_velocity_inclusion_circles(kP, accel_cmss_limited, desired_vel_cms, dt);
        adjust_velocity_exclusion_circles(kP, accel_cmss_limited, desired_vel_cms, dt);
    }

    if ((_enabled & AC_AVOID_STOP_AT_BEACON_FENCE) > 0) {
        adjust_velocity_beacon_fence(kP, accel_cmss_limited, desired_vel_cms, dt);
    }

    if ((_enabled & AC_AVOID_USE_PROXIMITY_SENSOR) > 0 && _proximity_enabled) {
        adjust_velocity_proximity(kP, accel_cmss_limited, desired_vel_cms, dt);
    }
}

// convenience function to accept Vector3f.  Only x and y are adjusted
void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel_cms, float dt)
{
    Vector2f des_vel_xy(desired_vel_cms.x, desired_vel_cms.y);
    adjust_velocity(kP, accel_cmss, des_vel_xy, dt);
    desired_vel_cms.x = des_vel_xy.x;
    desired_vel_cms.y = des_vel_xy.y;
}

// adjust desired horizontal speed so that the vehicle stops before the fence or object
// accel (maximum acceleration/deceleration) is in m/s/s
// heading is in radians
// speed is in m/s
// kP should be zero for linear response, non-zero for non-linear response
void AC_Avoid::adjust_speed(float kP, float accel, float heading, float &speed, float dt)
{
    // convert heading and speed into velocity vector
    Vector2f vel_xy;
    vel_xy.x = cosf(heading) * speed * 100.0f;
    vel_xy.y = sinf(heading) * speed * 100.0f;
    adjust_velocity(kP, accel * 100.0f, vel_xy, dt);

    // adjust speed towards zero
    if (is_negative(speed)) {
        speed = -vel_xy.length() * 0.01f;
    } else {
        speed = vel_xy.length() * 0.01f;
    }
}

// adjust vertical climb rate so vehicle does not break the vertical fence
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms, float dt)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // do not adjust climb_rate if level or descending
    if (climb_rate_cms <= 0.0f) {
        return;
    }

    // limit acceleration
    const float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    bool limit_alt = false;
    float alt_diff = 0.0f;   // distance from altitude limit to vehicle in metres (positive means vehicle is below limit)

    const AP_AHRS &_ahrs = AP::ahrs();

    // calculate distance below fence
    AC_Fence *fence = AP::fence();
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0 && fence && (fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) > 0) {
        // calculate distance from vehicle to safe altitude
        float veh_alt;
        _ahrs.get_relative_position_D_home(veh_alt);
        // _fence.get_safe_alt_max() is UP, veh_alt is DOWN:
        alt_diff = fence->get_safe_alt_max() + veh_alt;
        limit_alt = true;
    }

    // calculate distance to (e.g.) optical flow altitude limit
    // AHRS values are always in metres
    float alt_limit;
    float curr_alt;
    if (_ahrs.get_hgt_ctrl_limit(alt_limit) &&
        _ahrs.get_relative_position_D_origin(curr_alt)) {
        // alt_limit is UP, curr_alt is DOWN:
        const float ctrl_alt_diff = alt_limit + curr_alt;
        if (!limit_alt || ctrl_alt_diff < alt_diff) {
            alt_diff = ctrl_alt_diff;
            limit_alt = true;
        }
    }

    // get distance from proximity sensor
    float proximity_alt_diff;
    AP_Proximity *proximity = AP::proximity();
    if (proximity && proximity->get_upward_distance(proximity_alt_diff)) {
        proximity_alt_diff -= _margin;
        if (!limit_alt || proximity_alt_diff < alt_diff) {
            alt_diff = proximity_alt_diff;
            limit_alt = true;
        }
    }

    // limit climb rate
    if (limit_alt) {
        // do not allow climbing if we've breached the safe altitude
        if (alt_diff <= 0.0f) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
            return;
        }

        // limit climb rate
        const float max_speed = get_max_speed(kP, accel_cmss_limited, alt_diff*100.0f, dt);
        climb_rate_cms = MIN(max_speed, climb_rate_cms);
    }
}

// adjust roll-pitch to push vehicle away from objects
// roll and pitch value are in centi-degrees
void AC_Avoid::adjust_roll_pitch(float &roll, float &pitch, float veh_angle_max)
{
    // exit immediately if proximity based avoidance is disabled
    if ((_enabled & AC_AVOID_USE_PROXIMITY_SENSOR) == 0 || !_proximity_enabled) {
        return;
    }

    // exit immediately if angle max is zero
    if (_angle_max <= 0.0f || veh_angle_max <= 0.0f) {
        return;
    }

    float roll_positive = 0.0f;    // maximum positive roll value
    float roll_negative = 0.0f;    // minimum negative roll value
    float pitch_positive = 0.0f;   // maximum positive pitch value
    float pitch_negative = 0.0f;   // minimum negative pitch value

    // get maximum positive and negative roll and pitch percentages from proximity sensor
    get_proximity_roll_pitch_pct(roll_positive, roll_negative, pitch_positive, pitch_negative);

    // add maximum positive and negative percentages together for roll and pitch, convert to centi-degrees
    Vector2f rp_out((roll_positive + roll_negative) * 4500.0f, (pitch_positive + pitch_negative) * 4500.0f);

    // apply avoidance angular limits
    // the object avoidance lean angle is never more than 75% of the total angle-limit to allow the pilot to override
    const float angle_limit = constrain_float(_angle_max, 0.0f, veh_angle_max * AC_AVOID_ANGLE_MAX_PERCENT);
    float vec_len = rp_out.length();
    if (vec_len > angle_limit) {
        rp_out *= (angle_limit / vec_len);
    }

    // add passed in roll, pitch angles
    rp_out.x += roll;
    rp_out.y += pitch;

    // apply total angular limits
    vec_len = rp_out.length();
    if (vec_len > veh_angle_max) {
        rp_out *= (veh_angle_max / vec_len);
    }

    // return adjusted roll, pitch
    roll = rp_out.x;
    pitch = rp_out.y;
}

/*
 * Limits the component of desired_vel_cms in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance_cm.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt) const
{
    const float max_speed = get_max_speed(kP, accel_cmss, limit_distance_cm, dt);
    // project onto limit direction
    const float speed = desired_vel_cms * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel_cms += limit_direction*(max_speed - speed);
    }
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const
{
    if (is_zero(kP)) {
        return safe_sqrt(2.0f * distance_cm * accel_cmss);
    } else {
        return AC_AttitudeControl::sqrt_controller(distance_cm, kP, accel_cmss, dt);
    }
}

/*
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    AC_Fence &_fence = *fence;

    // exit if circular fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return;
    }

    // exit if the circular fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) != 0) {
        return;
    }

    // get desired speed
    const float desired_speed = desired_vel_cms.length();
    if (is_zero(desired_speed)) {
        // no avoidance necessary when desired speed is zero
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // get position as a 2D offset from ahrs home
    Vector2f position_xy;
    if (!_ahrs.get_relative_position_NE_home(position_xy)) {
        // we have no idea where we are....
        return;
    }
    position_xy *= 100.0f; // m -> cm

    // get the fence radius in cm
    const float fence_radius = _fence.get_radius() * 100.0f;
    // get the margin to the fence in cm
    const float margin_cm = _fence.get_margin() * 100.0f;

    // get vehicle distance from home
    const float dist_from_home = position_xy.length();
    if (dist_from_home > fence_radius) {
        // outside of circular fence, no velocity adjustments
        return;
    }

    // vehicle is inside the circular fence
    if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_SLIDE) {
        // implement sliding behaviour
        const Vector2f stopping_point = position_xy + desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
        const float stopping_point_dist_from_home = stopping_point.length();
        if (stopping_point_dist_from_home <= fence_radius - margin_cm) {
            // stopping before before fence so no need to adjust
            return;
        }
        // unsafe desired velocity - will not be able to stop before reaching margin from fence
        // Project stopping point radially onto fence boundary
        // Adjusted velocity will point towards this projected point at a safe speed
        const Vector2f target_offset = stopping_point * ((fence_radius - margin_cm) / stopping_point_dist_from_home);
        const Vector2f target_direction = target_offset - position_xy;
        const float distance_to_target = target_direction.length();
        if (is_positive(distance_to_target)) {
            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
            desired_vel_cms = target_direction * (MIN(desired_speed,max_speed) / distance_to_target);
        }
    } else {
        // implement stopping behaviour
        // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
        const Vector2f stopping_point_plus_margin = position_xy + desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
        const float stopping_point_plus_margin_dist_from_home = stopping_point_plus_margin.length();
        if (dist_from_home >= fence_radius - margin_cm) {
            // vehicle has already breached margin around fence
            // if stopping point is even further from home (i.e. in wrong direction) then adjust speed to zero
            // otherwise user is backing away from fence so do not apply limits
            if (stopping_point_plus_margin_dist_from_home >= dist_from_home) {
                desired_vel_cms.zero();
            }
        } else {
            // shorten vector without adjusting its direction
            Vector2f intersection;
            if (Vector2f::circle_segment_intersection(position_xy, stopping_point_plus_margin, Vector2f(0.0f,0.0f), fence_radius - margin_cm, intersection)) {
                const float distance_to_target = MAX((intersection - position_xy).length() - margin_cm, 0.0f);
                const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                if (max_speed < desired_speed) {
                    desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                }
            }
        }
    }
}

/*
 * Adjusts the desired velocity for the exclusion polygons
 */
void AC_Avoid::adjust_velocity_inclusion_and_exclusion_polygons(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // iterate through inclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
        if (num_points < 3) {
            // ignore exclusion polygons with less than 3 points
            continue;
        }
        // adjust velocity
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, true, fence->get_margin(), dt, true);
    }

    // iterate through exclusion polygons
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
        if (num_points < 3) {
            // ignore exclusion polygons with less than 3 points
            continue;
        }
        // adjust velocity
        adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, true, fence->get_margin(), dt, false);
    }
}

/*
 * Adjusts the desired velocity for the inclusion circles
 */
void AC_Avoid::adjust_velocity_inclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // return immediately if no inclusion circles
    const uint8_t num_circles = fence->polyfence().get_inclusion_circle_count();
    if (num_circles == 0) {
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // get desired speed
    const float desired_speed = desired_vel_cms.length();
    if (is_zero(desired_speed)) {
        // no avoidance necessary when desired speed is zero
        return;
    }

    // get vehicle position
    Vector2f position_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(position_NE)) {
        // do not limit velocity if we don't have a position estimate
        return;
    }
    position_NE = position_NE * 100.0f;  // m to cm

    // get the margin to the fence in cm
    const float margin_cm = fence->get_margin() * 100.0f;

    // get stopping distance as an offset from the vehicle
    Vector2f stopping_offset;
    switch ((AC_Avoid::BehaviourType)_behavior.get()) {
        case BEHAVIOR_SLIDE:
            stopping_offset = desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
            break;
        case BEHAVIOR_STOP:
            // calculate stopping point plus a margin so we look forward far enough to intersect with circular fence
            stopping_offset = desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
            break;
    }

    // iterate through inclusion circles
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {
            // get position relative to circle's center
            const Vector2f position_NE_rel = (position_NE - center_pos_cm);

            // if we are outside this circle do not limit velocity for this circle
            const float dist_sq_cm = position_NE_rel.length_squared();
            const float radius_cm = (radius * 100.0f);
            if (dist_sq_cm > sq(radius_cm)) {
                continue;
            }

            switch ((AC_Avoid::BehaviourType)_behavior.get()) {
                case BEHAVIOR_SLIDE: {
                    // implement sliding behaviour
                    const Vector2f stopping_point = position_NE_rel + stopping_offset;
                    const float stopping_point_dist = stopping_point.length();
                    if (is_zero(stopping_point_dist) || (stopping_point_dist <= (radius_cm - margin_cm))) {
                        // stopping before before fence so no need to adjust for this circle
                        continue;
                    }
                    // unsafe desired velocity - will not be able to stop before reaching margin from fence
                    // project stopping point radially onto fence boundary
                    // adjusted velocity will point towards this projected point at a safe speed
                    const Vector2f target_offset = stopping_point * ((radius_cm - margin_cm) / stopping_point_dist);
                    const Vector2f target_direction = target_offset - position_NE_rel;
                    const float distance_to_target = target_direction.length();
                    if (is_positive(distance_to_target)) {
                        const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                        desired_vel_cms = target_direction * (MIN(desired_speed,max_speed) / distance_to_target);
                    }
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    const Vector2f stopping_point_plus_margin = position_NE_rel + stopping_offset;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm >= radius_cm - margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is even further from center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        if (stopping_point_plus_margin.length() >= dist_cm) {
                            desired_vel_cms.zero();
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        Vector2f intersection;
                        if (Vector2f::circle_segment_intersection(position_NE_rel, stopping_point_plus_margin, Vector2f(0.0f,0.0f), radius_cm - margin_cm, intersection)) {
                            const float distance_to_target = MAX((intersection - position_NE_rel).length() - margin_cm, 0.0f);
                            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                            if (max_speed < desired_speed) {
                                desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
}

/*
 * Adjusts the desired velocity for the exclusion circles
 */
void AC_Avoid::adjust_velocity_exclusion_circles(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    const AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    // return immediately if no inclusion circles
    const uint8_t num_circles = fence->polyfence().get_exclusion_circle_count();
    if (num_circles == 0) {
        return;
    }

    // exit if polygon fences are not enabled
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // get desired speed
    const float desired_speed = desired_vel_cms.length();
    if (is_zero(desired_speed)) {
        // no avoidance necessary when desired speed is zero
        return;
    }

    // get vehicle position
    Vector2f position_NE;
    if (!AP::ahrs().get_relative_position_NE_origin(position_NE)) {
        // do not limit velocity if we don't have a position estimate
        return;
    }
    position_NE = position_NE * 100.0f;  // m to cm

    // get the margin to the fence in cm
    const float margin_cm = fence->get_margin() * 100.0f;

    // calculate stopping distance as an offset from the vehicle (only used for BEHAVIOR_STOP)
    // add a margin so we look forward far enough to intersect with circular fence
    Vector2f stopping_offset;
    if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_STOP) {
        stopping_offset = desired_vel_cms*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, desired_speed))/desired_speed);
    }

    // iterate through exclusion circles
    for (uint8_t i = 0; i < num_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            // get position relative to circle's center
            const Vector2f position_NE_rel = (position_NE - center_pos_cm);

            // if we are inside this circle do not limit velocity for this circle
            const float dist_sq_cm = position_NE_rel.length_squared();
            const float radius_cm = (radius * 100.0f);
            if (dist_sq_cm < sq(radius_cm)) {
                continue;
            }

            switch ((AC_Avoid::BehaviourType)_behavior.get()) {
                case BEHAVIOR_SLIDE: {
                    // vector from current position to circle's center
                    Vector2f limit_direction = center_pos_cm - position_NE;
                    if (limit_direction.is_zero()) {
                        // vehicle is exactly on circle center so do not limit velocity
                        continue;
                    }
                    // calculate distance to edge of circle
                    const float limit_distance_cm = limit_direction.length() - radius_cm;
                    if (!is_positive(limit_distance_cm)) {
                        // vehicle is within circle so do not limit velocity
                        continue;
                    }
                    // vehicle is outside the circle, adjust velocity to stay outside
                    limit_direction.normalize();
                    limit_velocity(kP, accel_cmss, desired_vel_cms, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                }
                break;
                case BEHAVIOR_STOP: {
                    // implement stopping behaviour
                    const Vector2f stopping_point_plus_margin = position_NE_rel + stopping_offset;
                    const float dist_cm = safe_sqrt(dist_sq_cm);
                    if (dist_cm < radius_cm + margin_cm) {
                        // vehicle has already breached margin around fence
                        // if stopping point is closer to center (i.e. in wrong direction) then adjust speed to zero
                        // otherwise user is backing away from fence so do not apply limits
                        if (stopping_point_plus_margin.length() <= dist_cm) {
                            desired_vel_cms.zero();
                            return;
                        }
                    } else {
                        // shorten vector without adjusting its direction
                        Vector2f intersection;
                        if (Vector2f::circle_segment_intersection(position_NE_rel, stopping_point_plus_margin, Vector2f(0.0f,0.0f), radius_cm + margin_cm, intersection)) {
                            const float distance_to_target = MAX((intersection - position_NE_rel).length() - margin_cm, 0.0f);
                            const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                            if (max_speed < desired_speed) {
                                desired_vel_cms *= MAX(max_speed, 0.0f) / desired_speed;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
}

/*
 * Adjusts the desired velocity for the beacon fence.
 */
void AC_Avoid::adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    AP_Beacon *_beacon = AP::beacon();

    // exit if the beacon is not present
    if (_beacon == nullptr) {
        return;
    }

    // get boundary from beacons
    uint16_t num_points = 0;
    const Vector2f* boundary = _beacon->get_boundary_points(num_points);
    if ((boundary == nullptr) || (num_points == 0)) {
        return;
    }

    // adjust velocity using beacon
    float margin = 0;
    if (AP::fence()) {
        margin = AP::fence()->get_margin();
    }
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, true, margin, dt, true);
}

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // exit immediately if proximity sensor is not present
    AP_Proximity *proximity = AP::proximity();
    if (!proximity) {
        return;
    }

    AP_Proximity &_proximity = *proximity;

    if (_proximity.get_status() != AP_Proximity::Status::Good) {
        return;
    }

    // get boundary from proximity sensor
    uint16_t num_points = 0;
    const Vector2f *boundary = _proximity.get_boundary_points(num_points);
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, false, _margin, dt, true);
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f* boundary, uint16_t num_points, bool earth_frame, float margin, float dt, bool stay_inside)
{
    // exit if there are no points
    if (boundary == nullptr || num_points == 0) {
        return;
    }

    // exit immediately if no desired velocity
    if (desired_vel_cms.is_zero()) {
        return;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    // do not adjust velocity if vehicle is outside the polygon fence
    Vector2f position_xy;
    if (earth_frame) {
        if (!_ahrs.get_relative_position_NE_origin(position_xy)) {
            // boundary is in earth frame but we have no idea
            // where we are
            return;
        }
        position_xy = position_xy * 100.0f;  // m to cm
    }

    // return if we have already breached polygon
    const bool inside_polygon = !Polygon_outside(position_xy, boundary, num_points);
    if (inside_polygon != stay_inside) {
        return;
    }

    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel(desired_vel_cms);

    // if boundary points are in body-frame, rotate velocity vector from earth frame to body-frame
    if (!earth_frame) {
        safe_vel.x = desired_vel_cms.y * _ahrs.sin_yaw() + desired_vel_cms.x * _ahrs.cos_yaw(); // right
        safe_vel.y = desired_vel_cms.y * _ahrs.cos_yaw() - desired_vel_cms.x * _ahrs.sin_yaw(); // forward
    }

    // calc margin in cm
    const float margin_cm = MAX(margin * 100.0f, 0.0f);

    // for stopping
    const float speed = safe_vel.length();
    const Vector2f stopping_point_plus_margin = position_xy + safe_vel*((2.0f + margin_cm + get_stopping_distance(kP, accel_cmss, speed))/speed);

    for (uint16_t i=0; i<num_points; i++) {
        uint16_t j = i+1;
        if (j >= num_points) {
            j = 0;
        }
        // end points of current edge
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_SLIDE) {
            // vector from current position to closest point on current edge
            Vector2f limit_direction = Vector2f::closest_point(position_xy, start, end) - position_xy;
            // distance to closest point
            const float limit_distance_cm = limit_direction.length();
            if (!is_zero(limit_distance_cm)) {
                // We are strictly inside the given edge.
                // Adjust velocity to not violate this edge.
                limit_direction /= limit_distance_cm;
                limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
            } else {
                // We are exactly on the edge - treat this as a fence breach.
                // i.e. do not adjust velocity.
                return;
            }
        } else {
            // find intersection with line segment
            Vector2f intersection;
            if (Vector2f::segment_intersection(position_xy, stopping_point_plus_margin, start, end, intersection)) {
                // vector from current position to point on current edge
                Vector2f limit_direction = intersection - position_xy;
                const float limit_distance_cm = limit_direction.length();
                if (!is_zero(limit_distance_cm)) {
                    if (limit_distance_cm <= margin_cm) {
                        // we are within the margin so stop vehicle
                        safe_vel.zero();
                    } else {
                        // vehicle inside the given edge, adjust velocity to not violate this edge
                        limit_direction /= limit_distance_cm;
                        limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance_cm - margin_cm, 0.0f), dt);
                    }
                } else {
                    // We are exactly on the edge - treat this as a fence breach.
                    // i.e. do not adjust velocity.
                    return;
                }
            }
        }
    }

    // set modified desired velocity vector
    if (earth_frame) {
        desired_vel_cms = safe_vel;
    } else {
        // if points were in body-frame, rotate resulting vector back to earth-frame
        desired_vel_cms.x = safe_vel.x * _ahrs.cos_yaw() - safe_vel.y * _ahrs.sin_yaw();
        desired_vel_cms.y = safe_vel.x * _ahrs.sin_yaw() + safe_vel.y * _ahrs.cos_yaw();
    }
}

/*
 * Computes distance required to stop, given current speed.
 *
 * Implementation copied from AC_PosControl.
 */
float AC_Avoid::get_stopping_distance(float kP, float accel_cmss, float speed_cms) const
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (accel_cmss <= 0.0f || is_zero(speed_cms)) {
        return 0.0f;
    }

    // handle linear deceleration
    if (kP <= 0.0f) {
        return 0.5f * sq(speed_cms) / accel_cmss;
    }

    // calculate distance within which we can stop
    // accel_cmss/kP is the point at which velocity switches from linear to sqrt
    if (speed_cms < accel_cmss/kP) {
        return speed_cms/kP;
    } else {
        // accel_cmss/(2.0f*kP*kP) is the distance at which we switch from linear to sqrt response
        return accel_cmss/(2.0f*kP*kP) + (speed_cms*speed_cms)/(2.0f*accel_cmss);
    }
}

// convert distance (in meters) to a lean percentage (in 0~1 range) for use in manual flight modes
float AC_Avoid::distance_to_lean_pct(float dist_m)
{
    // ignore objects beyond DIST_MAX
    if (dist_m < 0.0f || dist_m >= _dist_max || _dist_max <= 0.0f) {
        return 0.0f;
    }
    // inverted but linear response
    return 1.0f - (dist_m / _dist_max);
}

// returns the maximum positive and negative roll and pitch percentages (in -1 ~ +1 range) based on the proximity sensor
void AC_Avoid::get_proximity_roll_pitch_pct(float &roll_positive, float &roll_negative, float &pitch_positive, float &pitch_negative)
{
    AP_Proximity *proximity = AP::proximity();
    if (proximity == nullptr) {
        return;
    }
    AP_Proximity &_proximity = *proximity;

    // exit immediately if proximity sensor is not present
    if (_proximity.get_status() != AP_Proximity::Status::Good) {
        return;
    }

    const uint8_t obj_count = _proximity.get_object_count();

    // if no objects return
    if (obj_count == 0) {
        return;
    }

    // calculate maximum roll, pitch values from objects
    for (uint8_t i=0; i<obj_count; i++) {
        float ang_deg, dist_m;
        if (_proximity.get_object_angle_and_distance(i, ang_deg, dist_m)) {
            if (dist_m < _dist_max) {
                // convert distance to lean angle (in 0 to 1 range)
                const float lean_pct = distance_to_lean_pct(dist_m);
                // convert angle to roll and pitch lean percentages
                const float angle_rad = radians(ang_deg);
                const float roll_pct = -sinf(angle_rad) * lean_pct;
                const float pitch_pct = cosf(angle_rad) * lean_pct;
                // update roll, pitch maximums
                if (roll_pct > 0.0f) {
                    roll_positive = MAX(roll_positive, roll_pct);
                } else if (roll_pct < 0.0f) {
                    roll_negative = MIN(roll_negative, roll_pct);
                }
                if (pitch_pct > 0.0f) {
                    pitch_positive = MAX(pitch_positive, pitch_pct);
                } else if (pitch_pct < 0.0f) {
                    pitch_negative = MIN(pitch_negative, pitch_pct);
                }
            }
        }
    }
}

// singleton instance
AC_Avoid *AC_Avoid::_singleton;

namespace AP {

AC_Avoid *ac_avoid()
{
    return AC_Avoid::get_singleton();
}

}
