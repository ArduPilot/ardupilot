#include "AC_Avoid.h"

#if APM_BUILD_TYPE(APM_BUILD_APMrover2)
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_STOP
#else
 # define AP_AVOID_BEHAVE_DEFAULT AC_Avoid::BehaviourType::BEHAVIOR_SLIDE
#endif

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable stopping at fence
    // @Values: 0:None,1:StopAtFence,2:UseProximitySensor,3:StopAtFence and UseProximitySensor,4:StopAtBeaconFence,7:All
    // @Bitmask: 0:StopAtFence,1:UseProximitySensor,2:StopAtBeaconFence
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
AC_Avoid::AC_Avoid(const AP_AHRS& ahrs, const AC_Fence& fence, const AP_Proximity& proximity, const AP_Beacon* beacon)
    : _ahrs(ahrs),
      _fence(fence),
      _proximity(proximity),
      _beacon(beacon)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0) {
        adjust_velocity_circle_fence(kP, accel_cmss_limited, desired_vel_cms, dt);
        adjust_velocity_polygon_fence(kP, accel_cmss_limited, desired_vel_cms, dt);
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
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    bool limit_alt = false;
    float alt_diff = 0.0f;   // distance from altitude limit to vehicle in metres (positive means vehicle is below limit)

    // calculate distance below fence
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0 && (_fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) > 0) {
        // calculate distance from vehicle to safe altitude
        float veh_alt;
        _ahrs.get_relative_position_D_home(veh_alt);
        // _fence.get_safe_alt_max() is UP, veh_alt is DOWN:
        alt_diff = _fence.get_safe_alt_max() + veh_alt;
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
    if (_proximity.get_upward_distance(proximity_alt_diff)) {
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
    // exit if circular fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return;
    }

    // exit if the circular fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) != 0) {
        return;
    }

    // get position as a 2D offset from ahrs home
    Vector2f position_xy;
    if (!_ahrs.get_relative_position_NE_home(position_xy)) {
        // we have no idea where we are....
        return;
    }
    position_xy *= 100.0f; // m -> cm

    float speed = desired_vel_cms.length();
    // get the fence radius in cm
    const float fence_radius = _fence.get_radius() * 100.0f;
    // get the margin to the fence in cm
    const float margin_cm = _fence.get_margin() * 100.0f;

    if (!is_zero(speed) && position_xy.length() <= fence_radius) {
        // Currently inside circular fence
        Vector2f stopping_point = position_xy + desired_vel_cms*(get_stopping_distance(kP, accel_cmss, speed)/speed);
        float stopping_point_length = stopping_point.length();
        if (stopping_point_length > fence_radius - margin_cm) {
            // Unsafe desired velocity - will not be able to stop before fence breach
            if ((AC_Avoid::BehaviourType)_behavior.get() == BEHAVIOR_SLIDE) {
                // Project stopping point radially onto fence boundary
                // Adjusted velocity will point towards this projected point at a safe speed
                const Vector2f target = stopping_point * ((fence_radius - margin_cm) / stopping_point_length);
                const Vector2f target_direction = target - position_xy;
                const float distance_to_target = target_direction.length();
                const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                desired_vel_cms = target_direction * (MIN(speed,max_speed) / distance_to_target);
            } else {
                // shorten vector without adjusting its direction
                Vector2f intersection;
                if (Vector2f::circle_segment_intersection(position_xy, stopping_point, Vector2f(0.0f,0.0f), fence_radius, intersection)) {
                    const float distance_to_target = MAX((intersection - position_xy).length() - margin_cm, 0.0f);
                    const float max_speed = get_max_speed(kP, accel_cmss, distance_to_target, dt);
                    if (max_speed < speed) {
                        desired_vel_cms *= MAX(max_speed, 0.0f) / speed;
                    }
                }
            }
        }
    }
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // exit if the polygon fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // exit if the polygon fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_POLYGON) != 0) {
        return;
    }

    // exit immediately if no desired velocity
    if (desired_vel_cms.is_zero()) {
        return;
    }

    // get polygon boundary
    // Note: first point in list is the return-point (which copter does not use)
    uint16_t num_points;
    Vector2f* boundary = _fence.get_polygon_points(num_points);

    // adjust velocity using polygon
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, true, _fence.get_margin(), dt);
}

/*
 * Adjusts the desired velocity for the beacon fence.
 */
void AC_Avoid::adjust_velocity_beacon_fence(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // exit if the beacon is not present
    if (_beacon == nullptr) {
        return;
    }

    // exit immediately if no desired velocity
    if (desired_vel_cms.is_zero()) {
        return;
    }

    // get boundary from beacons
    uint16_t num_points;
    const Vector2f* boundary = _beacon->get_boundary_points(num_points);
    if (boundary == nullptr || num_points == 0) {
        return;
    }

    // adjust velocity using beacon
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, true, _fence.get_margin(), dt);
}

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel_cms, float dt)
{
    // exit immediately if proximity sensor is not present
    if (_proximity.get_status() != AP_Proximity::Proximity_Good) {
        return;
    }

    // exit immediately if no desired velocity
    if (desired_vel_cms.is_zero()) {
        return;
    }

    // get boundary from proximity sensor
    uint16_t num_points;
    const Vector2f *boundary = _proximity.get_boundary_points(num_points);
    adjust_velocity_polygon(kP, accel_cmss, desired_vel_cms, boundary, num_points, false, _margin, dt);
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f* boundary, uint16_t num_points, bool earth_frame, float margin, float dt)
{
    // exit if there are no points
    if (boundary == nullptr || num_points == 0) {
        return;
    }

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

    if (_fence.boundary_breached(position_xy, num_points, boundary)) {
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
    float margin_cm = MAX(margin * 100.0f, 0.0f);

    // for stopping
    float speed = safe_vel.length();
    Vector2f stopping_point = position_xy + safe_vel*((2.0f + get_stopping_distance(kP, accel_cmss, speed))/speed);

    uint16_t i, j;
    for (i = 1, j = num_points-1; i < num_points; j = i++) {
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
            if (Vector2f::segment_intersection(position_xy, stopping_point, start, end, intersection)) {
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
    // exit immediately if proximity sensor is not present
    if (_proximity.get_status() != AP_Proximity::Proximity_Good) {
        return;
    }

    uint8_t obj_count = _proximity.get_object_count();

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
