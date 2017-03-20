#include "AC_Avoid.h"

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable stopping at fence
    // @Values: 0:None,1:StopAtFence,2:UseProximitySensor,3:All
    // @Bitmask: 0:StopAtFence,1:UseProximitySensor
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1,  AC_Avoid, _enabled, AC_AVOID_ALL),

    // @Param: ANGLE_MAX
    // @DisplayName: Avoidance max lean angle in non-GPS flight modes
    // @Description: Max lean angle used to avoid obstacles while in non-GPS modes
    // @Range: 0 4500
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 2,  AC_Avoid, _angle_max, 1000),

    // @Param: DIST_MAX
    // @DisplayName: Avoidance distance maximum in non-GPS flight modes
    // @Description: Distance from object at which obstacle avoidance will begin in non-GPS modes
    // @Units: meters
    // @Range: 3 30
    // @User: Standard
    AP_GROUPINFO("DIST_MAX", 3,  AC_Avoid, _dist_max, AC_AVOID_NONGPS_DIST_MAX_DEFAULT),

    AP_GROUPEND
};

/// Constructor
AC_Avoid::AC_Avoid(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AC_Fence& fence, const AP_Proximity& proximity)
    : _ahrs(ahrs),
      _inav(inav),
      _fence(fence),
      _proximity(proximity)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0) {
        adjust_velocity_circle_fence(kP, accel_cmss_limited, desired_vel);
        adjust_velocity_polygon_fence(kP, accel_cmss_limited, desired_vel);
    }

    if ((_enabled & AC_AVOID_USE_PROXIMITY_SENSOR) > 0 && _proximity_enabled) {
        adjust_velocity_proximity(kP, accel_cmss_limited, desired_vel);
    }
}

// convenience function to accept Vector3f.  Only x and y are adjusted
void AC_Avoid::adjust_velocity(float kP, float accel_cmss, Vector3f &desired_vel)
{
    Vector2f des_vel_xy(desired_vel.x, desired_vel.y);
    adjust_velocity(kP, accel_cmss, des_vel_xy);
    desired_vel.x = des_vel_xy.x;
    desired_vel.y = des_vel_xy.y;
}

// adjust vertical climb rate so vehicle does not break the vertical fence
void AC_Avoid::adjust_velocity_z(float kP, float accel_cmss, float& climb_rate_cms)
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
    float alt_diff_cm = 0.0f;   // distance from altitude limit to vehicle in cm (positive means vehicle is below limit)

    // calculate distance below fence
    if ((_enabled & AC_AVOID_STOP_AT_FENCE) > 0 && (_fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) > 0) {
        // calculate distance from vehicle to safe altitude
        float veh_alt = get_alt_above_home();
        alt_diff_cm = _fence.get_safe_alt_max() * 100.0f - veh_alt;
        limit_alt = true;
    }

    // calculate distance to optical flow altitude limit
    float ekf_alt_limit_cm;
    if (_inav.get_hgt_ctrl_limit(ekf_alt_limit_cm)) {
        float ekf_alt_diff_cm = ekf_alt_limit_cm - _inav.get_altitude();
        if (!limit_alt || ekf_alt_diff_cm < alt_diff_cm) {
            alt_diff_cm = ekf_alt_diff_cm;
        }
        limit_alt = true;
    }

    // get distance from proximity sensor (in meters, convert to cm)
    float proximity_alt_diff_m;
    if (_proximity.get_upward_distance(proximity_alt_diff_m)) {
        float proximity_alt_diff_cm = (proximity_alt_diff_m - AC_AVOID_UPWARD_MARGIN_M) * 100.0f;
        if (!limit_alt || proximity_alt_diff_cm < alt_diff_cm) {
            alt_diff_cm = proximity_alt_diff_cm;
        }
        limit_alt = true;
    }

    // limit climb rate
    if (limit_alt) {
        // do not allow climbing if we've breached the safe altitude
        if (alt_diff_cm <= 0.0f) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
            return;
        }

        // limit climb rate
        const float max_speed = get_max_speed(kP, accel_cmss_limited, alt_diff_cm);
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
    float pitch_positive = 0.0f;   // maximum position pitch value
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
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle_fence(float kP, float accel_cmss, Vector2f &desired_vel)
{
    // exit if circular fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return;
    }

    // exit if the circular fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) != 0) {
        return;
    }

    // get position as a 2D offset in cm from ahrs home
    const Vector2f position_xy = get_position();

    float speed = desired_vel.length();
    // get the fence radius in cm
    const float fence_radius = _fence.get_radius() * 100.0f;
    // get the margin to the fence in cm
    const float margin = get_margin();

    if (!is_zero(speed) && position_xy.length() <= fence_radius) {
        // Currently inside circular fence
        Vector2f stopping_point = position_xy + desired_vel*(get_stopping_distance(kP, accel_cmss, speed)/speed);
        float stopping_point_length = stopping_point.length();
        if (stopping_point_length > fence_radius - margin) {
            // Unsafe desired velocity - will not be able to stop before fence breach
            // Project stopping point radially onto fence boundary
            // Adjusted velocity will point towards this projected point at a safe speed
            Vector2f target = stopping_point * ((fence_radius - margin) / stopping_point_length);
            Vector2f target_direction = target - position_xy;
            float distance_to_target = target_direction.length();
            float max_speed = get_max_speed(kP, accel_cmss, distance_to_target);
            desired_vel = target_direction * (MIN(speed,max_speed) / distance_to_target);
        }
    }
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon_fence(float kP, float accel_cmss, Vector2f &desired_vel)
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
    if (desired_vel.is_zero()) {
        return;
    }

    // get polygon boundary
    // Note: first point in list is the return-point (which copter does not use)
    uint16_t num_points;
    Vector2f* boundary = _fence.get_polygon_points(num_points);

    // adjust velocity using polygon
    adjust_velocity_polygon(kP, accel_cmss, desired_vel, boundary, num_points, true);
}

/*
 * Adjusts the desired velocity based on output from the proximity sensor
 */
void AC_Avoid::adjust_velocity_proximity(float kP, float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if proximity sensor is not present
    if (_proximity.get_status() != AP_Proximity::Proximity_Good) {
        return;
    }

    // exit immediately if no desired velocity
    if (desired_vel.is_zero()) {
        return;
    }

    // get boundary from proximity sensor
    uint16_t num_points;
    const Vector2f *boundary = _proximity.get_boundary_points(num_points);
    adjust_velocity_polygon(kP, accel_cmss, desired_vel, boundary, num_points, false);
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_polygon(float kP, float accel_cmss, Vector2f &desired_vel, const Vector2f* boundary, uint16_t num_points, bool earth_frame)
{
    // exit if there are no points
    if (boundary == nullptr || num_points == 0) {
        return;
    }

    // do not adjust velocity if vehicle is outside the polygon fence
    Vector3f position;
    if (earth_frame) {
        position = _inav.get_position();
    }
    Vector2f position_xy(position.x, position.y);
    if (_fence.boundary_breached(position_xy, num_points, boundary)) {
        return;
    }

    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel(desired_vel);

    // if boundary points are in body-frame, rotate velocity vector from earth frame to body-frame
    if (!earth_frame) {
        safe_vel.x = desired_vel.y * _ahrs.sin_yaw() + desired_vel.x * _ahrs.cos_yaw(); // right
        safe_vel.y = desired_vel.y * _ahrs.cos_yaw() - desired_vel.x * _ahrs.sin_yaw(); // forward
    }

    uint16_t i, j;
    for (i = 1, j = num_points-1; i < num_points; j = i++) {
        // end points of current edge
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        // vector from current position to closest point on current edge
        Vector2f limit_direction = Vector2f::closest_point(position_xy, start, end) - position_xy;
        // distance to closest point
        const float limit_distance = limit_direction.length();
        if (!is_zero(limit_distance)) {
            // We are strictly inside the given edge.
            // Adjust velocity to not violate this edge.
            limit_direction /= limit_distance;
            limit_velocity(kP, accel_cmss, safe_vel, limit_direction, MAX(limit_distance - get_margin(),0.0f));
        } else {
            // We are exactly on the edge - treat this as a fence breach.
            // i.e. do not adjust velocity.
            return;
        }
    }

    // set modified desired velocity vector
    if (earth_frame) {
        desired_vel = safe_vel;
    } else {
        // if points were in body-frame, rotate resulting vector back to earth-frame
        desired_vel.x = safe_vel.x * _ahrs.cos_yaw() - safe_vel.y * _ahrs.sin_yaw();
        desired_vel.y = safe_vel.x * _ahrs.sin_yaw() + safe_vel.y * _ahrs.cos_yaw();
    }
}

/*
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity(float kP, float accel_cmss, Vector2f &desired_vel, const Vector2f& limit_direction, float limit_distance) const
{
    const float max_speed = get_max_speed(kP, accel_cmss, limit_distance);
    // project onto limit direction
    const float speed = desired_vel * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel += limit_direction*(max_speed - speed);
    }
}

/*
 * Gets the current xy-position, relative to home (not relative to EKF origin)
 */
Vector2f AC_Avoid::get_position() const
{
    const Vector3f position_xyz = _inav.get_position();
    const Vector2f position_xy(position_xyz.x,position_xyz.y);
    const Vector2f diff = location_diff(_inav.get_origin(),_ahrs.get_home()) * 100.0f;
    return position_xy - diff;
}

/*
 * Gets the altitude above home in cm
 */
float AC_Avoid::get_alt_above_home() const
{
    // vehicle's alt above ekf origin + ekf origin's alt above sea level - home's alt above sea level
    return _inav.get_altitude() + _inav.get_origin().alt - _ahrs.get_home().alt;
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float kP, float accel_cmss, float distance) const
{
    return AC_AttitudeControl::sqrt_controller(distance, kP, accel_cmss);
}

/*
 * Computes distance required to stop, given current speed.
 *
 * Implementation copied from AC_PosControl.
 */
float AC_Avoid::get_stopping_distance(float kP, float accel_cmss, float speed) const
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || accel_cmss <= 0.0f || is_zero(speed)) {
        return 0.0f;
    }

    // calculate distance within which we can stop
    // accel_cmss/kP is the point at which velocity switches from linear to sqrt
    if (speed < accel_cmss/kP) {
        return speed/kP;
    } else {
        // accel_cmss/(2.0f*kP*kP) is the distance at which we switch from linear to sqrt response
        return accel_cmss/(2.0f*kP*kP) + (speed*speed)/(2.0f*accel_cmss);
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
                }
                if (roll_pct < 0.0f) {
                    roll_negative = MIN(roll_negative, roll_pct);
                }
                if (pitch_pct > 0.0f) {
                    pitch_positive = MAX(pitch_positive, pitch_pct);
                }
                if (pitch_pct < 0.0f) {
                    pitch_negative = MIN(pitch_negative, pitch_pct);
                }
            }
        }
    }
}
