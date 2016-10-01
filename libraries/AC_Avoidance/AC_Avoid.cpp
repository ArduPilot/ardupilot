#include "AC_Avoid.h"

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable stopping at fence
    // @Values: 0:None,1:StopAtFence
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1,  AC_Avoid, _enabled, AC_AVOID_STOP_AT_FENCE),

    AP_GROUPEND
};

/// Constructor
AC_Avoid::AC_Avoid(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AC_Fence& fence)
    : _ahrs(ahrs),
      _inav(inav),
      _fence(fence)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Avoid::adjust_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    if (_enabled == AC_AVOID_STOP_AT_FENCE) {
        adjust_velocity_circle(kP, accel_cmss_limited, desired_vel);
        adjust_velocity_poly(kP, accel_cmss_limited, desired_vel);
    }
}

// convenience function to accept Vector3f.  Only x and y are adjusted
void AC_Avoid::adjust_velocity(const float kP, const float accel_cmss, Vector3f &desired_vel)
{
    Vector2f des_vel_xy(desired_vel.x, desired_vel.y);
    adjust_velocity(kP, accel_cmss, des_vel_xy);
    desired_vel.x = des_vel_xy.x;
    desired_vel.y = des_vel_xy.y;
}

/*
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle(const float kP, const float accel_cmss, Vector2f &desired_vel)
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
 *
 * This function is supposed to solve the following optimization problem.
 * Find the vector safe_vel that is closest to desired_vel,
 * subject to the following constraints for each edge:
 * Let P be the closest point to the current position of the vehicle.
 * Let U be the unit vector in the direction of P from the current position.
 * Let D be the distance from the current position to P.
 *
 *    safe_vel must satisfy safe_vel * P <= max_speed(D)
 */
void AC_Avoid::adjust_velocity_poly(const float kP, const float accel_cmss, Vector2f &desired_vel)
{
    // exit if the polygon fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // exit if the polygon fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_POLYGON) != 0) {
        return;
    }

    // get polygon boundary
    // Note: first point in list is the return-point (which copter does not use)
    uint16_t num_points;
    Vector2f* boundary = _fence.get_polygon_points(num_points);

    // exit if there are no points
    if (boundary == NULL || num_points == 0) {
        return;
    }

    // do not adjust velocity if vehicle is outside the polygon fence
    const Vector3f& position = _inav.get_position();
    Vector2f position_xy(position.x, position.y);
    if (_fence.boundary_breached(position_xy, num_points, boundary)) {
        return;
    }

    // Two arrays giving constraints induced by the current position and fence
    // The ith constraint is:
    //    velocity * constraint_directions[i] <= max_speeds[i]
    Vector2f constraint_directions[num_points - 1];
    float max_speeds[num_points - 1];

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
            constraint_directions[i - 1] = limit_direction;
            max_speeds[i - 1] = get_max_speed(kP, accel_cmss, limit_distance - get_margin());
        } else {
            // We are exactly on the edge - treat this as a fence breach.
            // i.e. do not adjust velocity.
            return;
        }
    }

    adjust_velocity_constraints(kP, accel_cmss, constraint_directions, max_speeds, num_points - 1, desired_vel);
}

/*
 * Adjusts the velocity to satisfy the given set of constraints. The ith constraint is
 *    desired_vel * constraint_directions[i] <= max_speeds[i]
 */
void AC_Avoid::adjust_velocity_constraints(const float kP, const float accel_cmss, const Vector2f* constraint_directions, const float* max_speeds, const uint16_t num_constraints, Vector2f& desired_vel)
{
    for (uint16_t i = 0; i < num_constraints; i++) {
        if (limit_velocity(kP, accel_cmss, desired_vel, constraint_directions[i], max_speeds[i])) {
            // If velocity was adjusted, iterate through all prior constraints to check for
            // violations
            for (uint16_t j = 0; j < i; j++) {
                if (desired_vel * constraint_directions[j] > max_speeds[j]) {
                    // New velocity violates previous constraint
                    // Adjust velocity to intersection of the two constraints.
                    // This should be the closest point that satisfies both constraints.
                    // The following will do nothing if there is no intersection,
                    // but this should never happen.
                    intersection(constraint_directions[j], max_speeds[j], constraint_directions[i], max_speeds[i], desired_vel);
                }
            }
        }
    }
}

/*
 * Compute the intersection of the following two lines:
 *   vec1.x * x + vec1.y * y = c1
 *   vec2.x * x + vec2.y * y = c2
 *
 * Stores the result in out. Leaves out unchanged if there
 * is no intersection.
 */
void AC_Avoid::intersection(const Vector2f vec1, const float c1, const Vector2f vec2, const float c2, Vector2f& out)
{
    float det = vec1 % vec2;
    if (!is_zero(det)) {
        out.x = (c1 * vec2.y - c2 * vec1.y) / det;
        out.y = (c2 * vec1.x - c1 * vec2.x) / det;
    }
}

/*
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance.
 *
 * Return true iff the velocity was actually changed.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
bool AC_Avoid::limit_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel, const Vector2f limit_direction, const float max_speed) const
{
    // project onto limit direction
    const float speed = desired_vel * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel += limit_direction*(max_speed - speed);
        return true;
    } else {
        return false;
    }
}

/*
 * Gets the current xy-position, relative to home (not relative to EKF origin)
 */
Vector2f AC_Avoid::get_position()
{
    const Vector3f position_xyz = _inav.get_position();
    const Vector2f position_xy(position_xyz.x,position_xyz.y);
    const Vector2f diff = location_diff(_inav.get_origin(),_ahrs.get_home()) * 100.0f;
    return position_xy - diff;
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(const float kP, const float accel_cmss, const float distance) const
{
    return AC_AttitudeControl::sqrt_controller(distance, kP, accel_cmss);
}

/*
 * Computes distance required to stop, given current speed.
 *
 * Implementation copied from AC_PosControl.
 */
float AC_Avoid::get_stopping_distance(const float kP, const float accel_cmss, const float speed) const
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || accel_cmss <= 0.0f || is_zero(speed)) {
        return 0.0f;
    }

    // calculate point at which velocity switches from linear to sqrt
    float linear_speed = accel_cmss/kP;

    // calculate distance within which we can stop
    if (speed < linear_speed) {
        return speed/kP;
    } else {
        float linear_distance = accel_cmss/(2.0f*kP*kP);
        return linear_distance + (speed*speed)/(2.0f*accel_cmss);
    }
}
