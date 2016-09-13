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

/*
 * Adjusts the desired velocity so that the vehicle can stop
 * before the fence/object.
 */
void AC_Avoid::adjust_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    if (_enabled == AC_AVOID_STOP_AT_FENCE) {

        // get polygon boundary
        // Note: first point in list is the return-point (which copter does not use)
        uint16_t num_points_poly;
        Vector2f* boundary = _fence.get_polygon_points(num_points_poly);
        uint16_t num_points_reg_poly = get_num_points_reg_poly();

        // Allocate an array to store all constraints
        // Note: as new types of constraints are added (e.g. obstacles), this
        // needs to be updated.
        uint16_t num_constraints = 0;
        if (can_adjust_reg_poly()) {
            num_constraints += num_points_reg_poly;
        }
        if (can_adjust_poly()) {
            // One of the points in the boundary is not part of the
            // fence, so we don't need a constraint for that.
            num_constraints += num_points_poly - 1;
        }
        if (num_constraints == 0) {
            return;
        }
        // Two arrays giving constraints induced by the current position and fence
        // The ith constraint is:
        //    velocity * constraints[i].A <= constraints[i].c
        constraint constraints[num_constraints];
        // an index into the next free slot in constraints
        uint16_t free_index = 0;
        if (can_adjust_reg_poly()) {
            velocity_constraints_reg_poly(kP, accel_cmss_limited, num_points_reg_poly, get_radius(),
                                          constraints, free_index);
        }
        if (can_adjust_poly()) {
            velocity_constraints_poly(kP, accel_cmss_limited, boundary, num_points_poly,
                                      constraints, free_index);
        }

        // Adjust the velocity according to all constraints.
        // The number of constraints is equal to free_index and not necessarily
        // num_constraints. This is because we may not have filled the entire constraint
        // array. This can happen, for example, if the polygon fence has already
        // been breached.
        adjust_velocity_constraints(kP, accel_cmss, constraints, free_index, desired_vel);
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
 * Returns true if the regular polygon fence (centered at home)
 * adjustment should be run.
 */
bool AC_Avoid::can_adjust_reg_poly()
{
    return ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) &&
           ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) == 0);
}

/*
 * Returns true if the polygon fence adjustment should be run.
 */
bool AC_Avoid::can_adjust_poly()
{
    return ((_fence.get_enabled_fences() & AC_FENCE_TYPE_POLYGON) != 0) &&
           ((_fence.get_breaches() & AC_FENCE_TYPE_POLYGON) == 0);
}

/*
 * Computes velocity constraints for the regular polygon fence centered at home.
 *
 * In particular, for each edge:
 * Let P be the closest point to the current position of the vehicle.
 * Let U be the unit vector in the direction of P from the current position.
 * Let D be the distance from the current position to P.
 * safe_vel must satisfy
 *
 *       safe_vel * U <= max_speed(D)
 */
void AC_Avoid::velocity_constraints_reg_poly(const float kP, const float accel_cmss, const uint16_t num_points,
        const float radius, constraint* constraints, uint16_t& free_index)
{
    if (num_points < 3) {
        return;
    }

    // get position as a 2D offset in cm from ahrs home
    const Vector2f position_xy = get_position();

    // the angle between vertices
    const float angle_step_size = M_2PI/num_points;

    // Compute constraints for a regular polygon with num_points vertices
    // whose center is the origin and whose vertices are radius
    // distance from the origin.
    // The first vertex is on the positive x-axis.
    Vector2f start = Vector2f(radius, 0);
    uint16_t index = free_index;
    for (uint8_t i = 1; i <= num_points; i++) {
        const float angle = angle_step_size * i;
        Vector2f end = Vector2f(cosf(angle), sinf(angle)) * radius;

        // Check containment within the current edge
        // Should we move this containment check to AC_Fence?
        if ((end - start) % (position_xy - start) > 0) {
            // contained within current edge
            constraints[index] = velocity_constraint_edge(kP, accel_cmss, start, end, position_xy);
            start = end;
            index++;
        } else {
            // outside polygon - abort
            // we haven't changed free_index, so it
            // doesn't matter what values have been written to constraints
            return;
        }
    }

    free_index = index;
}

/*
 * Computes velocity constraints for the polygon fence.
 *
 * In particular, for each edge:
 * Let P be the closest point to the current position of the vehicle.
 * Let U be the unit vector in the direction of P from the current position.
 * Let D be the distance from the current position to P.
 * safe_vel must satisfy
 *
 *       safe_vel * U <= max_speed(D)
 */
void AC_Avoid::velocity_constraints_poly(const float kP, const float accel_cmss, const Vector2f* boundary,
        const uint16_t num_points, constraint* constraints, uint16_t& free_index)
{
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

    uint16_t i, j;
    for (i = 1, j = num_points-1; i < num_points; j = i++) {
        // end points of current edge
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        constraints[free_index] = velocity_constraint_edge(kP, accel_cmss, start, end, position_xy);;
        free_index++;
    }

}

/*
 * Computes the velocity constraint for the given edge. In particular,
 * Let P be the closest point to the current position of the vehicle.
 * Let U be the unit vector in the direction of P from the current position.
 * Let D be the distance from the current position to P.
 * The velocity must satisfy
 *
 *       velocity * U <= max_speed(D)
 */
constraint AC_Avoid::velocity_constraint_edge(const float kP, const float accel_cmss, const Vector2f start, const Vector2f end,
        const Vector2f position_xy)
{
    // vector from current position to closest point on current edge
    Vector2f limit_direction = Vector2f::closest_point(position_xy, start, end) - position_xy;
    // distance to closest point
    const float limit_distance = limit_direction.length();
    constraint res;
    if (!is_zero(limit_distance)) {
        // We are strictly inside the given edge.
        // Set constraint for this edge.
        limit_direction /= limit_distance;
        res.A = limit_direction;
        res.c = get_max_speed(kP, accel_cmss, limit_distance - get_margin());
    }

    // TODO: figure out what to do when the limit_distance is zero.
    // It's not a big deal since that means the vehicle is directly on
    // the fence boundary and should probably be considered a fence breach.
    return res;
}

/*
 * Adjusts the velocity to satisfy the given set of constraints. The ith constraint is
 *    desired_vel * constraints[i].A <= constraints[i].c
 */
void AC_Avoid::adjust_velocity_constraints(const float kP, const float accel_cmss, const constraint* constraints,
        const uint16_t num_constraints, Vector2f& desired_vel)
{
    for (uint16_t i = 0; i < num_constraints; i++) {
        if (limit_velocity(kP, accel_cmss, desired_vel, constraints[i].A, constraints[i].c)) {
            // If velocity was adjusted, iterate through all prior constraints to check for
            // violations
            for (uint16_t j = 0; j < i; j++) {
                if (desired_vel * constraints[j].A > constraints[j].c) {
                    // New velocity violates previous constraint
                    // Adjust velocity to intersection of the two constraints.
                    // This should be the closest point that satisfies both constraints.
                    // The following will do nothing if there is no intersection,
                    // but this should never happen.
                    intersection(constraints[j].A, constraints[j].c, constraints[i].A, constraints[i].c, desired_vel);
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
 * limit_direction to be at most the maximum speed.
 *
 * Return true iff the velocity was actually changed.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
bool AC_Avoid::limit_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel, const Vector2f limit_direction,
                              const float max_speed) const
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
