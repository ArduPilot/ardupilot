#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library

#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to avoid hitting fence

// bit masks for enabled fence types.
#define AC_AVOID_DISABLED               0       // avoidance disabled
#define AC_AVOID_STOP_AT_FENCE          1       // stop at fence

/*
 * This class prevents the vehicle from leaving a polygon fence in
 * 2 dimensions by limiting velocity (adjust_velocity).
 */
class AC_Avoid {
public:

    /// Constructor
    AC_Avoid(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AC_Fence& fence);

    /*
     * Adjusts the desired velocity so that the vehicle can stop
     * before the fence/object.
     * Note: Vector3f version is for convenience and only adjusts x and y axis
     */
    void adjust_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel);
    void adjust_velocity(const float kP, const float accel_cmss, Vector3f &desired_vel);

    static const struct AP_Param::GroupInfo var_info[];

private:

    /*
     * Adjusts the desired velocity for the circular fence.
     */
    void adjust_velocity_circle(const float kP, const float accel_cmss, Vector2f &desired_vel);

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
    void adjust_velocity_poly(const float kP, const float accel_cmss, Vector2f &desired_vel);

    /*
     * Adjusts the velocity to satisfy the given set of constraints. The ith constraint is
     *    desired_vel * constraint_directions[i] <= max_speeds[i]
     */
    void adjust_velocity_constraints(const float kP, const float accel_cmss, const Vector2f* constraint_directions, const float* max_speeds, const uint16_t num_constraints, Vector2f& desired_vel);


    /*
     * Compute the intersection of the following two lines:
     *   vec1.x * x + vec1.y * y = c1
     *   vec2.x * x + vec2.y * y = c2
     *
     * Stores the result in out. Leaves out unchanged if there
     * is no intersection.
     */
    void intersection(const Vector2f vec1, const float c1, const Vector2f vec2, const float c2, Vector2f& out);

    /*
     * Limits the component of desired_vel in the direction of the unit vector
     * limit_direction to be at most the maximum speed.
     *
     * Return true iff the velocity was actually changed.
     *
     * Uses velocity adjustment idea from Randy's second email on this thread:
     * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
     */
    bool limit_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel, const Vector2f limit_direction, const float max_speed) const;

    /*
     * Gets the current position, relative to home (not relative to EKF origin)
     */
    Vector2f get_position();

    /*
     * Computes the speed such that the stopping distance
     * of the vehicle will be exactly the input distance.
     */
    float get_max_speed(const float kP, const float accel_cmss, const float distance) const;

    /*
     * Computes distance required to stop, given current speed.
     */
    float get_stopping_distance(const float kP, const float accel_cmss, const float speed) const;

    /*
     * Gets the fence margin in cm
     */
    float get_margin() const {
        return _fence.get_margin() * 100.0f;
    }

    // external references
    const AP_AHRS& _ahrs;
    const AP_InertialNav& _inav;
    const AC_Fence& _fence;

    // parameters
    AP_Int8 _enabled;
};
