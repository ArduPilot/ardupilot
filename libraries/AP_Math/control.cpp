/*
 * control.cpp
 * Copyright (C) Leonard Hall 2020
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  this module provides common controller functions
 */
#include "AP_Math.h"
#include "vector2.h"
#include "vector3.h"
#include <AP_InternalError/AP_InternalError.h>

// control default definitions
#define CORNER_ACCELERATION_RATIO   1.0/safe_sqrt(2.0)   // acceleration reduction to enable zero overshoot corners

// Projects velocity forward in time using acceleration, constrained by directional limit.
// - If `limit` is non-zero, it defines a direction in which acceleration is constrained.
// - The `vel_error` value defines the direction of velocity error (its sign matters, not its magnitude).
// - When `limit` is active, velocity is only updated if doing so would not increase the error in the limited direction.
// - If velocity is currently opposing the limit direction, the update is clipped to avoid crossing zero.
// This prevents unwanted acceleration in the direction of a constraint when it would worsen velocity error.
void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error)
{
    float delta_vel = accel * dt;
    // do not add delta_vel if it will increase the velocity error in the direction of limit
    // unless adding delta_vel will reduce vel towards zero
    if (is_positive(delta_vel * limit) && is_positive(vel_error * limit)) {
        if (is_negative(vel * limit)) {
            delta_vel = constrain_float(delta_vel, -fabsf(vel), fabsf(vel));
        } else {
            delta_vel = 0.0;
        }
    }
    vel += delta_vel;
}

// Projects position and velocity forward in time using acceleration, constrained by directional limit.
// - `limit` defines the constrained direction of motion.
// - `pos_error` and `vel_error` define the sign of error in position and velocity respectively (magnitude is ignored).
// - If the update would increase position error in the constrained direction, the position update is skipped.
// - The velocity update then proceeds with directional limit handling via `update_vel_accel()`.
// This prevents motion in a constrained direction if it would worsen the position or velocity error.
void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit, float pos_error, float vel_error)
{
    // move position and velocity forward by dt if it does not increase error when limited.
    float delta_pos = vel * dt + accel * 0.5f * sq(dt);
    // do not add delta_pos if it will increase the velocity error in the direction of limit
    if (is_positive(delta_pos * limit) && is_positive(pos_error * limit)) {
        delta_pos = 0.0;
    }
    pos += delta_pos;

    update_vel_accel(vel, accel, dt, limit, vel_error);
}

// Projects velocity forward in time using acceleration, constrained by directional limits.
// - If the `limit` vector is non-zero, it defines a direction in which acceleration is constrained.
// - The `vel_error` vector defines the direction of velocity error (its magnitude is unused).
// - When `limit` is active, velocity is only updated if doing so would not increase the error in the limited direction.
// This function prevents the system from increasing velocity along the limit direction
// if doing so would worsen the velocity error.
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& vel_error)
{
    // increase velocity by acceleration * dt if it does not increase error when limited.
    // unless adding delta_vel will reduce the magnitude of vel
    Vector2f delta_vel = accel * dt;
    if (!limit.is_zero() && !delta_vel.is_zero()) {
        // check if delta_vel will increase the velocity error in the direction of limit
        if (is_positive(delta_vel * limit) && is_positive(vel_error * limit) && !is_negative(vel * limit)) {
            delta_vel.zero();
        }
    }
    vel += delta_vel;
}

// Projects position and velocity forward in time using acceleration, constrained by directional limits.
// - The `limit` vector defines a directional constraint: if non-zero, motion in that direction is restricted.
// - The `pos_error` and `vel_error` vectors represent the direction of error (magnitude is not used).
// - If a motion step would increase error along a limited axis, it is suppressed.
// This function avoids changes to position or velocity in the direction of `limit`
// if those changes would worsen the position or velocity error respectively.
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& pos_error, const Vector2f& vel_error)
{
    // move position and velocity forward by dt.
    Vector2f delta_pos = vel * dt + accel * 0.5f * sq(dt);

    if (!is_zero(limit.length_squared())) {
        // zero delta_pos if it will increase the velocity error in the direction of limit
        if (is_positive(delta_pos * limit) && is_positive(pos_error * limit)) {
            delta_pos.zero();
        }
    }

    pos += delta_pos.topostype();

    update_vel_accel_xy(vel, accel, dt, limit, vel_error);
}

// Applies jerk-limited shaping to the acceleration value to gradually approach a new target.
// - Constrains the rate of change of acceleration to be within ±`jerk_max` over time `dt`.
// - The current acceleration value is modified in-place.
// Useful for ensuring smooth transitions in thrust or lean angle command profiles.
void shape_accel(float accel_desired, float& accel,
                 float jerk_max, float dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        float accel_delta = accel_desired - accel;
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
        accel += accel_delta;
    }
}

// Applies jerk-limited shaping to a 2D acceleration vector.
// - Constrains the rate of change of acceleration to a maximum of `jerk_max` over time `dt`.
// - The current acceleration vector is modified in-place to approach `accel_desired`.
/// Ensures smooth acceleration transitions in both axes simultaneously.
void shape_accel_xy(const Vector2f& accel_desired, Vector2f& accel,
                    float jerk_max, float dt)
{
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        Vector2f accel_delta = accel_desired - accel;
        accel_delta.limit_length(jerk_max * dt);
        accel = accel + accel_delta;
    }
}

void shape_accel_xy(const Vector3f& accel_desired, Vector3f& accel,
                    float jerk_max, float dt)
{
    const Vector2f accel_desired_2f {accel_desired.x, accel_desired.y};
    Vector2f accel_2f {accel.x, accel.y};

    shape_accel_xy(accel_desired_2f, accel_2f, jerk_max, dt);
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

// Shapes velocity and acceleration using jerk-limited control.
// - Computes correction acceleration needed to reach `vel_desired` from current `vel`.
// - Uses a square-root controller with max acceleration and jerk constraints.
// - Correction is combined with feedforward `accel_desired`.
// - If `limit_total_accel` is true, total acceleration is constrained to `accel_min` / `accel_max`.
// The result is applied via `shape_accel`.
void shape_vel_accel(float vel_desired, float accel_desired,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_min, accel_max and jerk_max.
    if (!is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // velocity error to be corrected
    float vel_error = vel_desired - vel;

    // Calculate time constants and limits to ensure stable operation
    // The direction of acceleration limit is the same as the velocity error.
    // This is because the velocity error is negative when slowing down while
    // closing a positive position error.
    float KPa;
    if (is_positive(vel_error)) {
        KPa = jerk_max / accel_max;
    } else {
        KPa = jerk_max / (-accel_min);
    }

    // acceleration to correct velocity
    float accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    // constrain correction acceleration from accel_min to accel_max
    accel_target = constrain_float(accel_target, accel_min, accel_max);

    // velocity correction with input velocity
    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled
    if (limit_total_accel) {
        accel_target = constrain_float(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt);
}

// Computes a jerk-limited acceleration command in 2D to track a desired velocity input.
// - Uses a square-root controller to calculate correction acceleration based on velocity error.
// - Correction is constrained to stay within `accel_max` (total acceleration magnitude).
// - Correction is added to `accel_desired` (feedforward).
// - If `limit_total_accel` is true, total acceleration is constrained after summing.
// Ensures velocity tracking with smooth, physically constrained motion.
void shape_vel_accel_xy(const Vector2f& vel_desired, const Vector2f& accel_desired,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max and jerk_max.
    if (!is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPa = jerk_max / accel_max;

    // velocity error to be corrected
    const Vector2f vel_error = vel_desired - vel;

    // acceleration to correct velocity
    Vector2f accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    limit_accel_corner_xy(vel_desired, accel_target, accel_max);

    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled 
    if (limit_total_accel) {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

// Shapes position, velocity, and acceleration using a jerk-limited profile.
// - Computes velocity to close position error using a square-root controller.
// - That velocity is then shaped via `shape_vel_accel` to enforce acceleration and jerk limits.
// - Limits can be applied separately to correction and total values.
// Used for smooth point-to-point motion with constrained dynamics.
void shape_pos_vel_accel(postype_t pos_desired, float vel_desired, float accel_desired,
                         postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total)
{
    // sanity check vel_min, vel_max, accel_min, accel_max and jerk_max.
    if (is_positive(vel_min) || is_negative(vel_max) || !is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // position error to be corrected
    float pos_error = pos_desired - pos;

    // Calculate time constants and limits to ensure stable operation
    // The negative acceleration limit is used here because the square root controller
    // manages the approach to the setpoint. Therefore the acceleration is in the opposite
    // direction to the position error.
    float accel_tc_max;
    float KPv;
    if (is_positive(pos_error)) {
        accel_tc_max = -0.5 * accel_min;
        KPv = 0.5 * jerk_max / (-accel_min);
    } else {
        accel_tc_max = 0.5 * accel_max;
        KPv = 0.5 * jerk_max / accel_max;
    }

    // velocity to correct position
    float vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity between vel_min and vel_max
    if (is_negative(vel_min) || is_positive(vel_max)) {
        vel_target = constrain_float(vel_target, vel_min, vel_max);
    }

    // velocity correction with input velocity
    vel_target += vel_desired;

    // Constrain total velocity if limiting is enabled and the velocity range is valid (non-zero and min < max)
    if (limit_total && (vel_max > vel_min)) {
        vel_target = constrain_float(vel_target, vel_min, vel_max);
    }

    shape_vel_accel(vel_target, accel_desired, vel, accel, accel_min, accel_max, jerk_max, dt, limit_total);
}

// Computes a jerk-limited acceleration profile to move toward a position and velocity target in 2D.
// - Computes a velocity correction based on position error using a square-root controller.
// - That velocity is passed to `shape_vel_accel_xy` to generate a constrained acceleration.
// - Limits include: maximum velocity (`vel_max`), maximum acceleration (`accel_max`), and maximum jerk (`jerk_max`).
// - If `limit_total` is true, constraints are applied to the total command (not just the correction).
// Provides smooth trajectory shaping for lateral motion with bounded dynamics.
void shape_pos_vel_accel_xy(const Vector2p& pos_desired, const Vector2f& vel_desired, const Vector2f& accel_desired,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total)
{
    // sanity check vel_max, accel_max and jerk_max.
    if (is_negative(vel_max) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = 0.5 * jerk_max / accel_max;
    // Time constant for velocity shaping near the target
    const float accel_tc_max = 0.5 * accel_max;

    // Position error to be corrected — direction is preserved, magnitude used for shaping
    Vector2f pos_error = (pos_desired - pos).tofloat();

    // velocity to correct position
    Vector2f vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity to vel_max
    if (is_positive(vel_max)) {
        vel_target.limit_length(vel_max);
    }

    // velocity correction with input velocity
    vel_target = vel_target + vel_desired;
    
    // Constrain total velocity if limiting is enabled and vel_max is positive 
    if (limit_total && is_positive(vel_max)) {
        vel_target.limit_length(vel_max);
    }

    shape_vel_accel_xy(vel_target, accel_desired, vel, accel, accel_max, jerk_max, dt, limit_total);
}

// Computes a jerk-limited acceleration command to follow an angular position, velocity, and acceleration target.
// - This function applies jerk-limited shaping to angular acceleration, based on input angle, angular velocity, and angular acceleration.
// - Internally computes a target angular velocity using a square-root controller on the angle error.
// - Velocity and acceleration are both optionally constrained:
//   - If `limit_total` is true, limits apply to the total (not just correction) command.
//   - Setting `angle_vel_max` or `angle_accel_max` to zero disables that respective limit.
// - The acceleration output is shaped toward the target using `shape_vel_accel`.
// Used for attitude control with limited angular velocity and angular acceleration (e.g., roll/pitch shaping).         
void shape_angle_vel_accel(float angle_desired, float angle_vel_desired, float angle_accel_desired,
                         float angle, float angle_vel, float& angle_accel,
                         float angle_vel_max, float angle_accel_max,
                         float angle_jerk_max, float dt, bool limit_total)
{
    // sanity check accel_max
    if (!is_positive(angle_accel_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Estimate time to decelerate based on current angular velocity and acceleration limit
    float stopping_time = fabsf(angle_vel / angle_accel_max);

    // Compute total angular error with prediction of future motion, then wrap to [-π, π]
    float angle_error = angle_desired - angle - angle_vel * stopping_time;
    angle_error = wrap_PI(angle_error);
    angle_error += angle_vel * stopping_time;

    // Calculate time constants and limits to ensure stable operation
    // These ensure the square-root controller respects angular acceleration and jerk constraints
    const float angle_accel_tc_max = 0.5 * angle_accel_max;
    const float KPv = 0.5 * angle_jerk_max / angle_accel_max;

    // velocity to correct position
    float angle_vel_target = sqrt_controller(angle_error, KPv, angle_accel_tc_max, dt);

    // limit velocity to vel_max
    if (is_positive(angle_vel_max)){
        angle_vel_target = constrain_float(angle_vel_target, -angle_vel_max, angle_vel_max);
    }

    // velocity correction with input velocity
    angle_vel_target += angle_vel_desired;

    // Constrain total velocity if limiting is enabled and angle_vel_max is positive 
    if (limit_total && is_positive(angle_vel_max)){
        angle_vel_target = constrain_float(angle_vel_target, -angle_vel_max, angle_vel_max);
    }

    // Shape the angular acceleration using jerk-limited profile
    shape_vel_accel(angle_vel_target, angle_accel_desired, angle_vel, angle_accel, 
                    -angle_accel_max, angle_accel_max, angle_jerk_max, dt, limit_total);
}

// Limits a 2D acceleration vector to prioritize lateral (cross-track) acceleration over longitudinal (in-track) acceleration.
// - `vel` defines the current direction of motion (used to split the acceleration).
// - `accel` is modified in-place to remain within `accel_max`.
// - If the full acceleration vector exceeds `accel_max`, it is reshaped to prioritize lateral correction.
// - If `vel` is zero, a simple magnitude limit is applied.
// Returns true if the acceleration vector was modified.
bool limit_accel_xy(const Vector2f& vel, Vector2f& accel, float accel_max)
{
    // check accel_max is defined
    if (!is_positive(accel_max)) {
        return false;
    }
    // limit acceleration to accel_max while prioritizing cross track acceleration
    if (accel.length_squared() > sq(accel_max)) {
        if (vel.is_zero()) {
            // We do not have a direction of travel so do a simple vector length limit
            accel.limit_length(accel_max);
        } else {
            // calculate acceleration in the direction of and perpendicular to the velocity input
            const Vector2f vel_unit = vel.normalized();
            // acceleration in the direction of travel
            float accel_dir = vel_unit * accel;
            // cross track acceleration
            Vector2f accel_cross = accel - (vel_unit * accel_dir);
            if (accel_cross.limit_length(accel_max)) {
                accel_dir = 0.0;
            } else {
                // limit_length can't absolutely guarantee this subtraction
                // won't be slightly negative, so safe_sqrt is used
                float accel_max_dir = safe_sqrt(sq(accel_max) - accel_cross.length_squared());
                accel_dir = constrain_float(accel_dir, -accel_max_dir, accel_max_dir);
            }
            accel = accel_cross + vel_unit * accel_dir;
        }
        return true;
    }
    return false;
}

// Limits acceleration magnitude while preserving cross-track authority during turns.
// - Splits acceleration into components parallel and perpendicular to velocity.
// - Ensures sufficient lateral acceleration is reserved for path curvature.
// - If forward acceleration dominates, lateral acceleration is limited to
//   CORNER_ACCELERATION_RATIO * accel_max.
// Returns true if limiting was applied. (including simple magnitude limiting when vel is zero).
bool limit_accel_corner_xy(const Vector2f& vel, Vector2f& accel, float accel_max)
{
    // check accel_max is defined
    if (!is_positive(accel_max)) {
        return false;
    }
    if (vel.is_zero()) {
        // No along/cross decomposition possible; apply a simple magnitude limit.
        return accel.limit_length(accel_max);
    }

    // Unit velocity direction defines the along-track axis.
    const Vector2f vel_unit = vel.normalized();

    // Decompose accel into along-track and cross-track components.
    float accel_dir = vel_unit * accel;
    Vector2f accel_cross =  accel - (vel_unit * accel_dir);

    // If cross-track magnitude is at least as large as along-track, a simple vector limit
    // preserves at least 1/sqrt(2) of accel_max for cross-track correction.
    if (sq(accel_dir) <= accel_cross.length_squared()) {
        // accel can be simply limited in magnitude
        return accel.limit_length(accel_max);
    }

    // Along-track dominates: preserve cross-track authority by limiting cross-track first,
    // then allocate remaining magnitude to along-track.
    accel_cross.limit_length(CORNER_ACCELERATION_RATIO * accel_max);
    float accel_max_dir = safe_sqrt(sq(accel_max) - accel_cross.length_squared());
    accel_dir = constrain_float(accel_dir, -accel_max_dir, accel_max_dir);
    accel = accel_cross + vel_unit * accel_dir;
    return true;
}

// Piecewise square-root + linear controller that limits second-order response (acceleration).
// - Behaves like a P controller near the setpoint.
// - Switches to sqrt(2·a·Δx) shaping beyond a threshold to limit acceleration.
// - `second_ord_lim` sets the max acceleration allowed.
// - Returns the constrained correction rate for a given error and gain.
float sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // No second-order limit: use pure linear controller
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // No P gain, but with acceleration limit — use sqrt-shaped response only
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Both P and second-order limits defined — use hybrid model
        const float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            // Positive error beyond linear region — use sqrt branch
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
        } else if (error < -linear_dist) {
            // Negative error beyond linear region — use sqrt branch
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)));
        } else {
            // Inside linear region
            correction_rate = error * p;
        }
    }
    if (is_positive(dt)) {
        // Clamp to ensure we do not overshoot the error in the last time step
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

// Vector form of `sqrt_controller()`, applied along the direction of the input error vector.
// - Returns a correction vector with magnitude shaped using `sqrt_controller()`.
// - Direction is preserved from the input error.
// - Used in 2D position or velocity control with second-order constraints.
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt)
{
    const float error_length = error.length();
    if (!is_positive(error_length)) {
        return Vector2f{};
    }

    const float correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

// Inverts the output of `sqrt_controller()` to recover the input error that would produce a given output.
// - Useful for calculating required error to produce a desired rate.
// - Handles both linear and square-root regions of the controller response.
float inv_sqrt_controller(float output, float p, float D_max)
{
    // Degenerate case: second-order limit (D_max) is positive, but P gain is zero
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0 * D_max);
    }

    // Degenerate case: no D_max, but P gain is non-zero → use linear model
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }

    // Degenerate case: both gains are zero — no useful model
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0;
    }

    // Compute transition threshold between linear and sqrt regions
    const float linear_velocity = D_max / p;

    if (fabsf(output) < linear_velocity) {
        // Linear region: below transition threshold
        return output / p;
    }

    // Square-root region: above transition threshold
    const float linear_dist = D_max / sq(p);
    const float stopping_dist = (linear_dist * 0.5f) + sq(output) / (2.0 * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

// Calculates stopping distance required to reduce a velocity to zero using a square-root controller.
// - Uses the inverse of the `sqrt_controller()` response curve.
// - Inputs: velocity, P gain, and max deceleration (`accel_max`)
// - Output: stopping distance required to decelerate cleanly.
float stopping_distance(float velocity, float p, float accel_max)
{
    // Use inverse of sqrt_controller to compute stopping distance from current velocity
    return inv_sqrt_controller(velocity, p, accel_max);
}

// Computes the maximum possible acceleration or velocity in a specified 3D direction,
// constrained by separate limits in horizontal (XY) and vertical (Z) axes.
// - `direction` should be a non-zero vector indicating desired direction of travel.
// - Limits: max_xy, max_z_pos (upward), max_z_neg (downward)
// Returns the maximum achievable magnitude in that direction without violating any axis constraint.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_neg, float max_z_pos)
{
    // Reject zero-length direction vectors or undefined limits
    if (is_zero(direction.length_squared())) {
        return 0.0;
    }

    const float segment_length_xy = direction.xy().length();
    
    return kinematic_limit(segment_length_xy, direction.z, max_xy, max_z_neg, max_z_pos);
}

// compute the maximum allowed magnitude along a direction defined by segment_length_xy and segment_length_z components
// constrained by independent horizontal (max_xy) and vertical (max_z_pos/max_z_neg) limits
// returns the maximum achievable magnitude without exceeding any axis limit
float kinematic_limit(float segment_length_xy, float segment_length_z, float max_xy, float max_z_neg, float max_z_pos)
{
    // Reject zero-length direction vectors or undefined limits
    if (is_zero(max_xy) || is_zero(max_z_pos) || is_zero(max_z_neg)) {
        return 0.0;
    }

    max_xy = fabsf(max_xy);
    max_z_pos = fabsf(max_z_pos);
    max_z_neg = fabsf(max_z_neg);

    const float length = safe_sqrt(sq(segment_length_xy) + sq(segment_length_z));
    // check for divide by zero.
    if (!is_positive(length)) {
        return 0.0;
    }
    segment_length_xy /= length;
    segment_length_z /= length;

    if (is_zero(segment_length_xy)) {
        // Pure vertical motion
        return is_positive(segment_length_z) ? max_z_pos : max_z_neg;
    }

    if (is_zero(segment_length_z)) {
        // Pure horizontal motion
        return max_xy;
    }

    // Compute vertical-to-horizontal slope of desired direction
    const float slope = segment_length_z/segment_length_xy;
    if (is_positive(slope)) {
        // Ascending: check if slope is within limits
        if (fabsf(slope) < max_z_pos/max_xy) {
            return max_xy/segment_length_xy;
        }
        // Vertical limit dominates in upward direction
        return fabsf(max_z_pos/segment_length_z);
    }

    // Descending: check if slope is within limits
    if (fabsf(slope) < max_z_neg/max_xy) {
        return max_xy/segment_length_xy;
    }

    // Vertical limit dominates in downward direction
    return fabsf(max_z_neg/segment_length_z);
}

// Applies an exponential curve to a normalized input in the range [-1, 1].
// - `expo` shapes the curve (0 = linear, closer to 1 = more curvature).
// - Typically used for pilot stick input response shaping.
// - Clipped to `expo < 0.95` to avoid divide-by-zero or extreme scaling.
float input_expo(float input, float expo)
{
    // Clamp input to normalized stick range
    input = constrain_float(input, -1.0, 1.0);
    if (expo < 0.95) {
        // Expo shaping: increases control around center stick
        return (1 - expo) * input / (1 - expo * fabsf(input));
    }
    // If expo is too close to 1, return input unchanged
    return input;
}

// Converts a lean angle (radians) to horizontal acceleration in m/s².
float angle_rad_to_accel_mss(float angle_rad)
{
    // Convert lean angle to horizontal acceleration
    return GRAVITY_MSS * tanf(angle_rad);
}

// Converts a lean angle (degrees) to horizontal acceleration in m/s².
float angle_deg_to_accel_mss(float angle_deg)
{
    // Convert degrees to radians, then to acceleration
    return angle_rad_to_accel_mss(radians(angle_deg));
}

// Converts a horizontal acceleration (m/s²) to lean angle in radians.
// Assumes: angle = atan(a / g)
float accel_mss_to_angle_rad(float accel_mss)
{
    // Inverse of angle_rad_to_accel_mss
    return atanf(accel_mss/GRAVITY_MSS);
}

// Converts a horizontal acceleration (m/s²) to lean angle in degrees.
float accel_mss_to_angle_deg(float accel_mss)
{
    // Convert result of radian-based conversion to degrees
    return degrees(accel_mss_to_angle_rad(accel_mss));
}

// Converts pilot’s normalized roll/pitch input into target roll and pitch angles (radians).
// - `roll_in_norm` and `pitch_in_norm`: stick inputs in range [-1, 1]
// - `angle_max_rad`: maximum allowed lean angle
// - `angle_limit_rad`: secondary limit to constrain output while preserving full stick range
// Outputs are Euler angles in radians: `roll_out_rad`, `pitch_out_rad`
void rc_input_to_roll_pitch_rad(float roll_in_norm, float pitch_in_norm, float angle_max_rad, float angle_limit_rad, float &roll_out_rad, float &pitch_out_rad)
{
    // Constrain angle_max to 85 deg to avoid unstable behavior
    angle_max_rad = MIN(angle_max_rad, radians(85.0));

    // Convert normalized pitch and roll stick input into horizontal thrust components
    Vector2f thrust;
    thrust.x = - tanf(angle_max_rad * pitch_in_norm);
    thrust.y = tanf(angle_max_rad * roll_in_norm);

    // Calculate the horizontal thrust limit based on angle limit
    angle_limit_rad = constrain_float(angle_limit_rad, radians(10.0), angle_max_rad);
    float thrust_limit = tanf(angle_limit_rad);

    // Apply limit to the horizontal thrust vector (preserves stick direction)
    thrust.limit_length(thrust_limit);

    // Convert thrust vector back to pitch and roll Euler angles
    pitch_out_rad = - atanf(thrust.x);
    roll_out_rad = atanf(cosf(pitch_out_rad) * thrust.y);
}
