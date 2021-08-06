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
#define CONTROL_TIME_CONSTANT_RATIO 4.0   // time constant to ensure stable kinematic path generation

// update_vel_accel - single axis projection of velocity, vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero
void update_vel_accel(float& vel, float accel, float dt, float limit)
{
    const float delta_vel = accel * dt;
    if (!is_positive(delta_vel * limit)){
        vel += delta_vel;
    }
}

// update_pos_vel_accel - single axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero
void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit)
{
    // move position and velocity forward by dt if it does not increase error when limited.
    float delta_pos = vel * dt + accel * 0.5f * sq(dt);
    if (!is_positive(delta_pos * limit)){
        pos += delta_pos;
    }

    update_vel_accel(vel, accel, dt, limit);
}

// update_vel_accel - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, Vector2f limit)
{
    // increase velocity by acceleration * dt if it does not increase error when limited.
    Vector2f delta_vel = accel * dt;
    if (!is_zero(limit.length_squared())) {
        // zero delta_vel if it will increase the velocity error
        limit.normalize();
        if (is_positive(delta_vel * limit)) {
            delta_vel.zero();
        }
    }
    vel += delta_vel;
}

// update_pos_vel_accel - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, Vector2f limit)
{
    // move position and velocity forward by dt.
    Vector2f delta_pos = vel * dt + accel * 0.5f * sq(dt);

    if (!is_zero(limit.length_squared())) {
        // zero delta_vel if it will increase the velocity error
        limit.normalize();
        if (is_positive(delta_pos * limit)) {
            delta_pos.zero();
        }
    }

    pos += delta_pos.topostype();

    update_vel_accel_xy(vel, accel, dt, limit);
}

/* shape_accel calculates a jerk limited path from the current acceleration to an input acceleration.
 The function takes the current acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     acceleration limits - accel_min, accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to accel_input
*/
void shape_accel(float accel_input, float& accel,
                 float jerk_max, float dt)
{
    // jerk limit acceleration change
    float accel_delta = accel_input - accel;
    if (is_positive(jerk_max)) {
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
    }
    accel += accel_delta;
}

// 2D version
void shape_accel_xy(const Vector2f& accel_input, Vector2f& accel,
                    float jerk_max, float dt)
{
    // jerk limit acceleration change
    Vector2f accel_delta = accel_input - accel;
    if (is_positive(jerk_max)) {
        accel_delta.limit_length(jerk_max * dt);
    }
    accel = accel + accel_delta;
}

void shape_accel_xy(const Vector3f& accel_input, Vector3f& accel,
                    float jerk_max, float dt)
{
    const Vector2f accel_input_2f {accel_input.x, accel_input.y};
    Vector2f accel_2f {accel.x, accel.y};

    shape_accel_xy(accel_input_2f, accel_2f, jerk_max, dt);
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}


/* shape_vel_accel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to vel_input and accel_input
 The accel_max limit can be removed by setting it to zero.
*/
void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max
    if (!(is_negative(accel_min) && is_positive(accel_max))) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPa = jerk_max / accel_max;

    // velocity error to be corrected
    float vel_error = vel_input - vel;

    // acceleration to correct velocity
    float accel_target = vel_error * KPa;

    // constrain correction acceleration from accel_min to accel_max
    accel_target = constrain_float(accel_target, accel_min, accel_max);

    // velocity correction with input velocity
    accel_target += accel_input;

    // constrain total acceleration from accel_min to accel_max
    if (limit_total_accel) {
        accel_target = constrain_float(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt);
}

// 2D version
void shape_vel_accel_xy(const Vector2f &vel_input1, const Vector2f& accel_input,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max
    if (!is_positive(accel_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    Vector2f vel_input = vel_input1;

    // Calculate time constants and limits to ensure stable operation
    const float KPa = jerk_max / accel_max;

    // velocity error to be corrected
    const Vector2f vel_error = vel_input - vel;

    // acceleration to correct velocity
    Vector2f accel_target = vel_error * KPa;

    // limit correction acceleration to accel_max
    accel_target.limit_length(accel_max);

    accel_target += accel_input;

    // limit total acceleration to accel_max
    if (limit_total_accel) {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

/* shape_pos_vel_accel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to pos_input, vel_input and accel_input
 The vel_max, vel_correction_max, and accel_max limits can be removed by setting the desired limit to zero.
*/
void shape_pos_vel_accel(postype_t pos_input, float vel_input, float accel_input,
                         postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max
    if (!(is_negative(accel_min) && is_positive(accel_max))) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = jerk_max / (CONTROL_TIME_CONSTANT_RATIO * MAX(-accel_min, accel_max));
    const float accel_tc_max = MIN(-accel_min, accel_max) * (1.0 - 1.0 / CONTROL_TIME_CONSTANT_RATIO);

    // position error to be corrected
    float pos_error = pos_input - pos;

    // velocity to correct position
    float vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity to vel_max
    if (is_negative(vel_min) && is_positive(vel_max)){
        vel_target = constrain_float(vel_target, vel_min, vel_max);
    }

    // velocity correction with input velocity
    vel_target += vel_input;

    shape_vel_accel(vel_target, accel_input, vel, accel, accel_min, accel_max, jerk_max, dt, limit_total_accel);
}

// 2D version
void shape_pos_vel_accel_xy(const Vector2p& pos_input, const Vector2f& vel_input, const Vector2f& accel_input,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total_accel)
{
    // sanity check accel_max
    if (!is_positive(accel_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = jerk_max / (CONTROL_TIME_CONSTANT_RATIO * accel_max);
    const float accel_tc_max = accel_max * (1.0 - 1.0 / CONTROL_TIME_CONSTANT_RATIO);

    // position error to be corrected
    Vector2f pos_error = (pos_input - pos).tofloat();

    // velocity to correct position
    Vector2f vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity to vel_max
    if (is_negative(vel_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
    } else if (is_positive(vel_max)) {
        vel_target.limit_length(vel_max);
    }

    // velocity correction with input velocity
    vel_target = vel_target + vel_input;

    shape_vel_accel_xy(vel_target, accel_input, vel, accel, accel_max, jerk_max, dt, limit_total_accel);
}

// proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Both the P and second order limit have been defined.
        const float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)));
        } else {
            correction_rate = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

// proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt)
{
    const float error_length = error.length();
    if (!is_positive(error_length)) {
        return Vector2f{};
    }

    const float correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

// inverse of the sqrt controller.  calculates the input (aka error) to the sqrt_controller required to achieve a given output
float inv_sqrt_controller(float output, float p, float D_max)
{
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0 * D_max);
    }
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    const float linear_velocity = D_max / p;

    if (fabsf(output) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return output / p;
    }

    const float linear_dist = D_max / sq(p);
    const float stopping_dist = (linear_dist * 0.5f) + sq(output) / (2.0 * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

// calculate the stopping distance for the square root controller based deceleration path
float stopping_distance(float velocity, float p, float accel_max)
{
    return inv_sqrt_controller(velocity, p, accel_max);
}

// calculate the maximum acceleration or velocity in a given direction
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg)
{
    if (is_zero(direction.length_squared()) || is_zero(max_xy) || is_zero(max_z_pos) || is_zero(max_z_neg)) {
        return 0.0;
    }

    max_xy = fabsf(max_xy);
    max_z_pos = fabsf(max_z_pos);
    max_z_neg = fabsf(max_z_neg);

    direction.normalize();
    const float xy_length = Vector2f{direction.x, direction.y}.length();

    if (is_zero(xy_length)) {
        return is_positive(direction.z) ? max_z_pos : max_z_neg;
    }

    if (is_zero(direction.z)) {
        return max_xy;
    }

    const float slope = direction.z/xy_length;
    if (is_positive(slope)) {
        if (fabsf(slope) < max_z_pos/max_xy) {
            return max_xy/xy_length;
        }
        return fabsf(max_z_pos/direction.z);
    }

    if (fabsf(slope) < max_z_neg/max_xy) {
        return max_xy/xy_length;
    }
    return fabsf(max_z_neg/direction.z);
}
