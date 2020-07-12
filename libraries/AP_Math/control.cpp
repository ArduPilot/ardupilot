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
#include <AP_Logger/AP_Logger.h>

// update_pos_vel_accel - single axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
void update_pos_vel_accel(float& pos, float& vel, float& accel, float dt)
{
    // move position and velocity forward by dt.
    pos = pos + vel * dt + accel * 0.5f * sq(dt);
    vel = vel + accel * dt;
}

// update_pos_vel_accel_xy - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
void update_pos_vel_accel_xy(Vector2f& pos, Vector2f& vel, Vector2f& accel, float dt)
{
    // move position and velocity forward by dt.
    pos = pos + vel * dt + accel * 0.5f * sq(dt);
    vel = vel + accel * dt;
}

// update_pos_vel_accel_xy - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// This function only updates the x and y axis leaving the z axis unchanged.
void update_pos_vel_accel_xy(Vector3f& pos, Vector3f& vel, Vector3f& accel, float dt)
{
    Vector2f pos_2f = Vector2f(pos.x, pos.y);
    Vector2f vel_2f = Vector2f(vel.x, vel.y);
    Vector2f accel_2f = Vector2f(accel.x, accel.y);
    update_pos_vel_accel_xy(pos_2f, vel_2f, accel_2f, dt);
    pos.x = pos_2f.x;
    pos.y = pos_2f.y;
    vel.x = vel_2f.x;
    vel.y = vel_2f.y;
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

/* shape_vel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
*/
void shape_vel(float& vel_target, float vel, float& accel, float vel_max, float accel_max, float tc, float dt)
{
    // sanity check tc
    if (is_positive(tc)) {
        // Calculate time constants and limits to ensure stable operation
        float KPa = 1.0 / tc;
        float jerk_max = accel_max * KPa;

        // limit velocity to vel_max
        vel_target = constrain_float(vel_target, -vel_max, vel_max);

        // velocity error to be corrected
        float vel_error = vel_target - vel;

        // acceleration to correct velocity
        float accel_target = vel_error * KPa;

        // jerk limit acceleration change
        float accel_delta = accel_target - accel;
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
        accel = accel + accel_delta;

        // limit acceleration to accel_max
        accel = constrain_float(accel, -accel_max, accel_max);

        // Calculate maximum pos_input and vel_input based on limited system
        vel_error = accel / KPa;
        vel_target = vel_error + vel;
    }
}

/* shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
*/
void shape_vel_xy(Vector2f& vel_target, const Vector2f& vel, Vector2f& accel, float vel_max, float accel_max, float tc, float dt)
{
    if (is_positive(tc)) {
        // Calculate time constants and limits to ensure stable operation
        float KPa = 1.0 / tc;
        float jerk_max = accel_max * KPa;

        // limit velocity to vel_max
        limit_vector_length(vel_target.x, vel_target.y, vel_max);

        // velocity error to be corrected
        Vector2f vel_error = vel_target - vel;

        // acceleration to correct velocity
        Vector2f accel_target = vel_error * KPa;

        // jerk limit acceleration change
        Vector2f accel_delta = accel_target - accel;
        limit_vector_length(accel_delta.x, accel_delta.y, jerk_max * dt);
        accel = accel + accel_delta;

        // limit acceleration to accel_max
        limit_vector_length(accel.x, accel.y, accel_max);

        // Calculate maximum pos_input and vel_input based on limited system
        vel_error = accel / KPa;
        vel_target = vel_error + vel;
    }
}

/* shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of Vector3f inputs.
*/
void shape_vel_xy(Vector3f& vel_input, const Vector3f& vel, Vector3f& accel, float vel_max, float accel_max, float tc, float dt)
{
    Vector2f vel_input_2f = Vector2f(vel_input.x, vel_input.y);
    Vector2f vel_2f = Vector2f(vel.x, vel.y);
    Vector2f accel_2f = Vector2f(accel.x, accel.y);

    shape_vel_xy(vel_input_2f, vel_2f, accel_2f, vel_max, accel_max, tc, dt);
    vel_input.x = vel_input_2f.x;
    vel_input.y = vel_input_2f.y;
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

/* shape_pos_vel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
*/
void shape_pos_vel(float& pos_input, float vel_input, float pos, float vel, float& accel, float vel_max, float accel_max, float tc, float dt)
{
    if (is_positive(tc)) {
        // Calculate time constants and limits to ensure stable operation
        float KPv = 1.0 / (CONTROL_TIME_CONSTANT_RATIO*tc);
        float accel_tc_max = accel_max*(1-1.0f/CONTROL_TIME_CONSTANT_RATIO);

        // position error to be corrected
        float pos_error = pos_input - pos;

        // velocity to correct position
        float vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

        // velocity correction with input velocity
        vel_target = vel_target + vel_input;

        shape_vel(vel_target, vel, accel, vel_max, accel_max, tc, dt);

        vel_target = vel_target - vel_input;
        pos_error = stopping_point(vel_target, KPv, accel_tc_max);
        pos_input = pos_error + pos;
    }
}

/* shape_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
*/
void shape_pos_vel_xy(Vector2f& pos_input, const Vector2f& vel_input, const Vector2f& pos, const Vector2f& vel, Vector2f& accel, float vel_max, float accel_max, float tc, float dt)
{
    if (is_positive(tc)) {
        // Calculate time constants and limits to ensure stable operation
        float KPv = 1.0 / (CONTROL_TIME_CONSTANT_RATIO*tc);
        float accel_tc_max = accel_max*(1-1.0f/CONTROL_TIME_CONSTANT_RATIO);

        // position error to be corrected
        Vector2f pos_error = pos_input - pos;

        // velocity to correct position
        Vector2f vel_target = sqrt_controller_xy(pos_error, KPv, accel_tc_max);

        // velocity correction with input velocity
        vel_target = vel_target + vel_input;

        shape_vel_xy(vel_target, vel, accel, vel_max, accel_max, tc, dt);

        vel_target = vel_target - vel_input;
        pos_error = stopping_point_xy(vel_target, KPv, accel_tc_max);
        pos_input = pos_error + pos;
    }

#ifndef HAL_NO_LOGGING
    AP::logger().Write("SHP",
                       "TimeUS,DPX,DPY,DVX,DVY,PX,PY,VX,VY,AX,AY",
                       "soooooooooo",
                       "F0000000000",
                       "Qffffffffff",
                       AP_HAL::micros64(),
                       double(pos_input.x * 0.01f),
                       double(pos_input.y * 0.01f),
                       double(vel_input.x * 0.01f),
                       double(vel_input.y * 0.01f),
                       double(pos.x * 0.01f),
                       double(pos.y * 0.01f),
                       double(vel.x * 0.01f),
                       double(vel.y * 0.01f),
                       double(accel.x * 0.01f),
                       double(accel.y * 0.01f));
#endif
}

/* shape_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
 This function operates on the x and y axis of Vector3f inputs.
*/
void shape_pos_vel_xy(Vector3f& pos_input, const Vector3f& vel_input, const Vector3f& pos, const Vector3f& vel, Vector3f& accel, float vel_max, float accel_max, float tc, float dt)
{
    Vector2f pos_input_2f = Vector2f(pos_input.x, pos_input.y);
    Vector2f vel_input_2f = Vector2f(vel_input.x, vel_input.y);
    Vector2f pos_2f = Vector2f(pos.x, pos.y);
    Vector2f vel_2f = Vector2f(vel.x, vel.y);
    Vector2f accel_2f = Vector2f(accel.x, accel.y);

    shape_pos_vel_xy(pos_input_2f, vel_input_2f, pos_2f, vel_2f, accel_2f, vel_max, accel_max, tc, dt);
    pos_input.x = pos_input_2f.x;
    pos_input.y = pos_input_2f.y;
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(const float& error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error - (linear_dist / 2.0f)));
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


/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f sqrt_controller_xy(const Vector2f& error, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return error * p;
    }

    float linear_dist = second_ord_lim / sq(p);
    float error_length = error.length();
    if (error_length > linear_dist) {
        float first_order_scale = safe_sqrt(2.0f * second_ord_lim * (error_length - (linear_dist * 0.5f))) / error_length;
        return error * first_order_scale;
    } else {
        return error * p;
    }
}


/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector3f sqrt_controller_xy(const Vector3f& error, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return Vector3f(error.x * p, error.y * p, error.z);
    }

    float linear_dist = second_ord_lim / sq(p);
    float error_length = norm(error.x, error.y);
    if (error_length > linear_dist) {
        float first_order_scale = safe_sqrt(2.0f * second_ord_lim * (error_length - (linear_dist * 0.5f))) / error_length;
        return Vector3f(error.x * first_order_scale, error.y * first_order_scale, error.z);
    } else {
        return Vector3f(error.x * p, error.y * p, error.z);
    }
}

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
float stopping_point(float velocity, float p, float accel_max)
{
    if (is_positive(accel_max) && is_zero(p)) {
        return (velocity * velocity) / (2.0f * accel_max);
    } else if ((is_negative(accel_max) || is_zero(accel_max)) && !is_zero(p)) {
        return velocity / p;
    } else if ((is_negative(accel_max) || is_zero(accel_max)) && is_zero(p)) {
        return 0.0f;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    float linear_velocity = accel_max / p;

    if (fabsf(velocity) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return velocity / p;
    } else {
        float linear_dist = accel_max / sq(p);
        float stopping_distance = (linear_dist * 0.5f) + sq(velocity) / (2.0f * accel_max);
        if (is_positive(velocity)) {
            return stopping_distance;
        } else {
            return -stopping_distance;
        }
    }
}

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f stopping_point_xy(Vector2f velocity, float p, float accel_max)
{
    // calculate current velocity
    float velocity_length = velocity.length();
    float stopping_distance = stopping_point(velocity_length, p, accel_max);
    return velocity.normalized() * stopping_distance;
}


/// limit vector to a given length, returns true if vector was limited
bool limit_vector_length(float& vector_x, float& vector_y, float max_length)
{
    float vector_length = norm(vector_x, vector_y);
    if ((vector_length > max_length) && is_positive(vector_length)) {
        vector_x *= (max_length / vector_length);
        vector_y *= (max_length / vector_length);
        return true;
    }
    return false;
}
