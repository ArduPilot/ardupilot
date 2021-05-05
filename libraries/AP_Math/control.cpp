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

// control default definitions
#define CONTROL_TIME_CONSTANT_RATIO 4.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation

// update_pos_vel_accel - single axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
void update_pos_vel_accel(float& pos, float& vel, float accel, float dt)
{
    // move position and velocity forward by dt.
    pos = pos + vel * dt + accel * 0.5f * sq(dt);
    vel = vel + accel * dt;
}

/* shape_vel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
 The accel_max limit can be removed by setting it to zero.
*/
void shape_vel(float& vel_input, float vel, float& accel, float accel_max, float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPa = 1.0 / tc;
    const float jerk_max = accel_max * KPa;

    // velocity error to be corrected
    float vel_error = vel_input - vel;

    // acceleration to correct velocity
    const float accel_target = vel_error * KPa;

    // jerk limit acceleration change
    float accel_delta = accel_target - accel;
    if (is_positive(jerk_max)) {
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
    }
    accel += accel_delta;

    // limit acceleration to accel_max
    if (is_positive(accel_max)) {
        accel = constrain_float(accel, -accel_max, accel_max);
    }

    // Calculate maximum pos_input and vel_input based on limited system
    vel_error = accel / KPa;
    vel_input = vel_error + vel;
}


/* shape_pos_vel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 The vel_max, vel_correction_max, and accel_max limits can be removed by setting the desired limit to zero.
*/
void shape_pos_vel(float& pos_input, float vel_input, float pos, float vel, float& accel, float vel_max, float vel_correction_max, float accel_max, float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = 1.0 / (CONTROL_TIME_CONSTANT_RATIO*tc);
    const float accel_tc_max = accel_max*(1-1.0f/CONTROL_TIME_CONSTANT_RATIO);

    // position error to be corrected
    float pos_error = pos_input - pos;

    // velocity to correct position
    float vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity correction to vel_correction_max
    if (is_positive(vel_correction_max)) {
        vel_target = constrain_float(vel_target, -vel_correction_max, vel_correction_max);
    }

    // velocity correction with input velocity
    vel_target += vel_input;

    // limit velocity to vel_max
    if (is_positive(vel_max)) {
        vel_target = constrain_float(vel_target, -vel_max, vel_max);
    }

    shape_vel(vel_target, vel, accel, accel_max, tc, dt);

    vel_target -= vel_input;
    pos_error = stopping_distance(vel_target, KPv, accel_tc_max);
    pos_input = pos_error + pos;
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
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        const float linear_dist = second_ord_lim / sq(p);
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
    if (!is_positive(D_max) && is_zero(p)) {
        return 0.0f;
    }

    if (!is_positive(D_max)) {
        // second order limit is zero or negative.
        return output / p;
    }

    if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(D_max)) {
            return sq(output)/(2*D_max);
        }
        return -sq(output)/(2*D_max);
    }

    // both the P and second order limit have been defined.
    float linear_out = D_max / p;
    if (output > linear_out) {
        if (is_positive(D_max)) {
            return sq(output)/(2*D_max) + D_max/(4*sq(p));
        }
        return -sq(output)/(2*D_max) + D_max/(4*sq(p));
    }

    return output / p;
}

// calculate the stopping distance for the square root controller based deceleration path
float stopping_distance(float velocity, float p, float accel_max)
{
    if (is_positive(accel_max) && is_zero(p)) {
        return (velocity * velocity) / (2.0f * accel_max);
    }
    if ((is_negative(accel_max) || is_zero(accel_max)) && !is_zero(p)) {
        return velocity / p;
    }
    if ((is_negative(accel_max) || is_zero(accel_max)) && is_zero(p)) {
        return 0.0f;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    const float linear_velocity = accel_max / p;

    if (fabsf(velocity) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return velocity / p;
    }

    const float linear_dist = accel_max / sq(p);
    const float stopping_dist = (linear_dist * 0.5f) + sq(velocity) / (2.0f * accel_max);
    return is_positive(velocity) ? stopping_dist : -stopping_dist;
}

// calculate the maximum acceleration or velocity in a given direction
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg)
{
    if (is_zero(direction.length_squared())) {
        return 0.0f;
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
