#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "vector2.h"
#include "vector3.h"

#ifndef HAL_WITH_POSTYPE_DOUBLE
#define HAL_WITH_POSTYPE_DOUBLE BOARD_FLASH_SIZE > 1024
#endif

#if HAL_WITH_POSTYPE_DOUBLE
typedef double postype_t;
typedef Vector2d Vector2p;
typedef Vector3d Vector3p;
#define topostype todouble
#else
typedef float postype_t;
typedef Vector2f Vector2p;
typedef Vector3f Vector3p;
#define topostype tofloat
#endif

/*
  common controller helper functions
 */

// update_vel_accel projects the velocity, vel, forward in time based on a time step of dt and acceleration of accel.
// update_vel_accel - single axis projection.
void update_vel_accel(float& vel, float accel, float dt, float limit);

// update_vel_accel projects the velocity, vel, forward in time based on a time step of dt and acceleration of accel.
// update_vel_accel - single axis projection.
void update_pos_vel_accel(postype_t & pos, float& vel, float accel, float dt, float limit);

// update_pos_vel_accel_xy - dual axis projection operating on the x, y axis of Vector2f or Vector3f inputs.
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, Vector2f limit);

// update_pos_vel_accel_xy - dual axis projection operating on the x, y axis of Vector2f or Vector3f inputs.
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, Vector2f limit);

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
                 float jerk_max, float dt);

void shape_accel_xy(const Vector2f& accel_input, Vector2f& accel,
                    float jerk_max, float dt);

void shape_accel_xy(const Vector3f& accel_input, Vector3f& accel,
                    float jerk_max, float dt);

/* shape_vel calculates a jerk limited path from the current velocity and acceleration to an input velocity.
 The function takes the current velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     velocity limits - vel_min, vel_max,
     acceleration limits - accel_min, accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to vel_input and accel_input
*/
void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel);

void shape_vel_accel_xy(const Vector2f &vel_input1, const Vector2f& accel_input,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel);

/* shape_pos_vel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to pos_input, vel_input and accel_input
*/
void shape_pos_vel_accel(const postype_t pos_input, float vel_input, float accel_input,
                         const postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total_accel);

void shape_pos_vel_accel_xy(const Vector2p& pos_input, const Vector2f& vel_input, const Vector2f& accel_input,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total_accel);

// proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

// proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);

// inverse of the sqrt controller.  calculates the input (aka error) to the sqrt_controller required to achieve a given output
float inv_sqrt_controller(float output, float p, float D_max);

// calculate the stopping distance for the square root controller based deceleration path
float stopping_distance(float velocity, float p, float accel_max);

// calculate the maximum acceleration or velocity in a given direction
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);
