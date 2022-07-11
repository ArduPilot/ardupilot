#pragma once

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

// update_vel_accel - single axis projection of velocity, vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// vel_error - specifies the direction of the velocity error used in limit handling.
void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error);

// update_pos_vel_accel - single axis projection of position and velocity forward in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit, float pos_error, float vel_error);

// update_vel_accel - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& vel_error);

// update_pos_vel_accel - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// the position and velocity is not moved in the direction of limit if limit is not set to zero.
// limit - specifies if the system is unable to continue to accelerate.
// pos_error and vel_error - specifies the direction of the velocity error used in limit handling.
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& pos_error, const Vector2f& vel_error);

/* shape_accel calculates a jerk limited path from the current acceleration to an input acceleration.
 The function takes the current acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    acceleration limits - accel_min, accel_max,
    time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to accel_input.
*/
void shape_accel(float accel_input, float& accel,
                 float jerk_max, float dt);

// 2D version
void shape_accel_xy(const Vector2f& accel_input, Vector2f& accel,
                    float jerk_max, float dt);

void shape_accel_xy(const Vector3f& accel_input, Vector3f& accel,
                    float jerk_max, float dt);

/* shape_vel_accel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    maximum velocity - vel_max,
    maximum acceleration - accel_max,
    time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to vel_input and accel_input.
 The accel_max limit can be removed by setting it to zero.
*/
void shape_vel_accel(float vel_input, float accel_input,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel);

// 2D version
void shape_vel_accel_xy(const Vector2f& vel_input1, const Vector2f& accel_input,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel);

/* shape_pos_vel_accel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
    maximum velocity - vel_max,
    maximum acceleration - accel_max,
    time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the variable accel to follow a jerk limited kinematic path to pos_input, vel_input and accel_input.
 The vel_max, vel_correction_max, and accel_max limits can be removed by setting the desired limit to zero.
*/
void shape_pos_vel_accel(const postype_t pos_input, float vel_input, float accel_input,
                         const postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total_accel);

// 2D version
void shape_pos_vel_accel_xy(const Vector2p& pos_input, const Vector2f& vel_input, const Vector2f& accel_input,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total_accel);

/* limit_accel_xy limits the acceleration to prioritise acceleration perpendicular to the provided velocity vector.
 Input parameters are:
    vel is the velocity vector used to define the direction acceleration limit is biased in.
    accel is the acceleration vector to be limited.
    accel_max is the maximum length of the acceleration vector after being limited.
 Returns true when accel vector has been limited.
*/
bool limit_accel_xy(const Vector2f& vel, Vector2f& accel, float accel_max);

// sqrt_controller calculates the correction based on a proportional controller with piecewise sqrt sections to constrain second derivative.
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

// sqrt_controller calculates the correction based on a proportional controller with piecewise sqrt sections to constrain second derivative.
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);

// inv_sqrt_controller calculates the inverse of the sqrt controller.
// This function calculates the input (aka error) to the sqrt_controller required to achieve a given output.
float inv_sqrt_controller(float output, float p, float D_max);

// stopping_distance calculates the stopping distance for the square root controller based deceleration path.
float stopping_distance(float velocity, float p, float accel_max);

// kinematic_limit calculates the maximum acceleration or velocity in a given direction.
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);

// input_expo calculates the expo function on the normalised input.
// The input must be in the range of -1 to 1.
// The expo should be less than 1.0 but limited to be less than 0.95.
float input_expo(float input, float expo);

// angle_to_accel converts a maximum lean angle in degrees to an accel limit in m/s/s
float angle_to_accel(float angle_deg);

// accel_to_angle converts a maximum accel in m/s/s to a lean angle in degrees
float accel_to_angle(float accel);

// rc_input_to_roll_pitch - transform pilot's normalised roll or pitch stick input into a roll and pitch euler angle command
// roll_in_unit and pitch_in_unit - are normalised roll and pitch stick inputs
// angle_max_deg - maximum lean angle from the z axis
// angle_limit_deg - provides the ability to reduce the maximum output lean angle to less than angle_max_deg
// returns roll and pitch euler angles in degrees
void rc_input_to_roll_pitch(float roll_in_unit, float pitch_in_unit, float angle_max_deg, float angle_limit_deg, float &roll_out_deg, float &pitch_out_deg);
