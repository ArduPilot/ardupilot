#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include "vector2.h"
#include "vector3.h"

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

// Projects velocity forward in time using acceleration, constrained by directional limit.
// - If `limit` is non-zero, it defines a direction in which acceleration is constrained.
// - The `vel_error` value defines the direction of velocity error (its sign matters, not its magnitude).
// - When `limit` is active, velocity is only updated if doing so would not increase the error in the limited direction.
// - If velocity is currently opposing the limit direction, the update is clipped to avoid crossing zero.
// This prevents unwanted acceleration in the direction of a constraint when it would worsen velocity error.
void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error);

// Projects position and velocity forward in time using acceleration, constrained by directional limit.
// - `limit` defines the constrained direction of motion.
// - `pos_error` and `vel_error` define the sign of error in position and velocity respectively (magnitude is ignored).
// - If the update would increase position error in the constrained direction, the position update is skipped.
// - The velocity update then proceeds with directional limit handling via `update_vel_accel()`.
// This prevents motion in a constrained direction if it would worsen the position or velocity error.
void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit, float pos_error, float vel_error);

// Projects velocity forward in time using acceleration, constrained by directional limits.
// - If the `limit` vector is non-zero, it defines a direction in which acceleration is constrained.
// - The `vel_error` vector defines the direction of velocity error (its magnitude is unused).
// - When `limit` is active, velocity is only updated if doing so would not increase the error in the limited direction.
// This function prevents the system from increasing velocity along the limit direction
// if doing so would worsen the velocity error.
void update_vel_accel_xy(Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& vel_error);

// Projects position and velocity forward in time using acceleration, constrained by directional limits.
// - The `limit` vector defines a directional constraint: if non-zero, motion in that direction is restricted.
// - The `pos_error` and `vel_error` vectors represent the direction of error (magnitude is not used).
// - If a motion step would increase error along a limited axis, it is suppressed.
// This function avoids changes to position or velocity in the direction of `limit`
// if those changes would worsen the position or velocity error respectively.
void update_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, float dt, const Vector2f& limit, const Vector2f& pos_error, const Vector2f& vel_error);

// Applies jerk-limited shaping to the acceleration value to gradually approach a new target.
// - Constrains the rate of change of acceleration to be within ±`jerk_max` over time `dt`.
// - The current acceleration value is modified in-place.
// Useful for ensuring smooth transitions in thrust or lean angle command profiles.
void shape_accel(float accel_desired, float& accel,
                 float jerk_max, float dt);

// Applies jerk-limited shaping to a 2D acceleration vector.
// - Constrains the rate of change of acceleration to a maximum of `jerk_max` over time `dt`.
// - The current acceleration vector is modified in-place to approach `accel_desired`.
/// Ensures smooth acceleration transitions in both axes simultaneously.
void shape_accel_xy(const Vector2f& accel_desired, Vector2f& accel,
                    float jerk_max, float dt);

void shape_accel_xy(const Vector3f& accel_desired, Vector3f& accel,
                    float jerk_max, float dt);

// Shapes velocity and acceleration using jerk-limited control.
// - Computes correction acceleration needed to reach `vel_desired` from current `vel`.
// - Uses a square-root controller with max acceleration and jerk constraints.
// - Correction is combined with feedforward `accel_desired`.
// - If `limit_total_accel` is true, total acceleration is constrained to `accel_min` / `accel_max`.
// The result is applied via `shape_accel`.
void shape_vel_accel(float vel_desired, float accel_desired,
                     float vel, float& accel,
                     float accel_min, float accel_max,
                     float jerk_max, float dt, bool limit_total_accel);

// Computes a jerk-limited acceleration command in 2D to track a desired velocity input.
// - Uses a square-root controller to calculate correction acceleration based on velocity error.
// - Correction is constrained to stay within `accel_max` (total acceleration magnitude).
// - Correction is added to `accel_desired` (feedforward).
// - If `limit_total_accel` is true, total acceleration is constrained after summing.
// Ensures velocity tracking with smooth, physically constrained motion.
void shape_vel_accel_xy(const Vector2f& vel_desired, const Vector2f& accel_desired,
                        const Vector2f& vel, Vector2f& accel,
                        float accel_max, float jerk_max, float dt, bool limit_total_accel);

// Shapes position, velocity, and acceleration using a jerk-limited profile.
// - Computes velocity to close position error using a square-root controller.
// - That velocity is then shaped via `shape_vel_accel` to enforce acceleration and jerk limits.
// - Limits can be applied separately to correction and total values.
// Used for smooth point-to-point motion with constrained dynamics.
void shape_pos_vel_accel(const postype_t pos_desired, float vel_desired, float accel_desired,
                         const postype_t pos, float vel, float& accel,
                         float vel_min, float vel_max,
                         float accel_min, float accel_max,
                         float jerk_max, float dt, bool limit_total);

// Computes a jerk-limited acceleration profile to move toward a position and velocity target in 2D.
// - Computes a velocity correction based on position error using a square-root controller.
// - That velocity is passed to `shape_vel_accel_xy` to generate a constrained acceleration.
// - Limits include: maximum velocity (`vel_max`), maximum acceleration (`accel_max`), and maximum jerk (`jerk_max`).
// - If `limit_total` is true, constraints are applied to the total command (not just the correction).
// Provides smooth trajectory shaping for lateral motion with bounded dynamics.
void shape_pos_vel_accel_xy(const Vector2p& pos_desired, const Vector2f& vel_desired, const Vector2f& accel_desired,
                            const Vector2p& pos, const Vector2f& vel, Vector2f& accel,
                            float vel_max, float accel_max,
                            float jerk_max, float dt, bool limit_total);

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
                         float angle_jerk_max, float dt, bool limit_total);

// Limits a 2D acceleration vector to prioritize lateral (cross-track) acceleration over longitudinal (in-track) acceleration.
// - `vel` defines the current direction of motion (used to split the acceleration).
// - `accel` is modified in-place to remain within `accel_max`.
// - If the full acceleration vector exceeds `accel_max`, it is reshaped to prioritize lateral correction.
// - If `vel` is zero, a simple magnitude limit is applied.
// Returns true if the acceleration vector was modified.
bool limit_accel_xy(const Vector2f& vel, Vector2f& accel, float accel_max);

// Limits acceleration magnitude while preserving cross-track authority during turns.
// - Splits acceleration into components parallel and perpendicular to velocity.
// - Ensures sufficient lateral acceleration is reserved for path curvature.
// - If forward acceleration dominates, lateral acceleration is limited to
//   CORNER_ACCELERATION_RATIO * accel_max.
// Returns true if limiting was applied.
bool limit_accel_corner_xy(const Vector2f& vel, Vector2f& accel, float accel_max);

// Piecewise square-root + linear controller that limits second-order response (acceleration).
// - Behaves like a P controller near the setpoint.
// - Switches to sqrt(2·a·Δx) shaping beyond a threshold to limit acceleration.
// - `second_ord_lim` sets the max acceleration allowed.
// - Returns the constrained correction rate for a given error and gain.
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

// Vector form of `sqrt_controller()`, applied along the direction of the input error vector.
// - Returns a correction vector with magnitude shaped using `sqrt_controller()`.
// - Direction is preserved from the input error.
// - Used in 2D position or velocity control with second-order constraints.
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);

// Inverts the output of `sqrt_controller()` to recover the input error that would produce a given output.
// - Useful for calculating required error to produce a desired rate.
// - Handles both linear and square-root regions of the controller response.
float inv_sqrt_controller(float output, float p, float D_max);

// Calculates stopping distance required to reduce a velocity to zero using a square-root controller.
// - Uses the inverse of the `sqrt_controller()` response curve.
// - Inputs: velocity, P gain, and max deceleration (`accel_max`)
// - Output: stopping distance required to decelerate cleanly.
float stopping_distance(float velocity, float p, float accel_max);

// Computes the maximum possible acceleration or velocity in a specified 3D direction,
// constrained by separate limits in horizontal (XY) and vertical (Z) axes.
// - `direction` should be a non-zero vector indicating desired direction of travel.
// - Limits: max_xy, max_z_pos (upward), max_z_neg (downward)
// Returns the maximum achievable magnitude in that direction without violating any axis constraint.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_neg, float max_z_pos);

// compute the maximum allowed magnitude along a direction defined by segment_length_xy and segment_length_z components
// constrained by independent horizontal (max_xy) and vertical (max_z_pos/max_z_neg) limits
// returns the maximum achievable magnitude without exceeding any axis limit
float kinematic_limit(float segment_length_xy, float segment_length_z, float max_xy, float max_z_pos, float max_z_neg);

// Applies an exponential curve to a normalized input in the range [-1, 1].
// - `expo` shapes the curve (0 = linear, closer to 1 = more curvature).
// - Typically used for pilot stick input response shaping.
// - Clipped to `expo < 0.95` to avoid divide-by-zero or extreme scaling.
float input_expo(float input, float expo);

// Converts a lean angle (radians) to horizontal acceleration in m/s².
// Assumes flat Earth and small angle approximation: a = g * tan(θ)
float angle_rad_to_accel_mss(float angle_rad);

// Converts a lean angle (degrees) to horizontal acceleration in m/s².
float angle_deg_to_accel_mss(float angle_deg);

// Converts a horizontal acceleration (m/s²) to lean angle in radians.
// Assumes: angle = atan(a / g)
float accel_mss_to_angle_rad(float accel_mss);

// Converts a horizontal acceleration (m/s²) to lean angle in degrees.
float accel_mss_to_angle_deg(float accel_mss);

// Converts pilot’s normalized roll/pitch input into target roll and pitch angles (radians).
// - `roll_in_norm` and `pitch_in_norm`: stick inputs in range [-1, 1]
// - `angle_max_rad`: maximum allowed lean angle
// - `angle_limit_rad`: secondary limit to constrain output while preserving full stick range
// Outputs are Euler angles in radians: `roll_out_rad`, `pitch_out_rad`
void rc_input_to_roll_pitch_rad(float roll_in_norm, float pitch_in_norm, float angle_max_rad, float angle_limit_rad, float &roll_out_rad, float &pitch_out_rad);
