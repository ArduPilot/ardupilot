/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AC_StateFeedback_Position.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Constructor
AC_StateFeedback_Position::AC_StateFeedback_Position(const AC_StateFeedback_Params& params)
    : _params(params)
    , _last_update_us(0)
    , _gains_initialized(false)
{
    // Initialize gain matrix to zero
    for (uint8_t i = 0; i < 12; i++) {
        _K[i].zero();
    }

    // Load gains from parameters
    load_gains();
}

// Main control update
Vector4f AC_StateFeedback_Position::update(const Vector3f& pos_desired_ned,
                                           const Vector3f& pos_actual_ned,
                                           const Vector3f& vel_desired_ned,
                                           const Vector3f& vel_actual_ned,
                                           const Vector3f& att_desired_rad,
                                           const Vector3f& att_actual_rad,
                                           const Vector3f& rate_actual_rads,
                                           float dt)
{
    // Update timestamp
    _last_update_us = AP_HAL::micros64();

    // Reload gains if parameters changed
    if (!_gains_initialized) {
        load_gains();
    }

    // Check for valid gains
    if (!gains_valid()) {
        // Return zero control if gains are invalid
        return Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
    }

    // Compute state error vector (12 states)
    // x_error = x_desired - x_actual
    Vector3f pos_error = pos_desired_ned - pos_actual_ned;
    Vector3f vel_error = vel_desired_ned - vel_actual_ned;

    // Attitude error with angle wrapping
    Vector3f att_error;
    att_error.x = wrap_PI(att_desired_rad.x - att_actual_rad.x);  // Roll
    att_error.y = wrap_PI(att_desired_rad.y - att_actual_rad.y);  // Pitch
    att_error.z = wrap_PI(att_desired_rad.z - att_actual_rad.z);  // Yaw

    // Rate error (desired rates are typically zero for position control)
    Vector3f rate_error = -rate_actual_rads;

    // Compute control law: u = -K * x_error
    // K is 4×12, x_error is 12×1, u is 4×1
    Vector4f control(0.0f, 0.0f, 0.0f, 0.0f);

    // Matrix-vector multiplication: u = K * x_error
    // K stored as _K[col] = [K[0,col], K[1,col], K[2,col], K[3,col]]
    control += _K[0]  * pos_error.x;     // x position error
    control += _K[1]  * pos_error.y;     // y position error
    control += _K[2]  * pos_error.z;     // z position error
    control += _K[3]  * vel_error.x;     // vx velocity error
    control += _K[4]  * vel_error.y;     // vy velocity error
    control += _K[5]  * vel_error.z;     // vz velocity error
    control += _K[6]  * att_error.x;     // roll error
    control += _K[7]  * att_error.y;     // pitch error
    control += _K[8]  * att_error.z;     // yaw error
    control += _K[9]  * rate_error.x;    // p (roll rate) error
    control += _K[10] * rate_error.y;    // q (pitch rate) error
    control += _K[11] * rate_error.z;    // r (yaw rate) error

    // Apply saturation
    return saturate_control(control);
}

// Simplified position hold update (zero velocity targets)
Vector4f AC_StateFeedback_Position::update_pos_hold(const Vector3f& pos_desired_ned,
                                                    const Vector3f& pos_actual_ned,
                                                    const Vector3f& vel_actual_ned,
                                                    const Vector3f& att_actual_rad,
                                                    const Vector3f& rate_actual_rads,
                                                    float dt)
{
    // Call full update with zero velocity targets and hover attitude
    Vector3f vel_desired_ned(0.0f, 0.0f, 0.0f);
    Vector3f att_desired_rad(0.0f, 0.0f, att_actual_rad.z);  // Maintain current yaw, level attitude

    return update(pos_desired_ned, pos_actual_ned,
                 vel_desired_ned, vel_actual_ned,
                 att_desired_rad, att_actual_rad,
                 rate_actual_rads, dt);
}

// Load gain matrix from parameters
void AC_StateFeedback_Position::load_gains()
{
    // Load 4×12 gain matrix from parameters
    // Parameters are K1-K48, stored row-major in params
    // We store them column-major in _K for efficient computation

    const AC_StateFeedback_PositionParams& p = _params.position;

    // Column 0: x position gains
    _K[0] = Vector4f(p.K1, p.K13, p.K25, p.K37);

    // Column 1: y position gains
    _K[1] = Vector4f(p.K2, p.K14, p.K26, p.K38);

    // Column 2: z position gains
    _K[2] = Vector4f(p.K3, p.K15, p.K27, p.K39);

    // Column 3: vx velocity gains
    _K[3] = Vector4f(p.K4, p.K16, p.K28, p.K40);

    // Column 4: vy velocity gains
    _K[4] = Vector4f(p.K5, p.K17, p.K29, p.K41);

    // Column 5: vz velocity gains
    _K[5] = Vector4f(p.K6, p.K18, p.K30, p.K42);

    // Column 6: roll gains
    _K[6] = Vector4f(p.K7, p.K19, p.K31, p.K43);

    // Column 7: pitch gains
    _K[7] = Vector4f(p.K8, p.K20, p.K32, p.K44);

    // Column 8: yaw gains
    _K[8] = Vector4f(p.K9, p.K21, p.K33, p.K45);

    // Column 9: p (roll rate) gains
    _K[9] = Vector4f(p.K10, p.K22, p.K34, p.K46);

    // Column 10: q (pitch rate) gains
    _K[10] = Vector4f(p.K11, p.K23, p.K35, p.K47);

    // Column 11: r (yaw rate) gains
    _K[11] = Vector4f(p.K12, p.K24, p.K36, p.K48);

    _gains_initialized = true;
}

// Calculate LQR gains (placeholder for future implementation)
bool AC_StateFeedback_Position::calculate_lqr_gains()
{
    // TODO: Implement online LQR solver for 12-state system
    // This requires solving the Discrete Algebraic Riccati Equation (DARE)
    // For now, use pre-calculated gains from parameters
    hal.console->printf("AC_StateFeedback_Position: Online LQR not implemented, using parameter gains\n");
    load_gains();
    return _gains_initialized;
}

// Get current gain matrix
void AC_StateFeedback_Position::get_gains(Vector4f gains[12]) const
{
    for (uint8_t i = 0; i < 12; i++) {
        gains[i] = _K[i];
    }
}

// Reset controller state
void AC_StateFeedback_Position::reset()
{
    _last_update_us = 0;
    _gains_initialized = false;
    load_gains();
}

// Check if gains are valid
bool AC_StateFeedback_Position::gains_valid() const
{
    if (!_gains_initialized) {
        return false;
    }

    // Check if any gains are non-zero (at least vertical thrust gains should be non-zero)
    bool has_nonzero = false;
    for (uint8_t i = 0; i < 12; i++) {
        if (!_K[i].is_zero()) {
            has_nonzero = true;
            break;
        }
    }

    // Check for NaN
    for (uint8_t i = 0; i < 12; i++) {
        if (_K[i].is_nan()) {
            return false;
        }
    }

    return has_nonzero;
}

// Apply control saturation
Vector4f AC_StateFeedback_Position::saturate_control(const Vector4f& control) const
{
    Vector4f saturated;

    // Clamp each control output to [-1, 1]
    saturated.x = constrain_float(control.x, -1.0f, 1.0f);  // Vertical thrust
    saturated.y = constrain_float(control.y, -1.0f, 1.0f);  // Roll torque
    saturated.z = constrain_float(control.z, -1.0f, 1.0f);  // Pitch torque
    saturated.w = constrain_float(control.w, -1.0f, 1.0f);  // Yaw torque

    return saturated;
}

// Normalize angle to [-π, π]
float AC_StateFeedback_Position::wrap_PI(float angle) const
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}
