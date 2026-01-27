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

#include "AC_StateFeedback_Rate.h"
#include <AP_HAL/AP_HAL.h>

// Constructor
AC_StateFeedback_Rate::AC_StateFeedback_Rate(const AC_StateFeedback_Params& params)
    : _params(params)
    , _last_update_us(0)
    , _gains_initialized(false)
{
    // Initialize gain matrix to zero
    _K.zero();

    // Load gains from parameters
    load_gains();
}

// Main control update
Vector3f AC_StateFeedback_Rate::update(const Vector3f& rate_desired_rads,
                                       const Vector3f& rate_actual_rads,
                                       float dt)
{
    // Reload gains if LQR mode changed or not initialized
    if (!_gains_initialized || (_params.lqr_mode == 1 && _last_update_us == 0)) {
        load_gains();

        // If LQR mode enabled, calculate gains online
        if (_params.lqr_mode == 1) {
            calculate_lqr_gains();
        }
    }

    // Update timestamp
    _last_update_us = AP_HAL::micros64();

    // Sanity check gains
    if (!gains_valid()) {
        // Return zero control if gains invalid (safety fallback)
        return Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Compute state error: e = x_desired - x_actual
    Vector3f error = rate_desired_rads - rate_actual_rads;

    // State feedback control law: u = -K * e
    // K is 3×3, error is 3×1, result is 3×1
    Vector3f control;
    control.x = -(_K.a.x * error.x + _K.a.y * error.y + _K.a.z * error.z);
    control.y = -(_K.b.x * error.x + _K.b.y * error.y + _K.b.z * error.z);
    control.z = -(_K.c.x * error.x + _K.c.y * error.y + _K.c.z * error.z);

    // Apply saturation (motor limits)
    return saturate_control(control);
}

// Load gain matrix from parameters
void AC_StateFeedback_Rate::load_gains()
{
    // Load 3×3 gain matrix K from parameters (row-major order)
    // Row 0: roll control
    _K.a.x = _params.rate.K1;  // K[0,0]: roll rate → roll control
    _K.a.y = _params.rate.K2;  // K[0,1]: pitch rate → roll control
    _K.a.z = _params.rate.K3;  // K[0,2]: yaw rate → roll control

    // Row 1: pitch control
    _K.b.x = _params.rate.K4;  // K[1,0]: roll rate → pitch control
    _K.b.y = _params.rate.K5;  // K[1,1]: pitch rate → pitch control
    _K.b.z = _params.rate.K6;  // K[1,2]: yaw rate → pitch control

    // Row 2: yaw control
    _K.c.x = _params.rate.K7;  // K[2,0]: roll rate → yaw control
    _K.c.y = _params.rate.K8;  // K[2,1]: pitch rate → yaw control
    _K.c.z = _params.rate.K9;  // K[2,2]: yaw rate → yaw control

    _gains_initialized = true;
}

// Apply control saturation
Vector3f AC_StateFeedback_Rate::saturate_control(const Vector3f& control) const
{
    Vector3f saturated;
    saturated.x = constrain_float(control.x, -1.0f, 1.0f);
    saturated.y = constrain_float(control.y, -1.0f, 1.0f);
    saturated.z = constrain_float(control.z, -1.0f, 1.0f);
    return saturated;
}

// Reset controller state
void AC_StateFeedback_Rate::reset()
{
    _last_update_us = 0;
    // Gains remain loaded, only reset timing
}

// Check if gains are valid
bool AC_StateFeedback_Rate::gains_valid() const
{
    if (!_gains_initialized) {
        return false;
    }

    // Check diagonal elements are non-zero (at least)
    // For decoupled system, K[0,0], K[1,1], K[2,2] should be non-zero
    if (is_zero(_K.a.x) || is_zero(_K.b.y) || is_zero(_K.c.z)) {
        return false;
    }

    // Check for NaN
    if (isnan(_K.a.x) || isnan(_K.b.y) || isnan(_K.c.z)) {
        return false;
    }

    // Check gains are reasonable (not excessively large)
    // Maximum gain of 100 is very conservative
    const float max_gain = 100.0f;
    if (fabsf(_K.a.x) > max_gain || fabsf(_K.b.y) > max_gain || fabsf(_K.c.z) > max_gain) {
        return false;
    }

    return true;
}

// Calculate LQR gains from Q/R matrices (stub for now)
bool AC_StateFeedback_Rate::calculate_lqr_gains()
{
    // TODO: Implement online LQR solver using Discrete Algebraic Riccati Equation (DARE)
    // This would require:
    // 1. Build A, B matrices from inertia/damping parameters
    // 2. Discretize to Ad, Bd using matrix exponential
    // 3. Solve DARE: P = Ad'*P*Ad - (Ad'*P*Bd)*inv(R + Bd'*P*Bd)*(Bd'*P*Ad) + Q
    // 4. Compute K = inv(R + Bd'*P*Bd)*(Bd'*P*Ad)
    //
    // For now, users should use the offline Python tool (lqr_gain_calculator.py)
    // and set pre-calculated gains via parameters

    // If we reach here without implementation, just load from parameters
    load_gains();
    return true;
}
