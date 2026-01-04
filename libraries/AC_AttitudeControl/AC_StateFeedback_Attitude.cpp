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

#include "AC_StateFeedback_Attitude.h"
#include <AP_HAL/AP_HAL.h>

// Constructor
AC_StateFeedback_Attitude::AC_StateFeedback_Attitude(const AC_StateFeedback_Params& params)
    : _params(params)
    , _last_update_us(0)
    , _gains_initialized(false)
{
    // Initialize gain matrix to zero
    for (int i = 0; i < 6; i++) {
        _K[i].zero();
    }

    // Load gains from parameters
    load_gains();
}

// Main control update
Vector3f AC_StateFeedback_Attitude::update(const Vector3f& att_desired_rad,
                                           const Vector3f& att_actual_rad,
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
    // State vector: [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
    Vector3f angle_error;
    angle_error.x = wrap_PI(att_desired_rad.x - att_actual_rad.x);  // Roll error
    angle_error.y = wrap_PI(att_desired_rad.y - att_actual_rad.y);  // Pitch error
    angle_error.z = wrap_PI(att_desired_rad.z - att_actual_rad.z);  // Yaw error

    // Rate error (desired rate is 0 for attitude hold, or could be passed in)
    // For now, assume desired rates are 0 (pure attitude stabilization)
    Vector3f rate_error = -rate_actual_rads;

    // State feedback control law: u = -K * e
    // K is 3×6, e is 6×1, result is 3×1
    // K stored in column-major: _K[col] = [K[0,col], K[1,col], K[2,col]]
    Vector3f control;
    control.zero();

    // Matrix-vector multiply: u = -K * [angle_error; rate_error]
    // Column 0-2: angle errors
    control += _K[0] * angle_error.x;  // Roll angle contribution
    control += _K[1] * angle_error.y;  // Pitch angle contribution
    control += _K[2] * angle_error.z;  // Yaw angle contribution

    // Column 3-5: rate errors
    control += _K[3] * rate_error.x;   // Roll rate contribution
    control += _K[4] * rate_error.y;   // Pitch rate contribution
    control += _K[5] * rate_error.z;   // Yaw rate contribution

    // Negate (control law is u = -K*e, but we accumulated K*e)
    control = -control;

    // Apply saturation (motor limits)
    return saturate_control(control);
}

// Load gain matrix from parameters
void AC_StateFeedback_Attitude::load_gains()
{
    // Load 3×6 gain matrix K from parameters (row-major in params)
    // Store in column-major format: _K[col] = [K[0,col], K[1,col], K[2,col]]

    // Column 0: roll angle → [roll, pitch, yaw] control
    _K[0].x = _params.attitude.K1;   // K[0,0]: roll angle → roll control
    _K[0].y = _params.attitude.K7;   // K[1,0]: roll angle → pitch control
    _K[0].z = _params.attitude.K13;  // K[2,0]: roll angle → yaw control

    // Column 1: pitch angle → [roll, pitch, yaw] control
    _K[1].x = _params.attitude.K2;   // K[0,1]: pitch angle → roll control
    _K[1].y = _params.attitude.K8;   // K[1,1]: pitch angle → pitch control
    _K[1].z = _params.attitude.K14;  // K[2,1]: pitch angle → yaw control

    // Column 2: yaw angle → [roll, pitch, yaw] control
    _K[2].x = _params.attitude.K3;   // K[0,2]: yaw angle → roll control
    _K[2].y = _params.attitude.K9;   // K[1,2]: yaw angle → pitch control
    _K[2].z = _params.attitude.K15;  // K[2,2]: yaw angle → yaw control

    // Column 3: roll rate → [roll, pitch, yaw] control
    _K[3].x = _params.attitude.K4;   // K[0,3]: roll rate → roll control
    _K[3].y = _params.attitude.K10;  // K[1,3]: roll rate → pitch control
    _K[3].z = _params.attitude.K16;  // K[2,3]: roll rate → yaw control

    // Column 4: pitch rate → [roll, pitch, yaw] control
    _K[4].x = _params.attitude.K5;   // K[0,4]: pitch rate → roll control
    _K[4].y = _params.attitude.K11;  // K[1,4]: pitch rate → pitch control
    _K[4].z = _params.attitude.K17;  // K[2,4]: pitch rate → yaw control

    // Column 5: yaw rate → [roll, pitch, yaw] control
    _K[5].x = _params.attitude.K6;   // K[0,5]: yaw rate → roll control
    _K[5].y = _params.attitude.K12;  // K[1,5]: yaw rate → pitch control
    _K[5].z = _params.attitude.K18;  // K[2,5]: yaw rate → yaw control

    _gains_initialized = true;
}

// Apply control saturation
Vector3f AC_StateFeedback_Attitude::saturate_control(const Vector3f& control) const
{
    Vector3f saturated;
    saturated.x = constrain_float(control.x, -1.0f, 1.0f);
    saturated.y = constrain_float(control.y, -1.0f, 1.0f);
    saturated.z = constrain_float(control.z, -1.0f, 1.0f);
    return saturated;
}

// Reset controller state
void AC_StateFeedback_Attitude::reset()
{
    _last_update_us = 0;
    // Gains remain loaded, only reset timing
}

// Check if gains are valid
bool AC_StateFeedback_Attitude::gains_valid() const
{
    if (!_gains_initialized) {
        return false;
    }

    // Check diagonal elements are non-zero
    // For decoupled system, K[0,0], K[1,1], K[2,2] should be non-zero
    // These are in _K[0].x, _K[1].y, _K[2].z (angle gains on diagonal)
    if (is_zero(_K[0].x) || is_zero(_K[1].y) || is_zero(_K[2].z)) {
        return false;
    }

    // Check for NaN
    for (int i = 0; i < 6; i++) {
        if (isnan(_K[i].x) || isnan(_K[i].y) || isnan(_K[i].z)) {
            return false;
        }
    }

    // Check gains are reasonable (not excessively large)
    const float max_gain = 100.0f;
    for (int i = 0; i < 6; i++) {
        if (fabsf(_K[i].x) > max_gain || fabsf(_K[i].y) > max_gain || fabsf(_K[i].z) > max_gain) {
            return false;
        }
    }

    return true;
}

// Calculate LQR gains from Q/R matrices (stub for now)
bool AC_StateFeedback_Attitude::calculate_lqr_gains()
{
    // TODO: Implement online LQR solver
    // For now, users should use the offline Python tool (lqr_gain_calculator.py)
    // and set pre-calculated gains via parameters

    // If we reach here without implementation, just load from parameters
    load_gains();
    return true;
}

// Get gains for logging/debugging
void AC_StateFeedback_Attitude::get_gains(Vector3f gains[6]) const
{
    for (int i = 0; i < 6; i++) {
        gains[i] = _K[i];
    }
}

// Normalize angle to [-π, π]
float AC_StateFeedback_Attitude::wrap_PI(float angle) const
{
    // Use AP_Math's wrap_PI function
    return ::wrap_PI(angle);
}
