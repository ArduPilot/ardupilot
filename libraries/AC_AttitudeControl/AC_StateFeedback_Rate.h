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

#pragma once

/// @file    AC_StateFeedback_Rate.h
/// @brief   State feedback rate controller for ArduSub

#include <AP_Math/AP_Math.h>
#include "AC_StateFeedback_Params.h"

/// @class AC_StateFeedback_Rate
/// @brief State-space feedback controller for angular rate control
///
/// Implements the control law: u = -K * (x_desired - x_actual)
/// where:
///   x = [roll_rate, pitch_rate, yaw_rate] (rad/s, body frame)
///   u = [roll_torque, pitch_torque, yaw_torque] (normalized, -1 to +1)
///   K = 3×3 gain matrix
///
/// The controller uses a simplified decoupled model:
///   ẋ = Ax + Bu
///   A = diag([-Dx/Ixx, -Dy/Iyy, -Dz/Izz])  (damping)
///   B = diag([1/Ixx, 1/Iyy, 1/Izz])         (control effectiveness)
///
/// This assumes underwater vehicle dynamics are dominated by:
///   - Moment of inertia (Ixx, Iyy, Izz)
///   - Hydrodynamic damping (Dx, Dy, Dz)
///   - Weak cross-axis coupling (can be ignored for stability)
class AC_StateFeedback_Rate {
public:
    /// @brief Constructor
    /// @param params Reference to state feedback parameters
    AC_StateFeedback_Rate(const AC_StateFeedback_Params& params);

    /// @brief Main control update - computes control output
    /// @param rate_desired_rads Desired angular rates [roll, pitch, yaw] (rad/s)
    /// @param rate_actual_rads Actual angular rates from gyro (rad/s)
    /// @param dt Time step (seconds)
    /// @return Control output vector [roll, pitch, yaw] normalized to [-1, 1]
    Vector3f update(const Vector3f& rate_desired_rads,
                   const Vector3f& rate_actual_rads,
                   float dt);

    /// @brief Calculate LQR gains from Q/R matrices (online computation)
    /// @note Only used if params.lqr_mode == 1
    /// @return true if successful, false on error
    bool calculate_lqr_gains();

    /// @brief Get current gain matrix (for logging/debugging)
    /// @return 3×3 gain matrix K
    const Matrix3f& get_gains() const { return _K; }

    /// @brief Reset controller state (called on mode changes)
    void reset();

    /// @brief Check if gains are valid (non-zero, not NaN)
    /// @return true if gains are valid
    bool gains_valid() const;

private:
    /// Reference to parameter structure
    const AC_StateFeedback_Params& _params;

    /// Gain matrix K (3×3)
    /// K[row][col] maps state errors to control outputs
    Matrix3f _K;

    /// Last update time (for detecting dt anomalies)
    uint64_t _last_update_us;

    /// Gain validity flag
    bool _gains_initialized;

    /// @brief Load gain matrix from parameters
    /// @note Called during initialization and when parameters change
    void load_gains();

    /// @brief Apply control saturation limits
    /// @param control Control vector to saturate
    /// @return Saturated control vector (each element in [-1, 1])
    Vector3f saturate_control(const Vector3f& control) const;
};
