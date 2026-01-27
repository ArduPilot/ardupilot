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

/// @file    AC_StateFeedback_Attitude.h
/// @brief   State feedback attitude controller for ArduSub

#include <AP_Math/AP_Math.h>
#include "AC_StateFeedback_Params.h"

/// @class AC_StateFeedback_Attitude
/// @brief State-space feedback controller for attitude control
///
/// Implements the control law: u = -K * (x_desired - x_actual)
/// where:
///   x = [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate] (6 states)
///       angles in radians, rates in rad/s
///   u = [roll_torque, pitch_torque, yaw_torque] (normalized, -1 to +1)
///   K = 3×6 gain matrix
///
/// The controller uses a linearized attitude dynamics model:
///   ẋ = Ax + Bu
///   where:
///     x[0:2] = [φ, θ, ψ] (Euler angles)
///     x[3:5] = [p, q, r] (body rates)
///
///   A = [[0,  0,  0,  1,  0,  0],      # φ̇ = p
///        [0,  0,  0,  0,  1,  0],      # θ̇ = q
///        [0,  0,  0,  0,  0,  1],      # ψ̇ = r
///        [0,  0,  0, -Dx, 0,  0],      # ṗ = -Dx*p + u_roll/Ixx
///        [0,  0,  0,  0, -Dy, 0],      # q̇ = -Dy*q + u_pitch/Iyy
///        [0,  0,  0,  0,  0, -Dz]]     # ṙ = -Dz*r + u_yaw/Izz
///
///   B = [[0,      0,      0     ],
///        [0,      0,      0     ],
///        [0,      0,      0     ],
///        [1/Ixx,  0,      0     ],
///        [0,      1/Iyy,  0     ],
///        [0,      0,      1/Izz]]
///
/// This assumes small angle approximations and decoupled dynamics.
class AC_StateFeedback_Attitude {
public:
    /// @brief Constructor
    /// @param params Reference to state feedback parameters
    AC_StateFeedback_Attitude(const AC_StateFeedback_Params& params);

    /// @brief Main control update - computes control output
    /// @param att_desired_rad Desired Euler angles [roll, pitch, yaw] (rad)
    /// @param att_actual_rad Actual Euler angles [roll, pitch, yaw] (rad)
    /// @param rate_actual_rads Actual angular rates from gyro [p, q, r] (rad/s)
    /// @param dt Time step (seconds)
    /// @return Control output vector [roll, pitch, yaw] normalized to [-1, 1]
    Vector3f update(const Vector3f& att_desired_rad,
                   const Vector3f& att_actual_rad,
                   const Vector3f& rate_actual_rads,
                   float dt);

    /// @brief Calculate LQR gains from Q/R matrices (online computation)
    /// @note Only used if params.lqr_mode == 1
    /// @return true if successful, false on error
    bool calculate_lqr_gains();

    /// @brief Get current gain matrix (for logging/debugging)
    /// @return 3×6 gain matrix K stored as 3 Vector6f
    void get_gains(Vector3f gains[6]) const;

    /// @brief Reset controller state (called on mode changes)
    void reset();

    /// @brief Check if gains are valid (non-zero, not NaN)
    /// @return true if gains are valid
    bool gains_valid() const;

private:
    /// Reference to parameter structure
    const AC_StateFeedback_Params& _params;

    /// Gain matrix K (3×6)
    /// K[row][col] maps state errors to control outputs
    /// Stored as array of 6 Vector3f (column-major for easier access)
    Vector3f _K[6];  // K[col] = [K[0,col], K[1,col], K[2,col]]

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

    /// @brief Normalize angle to [-π, π]
    /// @param angle Angle in radians
    /// @return Normalized angle
    float wrap_PI(float angle) const;
};
