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

/// @file    AC_StateFeedback_Position.h
/// @brief   State feedback position controller for ArduSub

#include <AP_Math/AP_Math.h>
#include "AC_StateFeedback_Params.h"

// Simple 4-element vector for control outputs
struct Vector4f {
    float x, y, z, w;

    Vector4f() : x(0), y(0), z(0), w(0) {}
    Vector4f(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}

    void zero() { x = y = z = w = 0.0f; }
    bool is_zero() const { return ::is_zero(x) && ::is_zero(y) && ::is_zero(z) && ::is_zero(w); }
    bool is_nan() const { return isnan(x) || isnan(y) || isnan(z) || isnan(w); }

    Vector4f operator+(const Vector4f& v) const {
        return Vector4f(x + v.x, y + v.y, z + v.z, w + v.w);
    }

    Vector4f& operator+=(const Vector4f& v) {
        x += v.x; y += v.y; z += v.z; w += v.w;
        return *this;
    }

    Vector4f operator*(float scalar) const {
        return Vector4f(x * scalar, y * scalar, z * scalar, w * scalar);
    }
};

/// @class AC_StateFeedback_Position
/// @brief State-space feedback controller for full position + attitude control
///
/// Implements the control law: u = -K * (x_desired - x_actual)
/// where:
///   x = [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r] (12 states, NED frame)
///       positions in meters, velocities in m/s
///       angles in radians, rates in rad/s
///   u = [Tz, τ_roll, τ_pitch, τ_yaw] (4 controls, normalized -1 to +1)
///   K = 4×12 gain matrix
///
/// The controller uses a linearized 6-DOF underwater vehicle model:
///   ẋ = Ax + Bu
///   where:
///     x[0:2]  = [x, y, z] (NED position)
///     x[3:5]  = [vx, vy, vz] (NED velocity)
///     x[6:8]  = [φ, θ, ψ] (Euler angles)
///     x[9:11] = [p, q, r] (body rates)
///
///   A = [[0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0],   # ẋ = vx
///        [0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0],   # ẏ = vy
///        [0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0],   # ż = vz
///        [0,  0,  0, -Dx/m, 0,  0,  g,  0,  0,  0,  0,  0], # v̇x = -Dx*vx/m + g*θ + Fx/m
///        [0,  0,  0,  0, -Dy/m, 0, -g,  0,  0,  0,  0,  0], # v̇y = -Dy*vy/m - g*φ + Fy/m
///        [0,  0,  0,  0,  0, -Dz/m, 0,  0,  0,  0,  0,  0], # v̇z = -Dz*vz/m + Fz/m
///        [0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0],   # φ̇ = p
///        [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0],   # θ̇ = q
///        [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1],   # ψ̇ = r
///        [0,  0,  0,  0,  0,  0,  0,  0,  0, -Drx/Ixx, 0, 0], # ṗ = -Drx*p/Ixx + τx/Ixx
///        [0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -Dry/Iyy, 0], # q̇ = -Dry*q/Iyy + τy/Iyy
///        [0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -Drz/Izz]] # ṙ = -Drz*r/Izz + τz/Izz
///
///   B = [[0,      0,       0,       0      ],
///        [0,      0,       0,       0      ],
///        [0,      0,       0,       0      ],
///        [0,      0,       0,       0      ],  (Fx, Fy from attitude, handled by coupling)
///        [0,      0,       0,       0      ],
///        [1/m,    0,       0,       0      ],  (Vertical thrust control)
///        [0,      0,       0,       0      ],
///        [0,      0,       0,       0      ],
///        [0,      0,       0,       0      ],
///        [0,      1/Ixx,   0,       0      ],  (Roll torque control)
///        [0,      0,       1/Iyy,   0      ],  (Pitch torque control)
///        [0,      0,       0,       1/Izz  ]]  (Yaw torque control)
///
/// This assumes small angle approximations and uses simplified coupling.
class AC_StateFeedback_Position {
public:
    /// @brief Constructor
    /// @param params Reference to state feedback parameters
    AC_StateFeedback_Position(const AC_StateFeedback_Params& params);

    /// @brief Main control update - computes control output
    /// @param pos_desired_ned Desired NED position [x, y, z] (m)
    /// @param pos_actual_ned Actual NED position [x, y, z] (m)
    /// @param vel_desired_ned Desired NED velocity [vx, vy, vz] (m/s)
    /// @param vel_actual_ned Actual NED velocity [vx, vy, vz] (m/s)
    /// @param att_desired_rad Desired Euler angles [roll, pitch, yaw] (rad)
    /// @param att_actual_rad Actual Euler angles [roll, pitch, yaw] (rad)
    /// @param rate_actual_rads Actual angular rates from gyro [p, q, r] (rad/s)
    /// @param dt Time step (seconds)
    /// @return Control output: [vertical_thrust, roll_torque, pitch_torque, yaw_torque] normalized to [-1, 1]
    Vector4f update(const Vector3f& pos_desired_ned,
                   const Vector3f& pos_actual_ned,
                   const Vector3f& vel_desired_ned,
                   const Vector3f& vel_actual_ned,
                   const Vector3f& att_desired_rad,
                   const Vector3f& att_actual_rad,
                   const Vector3f& rate_actual_rads,
                   float dt);

    /// @brief Simplified update with zero velocity targets
    /// @param pos_desired_ned Desired NED position [x, y, z] (m)
    /// @param pos_actual_ned Actual NED position [x, y, z] (m)
    /// @param vel_actual_ned Actual NED velocity [vx, vy, vz] (m/s)
    /// @param att_actual_rad Actual Euler angles [roll, pitch, yaw] (rad)
    /// @param rate_actual_rads Actual angular rates from gyro [p, q, r] (rad/s)
    /// @param dt Time step (seconds)
    /// @return Control output vector [vertical_thrust, roll_torque, pitch_torque, yaw_torque]
    Vector4f update_pos_hold(const Vector3f& pos_desired_ned,
                            const Vector3f& pos_actual_ned,
                            const Vector3f& vel_actual_ned,
                            const Vector3f& att_actual_rad,
                            const Vector3f& rate_actual_rads,
                            float dt);

    /// @brief Calculate LQR gains from Q/R matrices (online computation)
    /// @note Only used if params.lqr_mode == 1
    /// @return true if successful, false on error
    bool calculate_lqr_gains();

    /// @brief Get current gain matrix (for logging/debugging)
    /// @param gains Output array of 12 Vector4f (K stored column-major)
    void get_gains(Vector4f gains[12]) const;

    /// @brief Reset controller state (called on mode changes)
    void reset();

    /// @brief Check if gains are valid (non-zero, not NaN)
    /// @return true if gains are valid
    bool gains_valid() const;

private:
    /// Reference to parameter structure
    const AC_StateFeedback_Params& _params;

    /// Gain matrix K (4×12)
    /// K[row][col] maps state errors to control outputs
    /// Stored as array of 12 Vector4f (column-major for easier access)
    /// K[col] = [K[0,col], K[1,col], K[2,col], K[3,col]]
    Vector4f _K[12];

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
    Vector4f saturate_control(const Vector4f& control) const;

    /// @brief Normalize angle to [-π, π]
    /// @param angle Angle in radians
    /// @return Normalized angle
    float wrap_PI(float angle) const;
};
