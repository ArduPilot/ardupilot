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

/// @file    AC_StateFeedback_Params.h
/// @brief   State Feedback Control Parameters for ArduSub

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// ==========================================
// RATE LOOP PARAMETERS (23 parameters)
// ==========================================

/// @class AC_StateFeedback_RateParams
/// @brief Rate loop parameters (3 states: roll_rate, pitch_rate, yaw_rate)
class AC_StateFeedback_RateParams {
public:
    static const struct AP_Param::GroupInfo var_info[];

    // System dynamics parameters (6)
    AP_Float Ixx;         ///< Roll moment of inertia (kg·m²)
    AP_Float Iyy;         ///< Pitch moment of inertia (kg·m²)
    AP_Float Izz;         ///< Yaw moment of inertia (kg·m²)
    AP_Float Dx;          ///< Roll damping coefficient (Nm/(rad/s))
    AP_Float Dy;          ///< Pitch damping coefficient (Nm/(rad/s))
    AP_Float Dz;          ///< Yaw damping coefficient (Nm/(rad/s))

    // Gain matrix K (3×3, 9 parameters)
    AP_Float K1, K2, K3;   ///< Row 0: Roll control gains
    AP_Float K4, K5, K6;   ///< Row 1: Pitch control gains
    AP_Float K7, K8, K9;   ///< Row 2: Yaw control gains

    // LQR weights (6 parameters, optional)
    AP_Float Q1, Q2, Q3;   ///< State weights [roll_rate, pitch_rate, yaw_rate]
    AP_Float R1, R2, R3;   ///< Control weights [roll, pitch, yaw]
};

// ==========================================
// ATTITUDE LOOP PARAMETERS (27 parameters)
// ==========================================

/// @class AC_StateFeedback_AttitudeParams
/// @brief Attitude loop parameters (6 states: roll, pitch, yaw, p, q, r)
class AC_StateFeedback_AttitudeParams {
public:
    static const struct AP_Param::GroupInfo var_info[];

    // Gain matrix K (3×6, 18 parameters)
    AP_Float K1, K2, K3, K4, K5, K6;       ///< Row 0: Roll control gains
    AP_Float K7, K8, K9, K10, K11, K12;    ///< Row 1: Pitch control gains
    AP_Float K13, K14, K15, K16, K17, K18; ///< Row 2: Yaw control gains

    // LQR weights (9 parameters, optional)
    AP_Float Q1, Q2, Q3, Q4, Q5, Q6;       ///< State weights [roll, pitch, yaw, p, q, r]
    AP_Float R1, R2, R3;                    ///< Control weights [roll, pitch, yaw]
};

// ==========================================
// POSITION LOOP PARAMETERS (36 parameters)
// ==========================================

/// @class AC_StateFeedback_PositionParams
/// @brief Position loop parameters (12 states: x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r)
class AC_StateFeedback_PositionParams {
public:
    static const struct AP_Param::GroupInfo var_info[];

    // System dynamics parameters (4)
    AP_Float mass;        ///< Vehicle mass (kg)
    AP_Float Dx;          ///< Translational damping X (N/(m/s))
    AP_Float Dy;          ///< Translational damping Y (N/(m/s))
    AP_Float Dz;          ///< Translational damping Z (N/(m/s))

    // Gain matrix K (4×12, 48 parameters) - Control: [Tz, τ_roll, τ_pitch, τ_yaw]
    AP_Float K1, K2, K3, K4, K5, K6, K7, K8, K9, K10, K11, K12;         ///< Row 0: Vertical thrust
    AP_Float K13, K14, K15, K16, K17, K18, K19, K20, K21, K22, K23, K24; ///< Row 1: Roll torque
    AP_Float K25, K26, K27, K28, K29, K30, K31, K32, K33, K34, K35, K36; ///< Row 2: Pitch torque
    AP_Float K37, K38, K39, K40, K41, K42, K43, K44, K45, K46, K47, K48; ///< Row 3: Yaw torque

    // LQR weights (11 parameters, optional)
    // Note: Q12 and R weights omitted to stay within 63-parameter limit, use offline LQR tool
    AP_Float Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8, Q9, Q10, Q11;  ///< State weights
};

// ==========================================
// MASTER PARAMETERS
// ==========================================

/// @class AC_StateFeedback_Params
/// @brief Master parameter class with nested subgroups
///
/// This class manages all parameters for the state-space feedback control system.
/// It uses nested subgroups to stay within ArduPilot's 64-parameter-per-group limit.
///
/// The enable parameter (SF_ENABLE) determines which loops use state feedback:
///   0 = PID only (default, backward compatible)
///   1 = State feedback rate loop only
///   2 = State feedback rate + attitude loops
///   3 = State feedback all loops (rate + attitude + position)
class AC_StateFeedback_Params {
public:
    AC_StateFeedback_Params(void);

    // Do not allow copies
    CLASS_NO_COPY(AC_StateFeedback_Params);

    // Parameter table
    static const struct AP_Param::GroupInfo var_info[];

    /// @brief Master enable switch for state feedback control
    /// 0 = PID only (default)
    /// 1 = State feedback rate loop
    /// 2 = State feedback rate + attitude
    /// 3 = State feedback all loops
    AP_Int8 enable;

    /// @brief LQR gain calculation mode
    /// 0 = Use pre-calculated gains (K parameters, default)
    /// 1 = Calculate gains online from Q/R matrices at startup
    AP_Int8 lqr_mode;

    /// @brief Rate loop parameters (23 parameters)
    AC_StateFeedback_RateParams rate;

    /// @brief Attitude loop parameters (27 parameters)
    AC_StateFeedback_AttitudeParams attitude;

    /// @brief Position loop parameters (36 parameters)
    AC_StateFeedback_PositionParams position;
};
