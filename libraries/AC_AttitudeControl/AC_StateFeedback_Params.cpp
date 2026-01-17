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

#include "AC_StateFeedback_Params.h"

// ==========================================
// RATE LOOP PARAMETERS
// ==========================================

const AP_Param::GroupInfo AC_StateFeedback_RateParams::var_info[] = {
    // System dynamics (indices 1-6)
    // @Param: IXX
    // @DisplayName: Roll Moment of Inertia
    // @Description: Vehicle's moment of inertia about roll axis
    // @Units: kg.m^2
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("IXX", 1, AC_StateFeedback_RateParams, Ixx, 0.15f),

    // @Param: IYY
    // @DisplayName: Pitch Moment of Inertia
    // @Description: Vehicle's moment of inertia about pitch axis
    // @Units: kg.m^2
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("IYY", 2, AC_StateFeedback_RateParams, Iyy, 0.15f),

    // @Param: IZZ
    // @DisplayName: Yaw Moment of Inertia
    // @Description: Vehicle's moment of inertia about yaw axis
    // @Units: kg.m^2
    // @Range: 0.01 1.0
    // @User: Advanced
    AP_GROUPINFO("IZZ", 3, AC_StateFeedback_RateParams, Izz, 0.25f),

    // @Param: DX
    // @DisplayName: Roll Damping
    // @Description: Hydrodynamic damping on roll axis
    // @Units: Nm/(rad/s)
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("DX", 4, AC_StateFeedback_RateParams, Dx, 0.5f),

    // @Param: DY
    // @DisplayName: Pitch Damping
    // @Description: Hydrodynamic damping on pitch axis
    // @Units: Nm/(rad/s)
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("DY", 5, AC_StateFeedback_RateParams, Dy, 0.5f),

    // @Param: DZ
    // @DisplayName: Yaw Damping
    // @Description: Hydrodynamic damping on yaw axis
    // @Units: Nm/(rad/s)
    // @Range: 0.0 2.0
    // @User: Advanced
    AP_GROUPINFO("DZ", 6, AC_StateFeedback_RateParams, Dz, 0.3f),

    // Gain matrix (indices 7-15)
    AP_GROUPINFO("K1", 7, AC_StateFeedback_RateParams, K1, 3.162f),
    AP_GROUPINFO("K2", 8, AC_StateFeedback_RateParams, K2, 0.0f),
    AP_GROUPINFO("K3", 9, AC_StateFeedback_RateParams, K3, 0.0f),
    AP_GROUPINFO("K4", 10, AC_StateFeedback_RateParams, K4, 0.0f),
    AP_GROUPINFO("K5", 11, AC_StateFeedback_RateParams, K5, 3.162f),
    AP_GROUPINFO("K6", 12, AC_StateFeedback_RateParams, K6, 0.0f),
    AP_GROUPINFO("K7", 13, AC_StateFeedback_RateParams, K7, 0.0f),
    AP_GROUPINFO("K8", 14, AC_StateFeedback_RateParams, K8, 0.0f),
    AP_GROUPINFO("K9", 15, AC_StateFeedback_RateParams, K9, 2.236f),

    // LQR weights (indices 16-21)
    AP_GROUPINFO("Q1", 16, AC_StateFeedback_RateParams, Q1, 10.0f),
    AP_GROUPINFO("Q2", 17, AC_StateFeedback_RateParams, Q2, 10.0f),
    AP_GROUPINFO("Q3", 18, AC_StateFeedback_RateParams, Q3, 10.0f),
    AP_GROUPINFO("R1", 19, AC_StateFeedback_RateParams, R1, 1.0f),
    AP_GROUPINFO("R2", 20, AC_StateFeedback_RateParams, R2, 1.0f),
    AP_GROUPINFO("R3", 21, AC_StateFeedback_RateParams, R3, 1.0f),

    AP_GROUPEND
};

// ==========================================
// ATTITUDE LOOP PARAMETERS
// ==========================================

const AP_Param::GroupInfo AC_StateFeedback_AttitudeParams::var_info[] = {
    // Gain matrix (indices 1-18)
    AP_GROUPINFO("K1", 1, AC_StateFeedback_AttitudeParams, K1, 10.0f),
    AP_GROUPINFO("K2", 2, AC_StateFeedback_AttitudeParams, K2, 0.0f),
    AP_GROUPINFO("K3", 3, AC_StateFeedback_AttitudeParams, K3, 0.0f),
    AP_GROUPINFO("K4", 4, AC_StateFeedback_AttitudeParams, K4, 3.162f),
    AP_GROUPINFO("K5", 5, AC_StateFeedback_AttitudeParams, K5, 0.0f),
    AP_GROUPINFO("K6", 6, AC_StateFeedback_AttitudeParams, K6, 0.0f),
    AP_GROUPINFO("K7", 7, AC_StateFeedback_AttitudeParams, K7, 0.0f),
    AP_GROUPINFO("K8", 8, AC_StateFeedback_AttitudeParams, K8, 10.0f),
    AP_GROUPINFO("K9", 9, AC_StateFeedback_AttitudeParams, K9, 0.0f),
    AP_GROUPINFO("K10", 10, AC_StateFeedback_AttitudeParams, K10, 0.0f),
    AP_GROUPINFO("K11", 11, AC_StateFeedback_AttitudeParams, K11, 3.162f),
    AP_GROUPINFO("K12", 12, AC_StateFeedback_AttitudeParams, K12, 0.0f),
    AP_GROUPINFO("K13", 13, AC_StateFeedback_AttitudeParams, K13, 0.0f),
    AP_GROUPINFO("K14", 14, AC_StateFeedback_AttitudeParams, K14, 0.0f),
    AP_GROUPINFO("K15", 15, AC_StateFeedback_AttitudeParams, K15, 10.0f),
    AP_GROUPINFO("K16", 16, AC_StateFeedback_AttitudeParams, K16, 0.0f),
    AP_GROUPINFO("K17", 17, AC_StateFeedback_AttitudeParams, K17, 0.0f),
    AP_GROUPINFO("K18", 18, AC_StateFeedback_AttitudeParams, K18, 2.236f),

    // LQR weights (indices 19-27)
    AP_GROUPINFO("Q1", 19, AC_StateFeedback_AttitudeParams, Q1, 100.0f),
    AP_GROUPINFO("Q2", 20, AC_StateFeedback_AttitudeParams, Q2, 100.0f),
    AP_GROUPINFO("Q3", 21, AC_StateFeedback_AttitudeParams, Q3, 100.0f),
    AP_GROUPINFO("Q4", 22, AC_StateFeedback_AttitudeParams, Q4, 10.0f),
    AP_GROUPINFO("Q5", 23, AC_StateFeedback_AttitudeParams, Q5, 10.0f),
    AP_GROUPINFO("Q6", 24, AC_StateFeedback_AttitudeParams, Q6, 10.0f),
    AP_GROUPINFO("R1", 25, AC_StateFeedback_AttitudeParams, R1, 1.0f),
    AP_GROUPINFO("R2", 26, AC_StateFeedback_AttitudeParams, R2, 1.0f),
    AP_GROUPINFO("R3", 27, AC_StateFeedback_AttitudeParams, R3, 1.0f),

    AP_GROUPEND
};

// ==========================================
// POSITION LOOP PARAMETERS
// ==========================================

const AP_Param::GroupInfo AC_StateFeedback_PositionParams::var_info[] = {
    // System dynamics (indices 1-4)
    // @Param: MASS
    // @DisplayName: Vehicle Mass
    // @Description: Total vehicle mass including water displacement
    // @Units: kg
    // @Range: 1.0 50.0
    // @User: Advanced
    AP_GROUPINFO("MASS", 1, AC_StateFeedback_PositionParams, mass, 10.0f),

    // @Param: DX
    // @DisplayName: X-axis Damping
    // @Description: Translational damping coefficient for forward/back motion
    // @Units: N/(m/s)
    // @Range: 0.0 20.0
    // @User: Advanced
    AP_GROUPINFO("DX", 2, AC_StateFeedback_PositionParams, Dx, 5.0f),

    // @Param: DY
    // @DisplayName: Y-axis Damping
    // @Description: Translational damping coefficient for left/right motion
    // @Units: N/(m/s)
    // @Range: 0.0 20.0
    // @User: Advanced
    AP_GROUPINFO("DY", 3, AC_StateFeedback_PositionParams, Dy, 5.0f),

    // @Param: DZ
    // @DisplayName: Z-axis Damping
    // @Description: Translational damping coefficient for up/down motion
    // @Units: N/(m/s)
    // @Range: 0.0 20.0
    // @User: Advanced
    AP_GROUPINFO("DZ", 4, AC_StateFeedback_PositionParams, Dz, 5.0f),

    // Gain matrix (indices 5-52)
    AP_GROUPINFO("K1", 5, AC_StateFeedback_PositionParams, K1, 0.0f),
    AP_GROUPINFO("K2", 6, AC_StateFeedback_PositionParams, K2, 0.0f),
    AP_GROUPINFO("K3", 7, AC_StateFeedback_PositionParams, K3, 1.0f),
    AP_GROUPINFO("K4", 8, AC_StateFeedback_PositionParams, K4, 0.0f),
    AP_GROUPINFO("K5", 9, AC_StateFeedback_PositionParams, K5, 0.0f),
    AP_GROUPINFO("K6", 10, AC_StateFeedback_PositionParams, K6, 0.5f),
    AP_GROUPINFO("K7", 11, AC_StateFeedback_PositionParams, K7, 0.0f),
    AP_GROUPINFO("K8", 12, AC_StateFeedback_PositionParams, K8, 0.0f),
    AP_GROUPINFO("K9", 13, AC_StateFeedback_PositionParams, K9, 0.0f),
    AP_GROUPINFO("K10", 14, AC_StateFeedback_PositionParams, K10, 0.0f),
    AP_GROUPINFO("K11", 15, AC_StateFeedback_PositionParams, K11, 0.0f),
    AP_GROUPINFO("K12", 16, AC_StateFeedback_PositionParams, K12, 0.0f),
    AP_GROUPINFO("K13", 17, AC_StateFeedback_PositionParams, K13, 0.1f),
    AP_GROUPINFO("K14", 18, AC_StateFeedback_PositionParams, K14, 0.0f),
    AP_GROUPINFO("K15", 19, AC_StateFeedback_PositionParams, K15, 0.0f),
    AP_GROUPINFO("K16", 20, AC_StateFeedback_PositionParams, K16, 0.05f),
    AP_GROUPINFO("K17", 21, AC_StateFeedback_PositionParams, K17, 0.0f),
    AP_GROUPINFO("K18", 22, AC_StateFeedback_PositionParams, K18, 0.0f),
    AP_GROUPINFO("K19", 23, AC_StateFeedback_PositionParams, K19, 0.5f),
    AP_GROUPINFO("K20", 24, AC_StateFeedback_PositionParams, K20, 0.0f),
    AP_GROUPINFO("K21", 25, AC_StateFeedback_PositionParams, K21, 0.0f),
    AP_GROUPINFO("K22", 26, AC_StateFeedback_PositionParams, K22, 0.1f),
    AP_GROUPINFO("K23", 27, AC_StateFeedback_PositionParams, K23, 0.0f),
    AP_GROUPINFO("K24", 28, AC_StateFeedback_PositionParams, K24, 0.0f),
    AP_GROUPINFO("K25", 29, AC_StateFeedback_PositionParams, K25, 0.0f),
    AP_GROUPINFO("K26", 30, AC_StateFeedback_PositionParams, K26, 0.1f),
    AP_GROUPINFO("K27", 31, AC_StateFeedback_PositionParams, K27, 0.0f),
    AP_GROUPINFO("K28", 32, AC_StateFeedback_PositionParams, K28, 0.0f),
    AP_GROUPINFO("K29", 33, AC_StateFeedback_PositionParams, K29, 0.05f),
    AP_GROUPINFO("K30", 34, AC_StateFeedback_PositionParams, K30, 0.0f),
    AP_GROUPINFO("K31", 35, AC_StateFeedback_PositionParams, K31, 0.0f),
    AP_GROUPINFO("K32", 36, AC_StateFeedback_PositionParams, K32, 0.5f),
    AP_GROUPINFO("K33", 37, AC_StateFeedback_PositionParams, K33, 0.0f),
    AP_GROUPINFO("K34", 38, AC_StateFeedback_PositionParams, K34, 0.0f),
    AP_GROUPINFO("K35", 39, AC_StateFeedback_PositionParams, K35, 0.1f),
    AP_GROUPINFO("K36", 40, AC_StateFeedback_PositionParams, K36, 0.0f),
    AP_GROUPINFO("K37", 41, AC_StateFeedback_PositionParams, K37, 0.0f),
    AP_GROUPINFO("K38", 42, AC_StateFeedback_PositionParams, K38, 0.0f),
    AP_GROUPINFO("K39", 43, AC_StateFeedback_PositionParams, K39, 0.0f),
    AP_GROUPINFO("K40", 44, AC_StateFeedback_PositionParams, K40, 0.0f),
    AP_GROUPINFO("K41", 45, AC_StateFeedback_PositionParams, K41, 0.0f),
    AP_GROUPINFO("K42", 46, AC_StateFeedback_PositionParams, K42, 0.0f),
    AP_GROUPINFO("K43", 47, AC_StateFeedback_PositionParams, K43, 0.0f),
    AP_GROUPINFO("K44", 48, AC_StateFeedback_PositionParams, K44, 0.0f),
    AP_GROUPINFO("K45", 49, AC_StateFeedback_PositionParams, K45, 0.5f),
    AP_GROUPINFO("K46", 50, AC_StateFeedback_PositionParams, K46, 0.0f),
    AP_GROUPINFO("K47", 51, AC_StateFeedback_PositionParams, K47, 0.0f),
    AP_GROUPINFO("K48", 52, AC_StateFeedback_PositionParams, K48, 0.1f),

    // LQR weights (indices 53-63)
    AP_GROUPINFO("Q1", 53, AC_StateFeedback_PositionParams, Q1, 10.0f),
    AP_GROUPINFO("Q2", 54, AC_StateFeedback_PositionParams, Q2, 10.0f),
    AP_GROUPINFO("Q3", 55, AC_StateFeedback_PositionParams, Q3, 10.0f),
    AP_GROUPINFO("Q4", 56, AC_StateFeedback_PositionParams, Q4, 1.0f),
    AP_GROUPINFO("Q5", 57, AC_StateFeedback_PositionParams, Q5, 1.0f),
    AP_GROUPINFO("Q6", 58, AC_StateFeedback_PositionParams, Q6, 1.0f),
    AP_GROUPINFO("Q7", 59, AC_StateFeedback_PositionParams, Q7, 100.0f),
    AP_GROUPINFO("Q8", 60, AC_StateFeedback_PositionParams, Q8, 100.0f),
    AP_GROUPINFO("Q9", 61, AC_StateFeedback_PositionParams, Q9, 100.0f),
    AP_GROUPINFO("Q10", 62, AC_StateFeedback_PositionParams, Q10, 10.0f),
    AP_GROUPINFO("Q11", 63, AC_StateFeedback_PositionParams, Q11, 10.0f),
    // Note: Q12 and R weights omitted to stay within 63-parameter limit, use offline tool

    AP_GROUPEND
};

// ==========================================
// MASTER PARAMETER CLASS
// ==========================================

// Constructor
AC_StateFeedback_Params::AC_StateFeedback_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Master parameter table
const AP_Param::GroupInfo AC_StateFeedback_Params::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: State Feedback Control Enable
    // @Description: Enables state feedback control. 0=PID only, 1=Rate loop, 2=Rate+Attitude, 3=All loops
    // @Values: 0:PID Only, 1:Rate Loop, 2:Rate+Attitude, 3:All Loops
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AC_StateFeedback_Params, enable, 0),

    // @Param: LQR_MODE
    // @DisplayName: LQR Gain Calculation Mode
    // @Description: 0=Use pre-calculated gains, 1=Calculate online (experimental)
    // @Values: 0:Pre-calculated, 1:Online Calculation
    // @User: Advanced
    AP_GROUPINFO("LQR_MODE", 2, AC_StateFeedback_Params, lqr_mode, 0),

    // @Group: R_
    // @Path: AC_StateFeedback_Params.cpp
    AP_SUBGROUPINFO(rate, "R_", 3, AC_StateFeedback_Params, AC_StateFeedback_RateParams),

    // @Group: A_
    // @Path: AC_StateFeedback_Params.cpp
    AP_SUBGROUPINFO(attitude, "A_", 4, AC_StateFeedback_Params, AC_StateFeedback_AttitudeParams),

    // @Group: P_
    // @Path: AC_StateFeedback_Params.cpp
    AP_SUBGROUPINFO(position, "P_", 5, AC_StateFeedback_Params, AC_StateFeedback_PositionParams),

    AP_GROUPEND
};
