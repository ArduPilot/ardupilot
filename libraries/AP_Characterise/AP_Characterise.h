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

#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixRC.h>

#define N_KF_st 11 // number of KF states
#define N_KF_eq 3  // number of KF equations/predictions

class AP_Characterise
{
public:
    AP_Characterise();

    /* Do not allow copies */
    AP_Characterise(const AP_Characterise &other) = delete;
    AP_Characterise &operator=(const AP_Characterise&) = delete;

    static AP_Characterise *get_singleton();

    // return true if characterise is enabled
    bool enabled() const;

    // pre-arm check we have everything we need
    bool prearm_healthy();

    // get the reason for a prearm failure
    char *prearm_failure_reason() {return prearm_fail_string;};

    // read in and average inputs
    void update_inputs();

    // run a KF update
    void update();

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 _enable;
    AP_Float _s;                  // wing area (m)
    AP_Float _mass;               // vehicle mass (kg)
    AP_Float _d;                  // propeller diameter (m)
    AP_Int8 _num_props;           // number of propellers
    AP_Int32 _prop_rpm_bitmask;   // bitmask of rpm sensors to use
    AP_Int32 _prop_power_bitmask; // bitmask of power sensors to use
    // measurement noise parameters
    AP_Float _R_force_x;
    AP_Float _R_force_z;
    AP_Float _R_power;
    // process noise paramiters
    AP_Float _Q_CL0;
    AP_Float _Q_CLa;
    AP_Float _Q_CD0;
    AP_Float _Q_CDa;
    AP_Float _Q_CDaa;
    AP_Float _Q_Ct0;
    AP_Float _Q_Ctj;
    AP_Float _Q_Ctjj;
    AP_Float _Q_Cp0;
    AP_Float _Q_Cpj;
    AP_Float _Q_Cpjj;

    // variables for averaging inputs
    float _AoA_mean;
    float _V_mean;
    float _rho_mean;
    float _rpm_mean;
    float _accel_x_mean;
    float _accel_z_mean;
    float _power_mean;
    uint16_t _samples;
    uint16_t _rpm_samples;
    uint16_t _power_samples;

    // string representing last reason for prearm failure
    char prearm_fail_string[42];

    // read inputs from ESC telem and battery monitor
    void read_esc_telem();
    void read_bat_mon();

    // KF varables
    VectorN<float,N_KF_st>   _x;         // KF states, aerodynamic coefficients
    MatrixN<float,N_KF_st>   _P{100.0f}; // KF state covariance matrix, initially a diagonal with values of 100

    static AP_Characterise *_singleton;
};

namespace AP {
    AP_Characterise *characterise();
};
