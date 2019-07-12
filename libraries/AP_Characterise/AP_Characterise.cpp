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

#include "AP_Characterise.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Characterise::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Aerodynamic Characterization
    // @Description: Enable Aerodynamic Characterization
    // @Values: 0:Disabled, 1:Enabled read only, 2:Enabled KF learning
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Characterise, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: WING_AREA
    // @DisplayName: Reference wing area for lift and drag coefficients
    // @Description: reference area using in lift and drag calculation, changing this will require coefficients to be re-learned
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("WING_AREA", 2, AP_Characterise, _s, 1.0f),

    // @Param: MASS
    // @DisplayName: Vehicle mass
    // @Description: vehicle mass (in KG) used to convert from force to acceleration
    // @User: Standard
    AP_GROUPINFO("MASS", 3, AP_Characterise, _mass, 1.0f),

    // @Param: PROP_D
    // @DisplayName: Propeller diameter
    // @Description: propeller reference diameter, used in thrust coefficient calculations, changing this will require coefficients to be re-learned
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("PROP_D", 4, AP_Characterise, _d, 0.2f),

    // @Param: PROP_NO
    // @DisplayName: Number of propellers
    // @Description: Number of propellers used in forward flight, it is assumed all propellers are identical
    // @Increment: 1
    // @Range: 1 2
    // @User: Standard
    AP_GROUPINFO("PROP_NUM", 5, AP_Characterise, _num_props, 1),

    // @Param: PROP_RPM
    // @DisplayName: propeller rpm source
    // @Description: sensor used as source for RPM value for KF, if multiple are selected the average will be taken
    // @Bitmask: 0:ESC1 telem, 1:ESC2 telem, 2:ESC3 telem, 3:ESC4 telem, 4:ESC5 telem, 5:ESC6 telem, 6:ESC7 telem, 7:ESC8 telem, 8:RPM1, 9:RPM2
    // @User: Standard
    AP_GROUPINFO("PROP_RPM", 6, AP_Characterise, _prop_rpm_bitmask, 0),

    // @Param: PROP_PWR
    // @DisplayName: propeller power source
    // @Description: sensor used as source for power value for KF, if multiple are selected the average will be taken
    // @Bitmask: 0:ESC1 telem, 1:ESC2 telem, 2:ESC3 telem, 3:ESC4 telem, 4:ESC5 telem, 5:ESC6 telem, 6:ESC7 telem, 7:ESC8 telem, 8:Bat mon, 9:Bat mon 1, 10:Bat mon 2, 11:Bat mon 3, 12:Bat mon 4, 13:Bat mon 5, 14:Bat mon 6, 15:Bat mon 7, 16:Bat mon 8
    // @User: Standard
    AP_GROUPINFO("PROP_PWR", 7, AP_Characterise, _prop_power_bitmask, 0),

    // @Param: R_FORCE_X
    // @DisplayName: Force X measurement noise
    // @Description: measurement noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("R_FORCE_X", 8, AP_Characterise, _R_force_x, 1),

    // @Param: R_FORCE_Z
    // @DisplayName: Force Z measurement noise
    // @Description: measurement noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("R_FORCE_Z", 9, AP_Characterise, _R_force_z, 1),

    // @Param: R_POWER
    // @DisplayName: power measurement noise
    // @Description: measurement noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("R_POWER", 10, AP_Characterise, _R_power, 1),

    // @Param: Q_CL0
    // @DisplayName: CL0 process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CL0", 11, AP_Characterise, _Q_CL0, 1),

    // @Param: Q_CLA
    // @DisplayName: CLa process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CLA", 12, AP_Characterise, _Q_CLa, 1),

    // @Param: Q_CD0
    // @DisplayName: CD0 process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CD0", 13, AP_Characterise, _Q_CD0, 1),

    // @Param: Q_CDA
    // @DisplayName: CDa process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CDA", 14, AP_Characterise, _Q_CDa, 1),

    // @Param: Q_CDAA
    // @DisplayName: CL0 process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CDAA", 15, AP_Characterise, _Q_CDaa, 1),

    // @Param: Q_CT0
    // @DisplayName: Ct0 process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CT0", 16, AP_Characterise, _Q_Ct0, 1),

    // @Param: Q_CTJ
    // @DisplayName: Ctj process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CTJ", 17, AP_Characterise, _Q_Ctj, 1),

    // @Param: Q_CTJJ
    // @DisplayName: Ctjj process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CTJJ", 18, AP_Characterise, _Q_Ctjj, 1),

    // @Param: Q_CP0
    // @DisplayName: Cp0 process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CP0", 19, AP_Characterise, _Q_Cp0, 1),

    // @Param: Q_CPJ
    // @DisplayName: Cpj process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CPJ", 20, AP_Characterise, _Q_Cpj, 1),

    // @Param: Q_CPJJ
    // @DisplayName: Cpjj process noise
    // @Description: process noise variance used in KF calculations
    // @User: Standard
    AP_GROUPINFO("Q_CPJJ", 21, AP_Characterise, _Q_Cpjj, 1),

    AP_GROUPEND};

// constructor
AP_Characterise::AP_Characterise()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many AP_Characterise instances");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_Characterise singleton
 */
AP_Characterise *AP_Characterise::get_singleton()
{
    return _singleton;
}

// pre-arm check we have everything we need
bool AP_Characterise::prearm_healthy()
{
    // reload encase we previously set it to something else
    _enable.load();

    if (_enable != 2) {
        return true;
    }

    // if we are enabled for learning then we must have access to the appropriate inputs
    if (!is_positive(_s)) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Must set wing area for characterization");
        _enable.set(1); // disable for the time being
        return false;
    }

    if (!is_positive(_mass)) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                          "Must set mass for characterization");
        _enable.set(1);
        return false;
    }

    if (!is_positive(_d)) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Must set prop dia for characterization");
        _enable.set(1);
        return false;
    }

    if (_prop_rpm_bitmask == 0) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Must set rpm sensor characterization");
        _enable.set(1);
        return false;
    }

    if (_prop_power_bitmask == 0) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Must set power sensor for characterization");
        _enable.set(1);
        return false;
    }

    return true;
}

// update KF inputs, this reads in values that are required for the prediction, expected to be called at 400hz
void AP_Characterise::update_inputs()
{
    if (_enable != 2) {
        return;
    }

    const float AoA = AP::ahrs().getAOA();
    _AoA_mean = ((_AoA_mean*_samples) + AoA) / (_samples + 1);

    float Airspeed;
    const bool have_airspeed = AP::ahrs().airspeed_estimate(&Airspeed);
    if (have_airspeed) {
        _V_mean = ((_V_mean*_samples) + Airspeed) / (_samples + 1);
    }

    AP_Baro *baro = AP_Baro::get_singleton();
    if (baro != nullptr) {
        // convert from density ratio to density
        const float rho = baro->get_air_density_ratio() * SSL_AIR_DENSITY;
        _rho_mean = ((_rho_mean*_samples) + rho) / (_samples + 1);
    }

    // body frame accelerations
    Vector3f bf_accel = AP::ahrs().get_accel_ef_blended();
    bf_accel.z += GRAVITY_MSS;      // remove gravity
    const Matrix3f &rot = AP::ahrs().get_rotation_body_to_ned();
    bf_accel = rot.mul_transpose(bf_accel);
    _accel_x_mean = ((_accel_x_mean*_samples) + bf_accel.x) / (_samples + 1);
    _accel_z_mean = ((_accel_z_mean*_samples) + bf_accel.z) / (_samples + 1);

    // Zero rpm reading if throttle is zero
    const bool have_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > 0;

    // read in power and rpm from ESC's
    read_esc_telem(have_throttle);

    // read RPM1 and RPM2
    const AP_RPM *rpm = AP_RPM::get_singleton();
    if (rpm != nullptr) {
        for (uint8_t i=0; i<2; i++) {
            if ((_prop_rpm_bitmask & (1 << (8+i))) != 0) {
                if (have_throttle) {
                    _rpm_mean = ((_rpm_mean*_rpm_samples) + rpm->get_rpm(i)) / (_rpm_samples + 1);
                } else {
                    _rpm_mean = (_rpm_mean*_rpm_samples) / (_rpm_samples + 1);
                }
                _rpm_samples++;
            }
        }
    }

    // read in power from battery monitor's
    read_bat_mon();

    _samples++;
}

// read in power and rpm from blheli esc
void AP_Characterise::read_esc_telem(const bool have_throttle)
{
#ifdef HAVE_AP_BLHELI_SUPPORT
    AP_BLHeli *blheli = AP_BLHeli::get_singleton();
    if (!blheli) {
        return;
    }

    for (uint8_t i=0; i<AP_BLHELI_MAX_ESCS; i++) {
        bool use_rpm = (_prop_rpm_bitmask & (1 << i)) != 0;
        bool use_power = (_prop_power_bitmask & (1 << i)) != 0;

        if (!use_rpm && !use_power) {
            continue;
        }

        AP_BLHeli::telem_data td;
        if (!blheli->get_telem_data(i, td)) {
            continue;
        }

        if (use_rpm) {
            if (have_throttle) {
                _rpm_mean = ((_rpm_mean*_rpm_samples) + td.rpm) / (_rpm_samples + 1);
            } else {
                _rpm_mean = (_rpm_mean*_rpm_samples) / (_rpm_samples + 1);
            }
            _rpm_samples++;
        }

        if (use_power) {
            // convert from cV and cI
            _power_mean = ((_power_mean*_power_samples) + (td.voltage * td.current * 0.0001)) / (_power_samples + 1);
            _power_samples++;
        }
    }
#endif
}

// read in power from battery monitor
void AP_Characterise::read_bat_mon()
{
    const AP_BattMonitor *bat = AP_BattMonitor::get_singleton();
    if (!bat) {
        return;
    }

    for (uint8_t i=0; i<AP_BATT_MONITOR_MAX_INSTANCES; i++) {
        if ((_prop_power_bitmask & (1 << (8+i))) == 0){
            continue;
        }

        _power_mean = ((_power_mean*_power_samples) + (bat->current_amps(i) * bat->voltage(i))) / (_power_samples + 1);
        _power_samples++;
    }
}

// update the characterization KF, expected to be called at 10hz
void AP_Characterise::update()
{
    if (!hal.util->get_soft_armed() || AP::ahrs().get_time_flying_ms() < 10000) {
        // only learn when flying and with enough time to be clear of
        // the ground
        return;
    }

    // define variables
    VectorN<float,N_KF_eq>            R;
    VectorN<float,N_KF_st>            Q;
    MatrixRC<float,N_KF_eq,N_KF_st>   H;
    VectorN<float,N_KF_eq>            prediction;
    VectorN<float,N_KF_eq>            truth;
    VectorN<float,N_KF_eq>            innovation;
    MatrixRC<float,N_KF_eq,N_KF_st>   temp;
    MatrixN<float,N_KF_eq>            S;
    MatrixRC<float,N_KF_st,N_KF_eq>   KG;
    MatrixN<float,N_KF_st>            temp1;
    VectorN<float,N_KF_st>            delta_x;

    // build R and Q from parameters
    R[0] =  powf(_R_force_x,2);
    R[1] =  powf(_R_force_z,2);
    R[2] =  powf(_R_power,2);
    Q[0] =  powf(_Q_CL0,2);
    Q[1] =  powf(_Q_CLa,2);
    Q[2] =  powf(_Q_CD0,2);
    Q[3] =  powf(_Q_CDa,2);
    Q[4] =  powf(_Q_CDaa,2);
    Q[5] =  powf(_Q_Ct0,2);
    Q[6] =  powf(_Q_Ctj,2);
    Q[7] =  powf(_Q_Ctjj,2);
    Q[8] =  powf(_Q_Cp0,2);
    Q[9] =  powf(_Q_Cpj,2);
    Q[10] = powf(_Q_Cpjj,2);

    // temp to stop divide by zero
    _rpm_mean = MAX(_rpm_mean,0.1f);

    // calculate the known values
    const float AoA = _AoA_mean;                             // angle of attack (deg)
    const float q = 0.5f * _rho_mean * _V_mean * _V_mean;    // dynamic pressure (kg/m^2)
    const float w = _rpm_mean * (1.0f/60.0f);                // prop rotation speed (Hz)
    float       J = _V_mean / (w * _d);                      // prop advance ratio
 
    int8_t effective_num_props = _num_props;
    if (J > 4.0f) {
        // large advance ratio means low prop rpm so assume zero
        // thrust and don't update states related to the prop
        effective_num_props = 0;
        // limit J to prevent infs and nans in the matrix's
        J = 4.0f;
    }

    // gather together some common terms to tidy equations
    const float sin_q_s   = sinf(radians(AoA))*q*_s;
    const float cos_q_s   = cosf(radians(AoA))*q*_s;
    const float rho_w2_d4 = _rho_mean*powf(w,2)*powf(_d,4)*effective_num_props;
    const float rho_w3_d5 = _rho_mean*powf(w,3)*powf(_d,5)*effective_num_props;

    // X force (thrust - drag, positive forwards) is sum of..
    H.set(0,0,sin_q_s);          // * Cl0
    H.set(0,1,sin_q_s*AoA);      // * Cla
    H.set(0,2,-cos_q_s);         // * Cd0
    H.set(0,3,-cos_q_s*AoA);     // * Cda
    H.set(0,4,-cos_q_s*AoA*AoA); // * Cdaa
    H.set(0,5,rho_w2_d4);        // * Ct0
    H.set(0,6,rho_w2_d4*J);      // * Ctj
    H.set(0,7,rho_w2_d4*J*J);    // * Ctjj

    // Z force (lift, positive down) is sum of..
    H.set(1,0,-cos_q_s);         // * Cl0
    H.set(1,1,-cos_q_s*AoA);     // * Cla
    H.set(1,2,-sin_q_s);         // * Cd0
    H.set(1,3,-sin_q_s*AoA);     // * Cda
    H.set(1,4,-sin_q_s*AoA*AoA); // * Cdaa

    // propeller power is sum of..
    H.set(2,8,rho_w3_d5);        // * CP0
    H.set(2,9,rho_w3_d5*J);      // * CPj
    H.set(2,10,rho_w3_d5*J*J);   // * CPjj

    // Calculate the predicted forces and power from the above equations
    prediction.mult(H,_x);

    // the true values as measured
    truth[0] = (_accel_x_mean * _mass);
    truth[1] = (_accel_z_mean * _mass);
    truth[2] = _power_mean;

    // Calculate the error between actual and predicted (this is called innovation in a KF)
    innovation = truth - prediction;

    // Calculate the Kalman gain
    temp = H*_P;                          // H*P
    S.mult_trans(temp,H);                 // H*P*H'
    S.diag_add(R);                        // S = H*P*H' + R
    KG.trans_mult(temp,S.inverse_matN()); // (P*H')/S = (H*P)'/S = (H*P)'*inv(S)

    if (effective_num_props == 0){
        // ignore prop effects
        for (uint8_t i=5; i<11; i++) {
            KG.set(i,0,0.0f);
            KG.set(i,1,0.0f);
            KG.set(i,2,0.0f);
            Q[i] = 0.0f;
        }
    }

    // Update the state covariance, note there is no multiplication of P as we are expecting states to be constant
    // P = A*P*A' if A is identity, add process noise Q, ensure P remains symmetric
    temp1.mult_trans(KG*S,KG); // KG*S*KG'
    if (temp1.is_safe()) {     // check inf and nan before updating covariance
        _P -= temp1;           // P = A*P*A' - KG*S*KG'
        _P.force_symmetry();
    }
    _P.diag_add(Q);            // P = A*P*A' - KG*S*KG' + Q;

    // Update state prediction
    delta_x = KG*innovation;
    // check for inf or nan before we update the states
    if (delta_x.is_safe()) {
        _x += delta_x;
    }

    // constrain states by assuming some facts about the aircraft
    _x[1]  = MAX(_x[1],0.0f);  // Lift must increase with angle of attack
    _x[2]  = MAX(_x[2],0.0f);  // Cd 0 must be greater than zero
    _x[4]  = MAX(_x[4],0.0f);  // Drag must increase with angle of attack squared
    _x[5]  = MAX(_x[5],0.0f);  // Must have some static thrust
    _x[7]  = MIN(_x[7],0.0f);  // thrust must decrease with advance ratio squared
    _x[8]  = MAX(_x[8],0.0f);  // Must use some power for static thrust
    _x[10] = MIN(_x[10],0.0f); // Power must decrease with advance ratio squared

    // do some logging
    const uint64_t time_us = AP_HAL::micros64();
    const struct log_aero_KF_input log1{
        LOG_PACKET_HEADER_INIT(LOG_AERO_KF_MSG1),
        time_us : time_us,
        aoa     : _AoA_mean,      // Angle of attack (deg)
        air_spd : _V_mean,        // Airspeed (m/s)
        rho     : _rho_mean,      // pressure (kg/m^2)
        rpm     : _rpm_mean,      // prop rpm (converted to rps in KF)
        accel_x : _accel_x_mean,  // acceleration x (m/s^2)
        accel_z : _accel_z_mean,  // acceleration z (m/s^2)
        power   : _power_mean,    // power (w)
    };

    const struct log_aero_KF_innovation log2{
        LOG_PACKET_HEADER_INIT(LOG_AERO_KF_MSG2),
        time_us            : time_us,
        calc_force_x       : prediction[0],  // calculated force x (N)
        calc_force_z       : prediction[1],  // calculated force z (N)
        calc_power         : prediction[2],  // calculated power (w)
        true_force_x       : truth[0],       // measured force x (N)
        true_force_z       : truth[1],       // measured force z (N)
        true_power         : truth[2],       // measured power (w)
        force_x_innovation : innovation[0],  // force innovation x (N)
        force_z_innovation : innovation[1],  // force innovation Z (N)
        power_innovation   : innovation[2],  // power innovation (w)
    };

    const struct log_aero_KF_state log3{
        LOG_PACKET_HEADER_INIT(LOG_AERO_KF_MSG3),
        time_us  : time_us,
        Cl0      : _x[0],   // Cl0
        Cla      : _x[1],   // Cla
        Cd0      : _x[2],   // Cd0
        Cda      : _x[3],   // Cda
        Cdaa     : _x[4],   // Cdaa
        Ct0      : _x[5],   // Ct0
        Ctj      : _x[6],   // Ctj
        Ctjj     : _x[7],   // Ctjj
        CP0      : _x[8],   // CP0
        CPj      : _x[9],   // CPj
        CPjj     : _x[10],  // CPjj
    };

    const struct log_aero_KF_covariance log4{
        LOG_PACKET_HEADER_INIT(LOG_AERO_KF_MSG4),
        time_us      : time_us,
        Cl0_cov      : _P.get(0,0),   // Cl0
        Cla_cov      : _P.get(1,1),   // Cla
        Cd0_cov      : _P.get(2,2),   // Cd0
        Cda_cov      : _P.get(3,3),   // Cda
        Cdaa_cov     : _P.get(4,4),   // Cdaa
        Ct0_cov      : _P.get(5,5),   // Ct0
        Ctj_cov      : _P.get(6,6),   // Ctj
        Ctjj_cov     : _P.get(7,7),   // Ctjj
        CP0_cov      : _P.get(8,8),   // CP0
        CPj_cov      : _P.get(9,9),   // CPj
        CPjj_cov     : _P.get(10,10), // CPjj
    };

    AP::logger().WriteBlock(&log1, sizeof(log1));
    AP::logger().WriteBlock(&log2, sizeof(log2));
    AP::logger().WriteBlock(&log3, sizeof(log3));
    AP::logger().WriteBlock(&log4, sizeof(log4));

    // reset number of samples for mean values
    _samples = 0;
    _rpm_samples = 0;
    _power_samples = 0;

}

AP_Characterise *AP_Characterise::_singleton = nullptr;

namespace AP {
    AP_Characterise * characterise()
    {
        return AP_Characterise::get_singleton();
    }
};
