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

/*
    SITL.cpp - software in the loop state
*/

#include "SITL.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

SITL *SITL::_s_instance = nullptr;

// table of user settable parameters
const AP_Param::GroupInfo SITL::var_info[] = {
    AP_GROUPINFO("BARO_RND",   0, SITL,  baro_noise,  0.2f),
    AP_GROUPINFO("GYR_RND",    1, SITL,  gyro_noise,  0),
    AP_GROUPINFO("ACC_RND",    2, SITL,  accel_noise, 0),
    AP_GROUPINFO("MAG_RND",    3, SITL,  mag_noise,   0),
    AP_GROUPINFO("GPS_DISABLE",4, SITL,  gps_disable, 0),
    AP_GROUPINFO("DRIFT_SPEED",5, SITL,  drift_speed, 0.05f),
    AP_GROUPINFO("DRIFT_TIME", 6, SITL,  drift_time,  5),
    AP_GROUPINFO("GPS_DELAY",  7, SITL,  gps_delay,   1),
    AP_GROUPINFO("ENGINE_MUL", 8, SITL,  engine_mul,  1),
    AP_GROUPINFO("WIND_SPD",   9, SITL,  wind_speed,  0),
    AP_GROUPINFO("WIND_DIR",  10, SITL,  wind_direction,  180),
    AP_GROUPINFO("WIND_TURB", 11, SITL,  wind_turbulance,  0),
    AP_GROUPINFO("GPS_TYPE",  12, SITL,  gps_type,  SITL::GPS_TYPE_UBLOX),
    AP_GROUPINFO("GPS_BYTELOSS",  13, SITL,  gps_byteloss,  0),
    AP_GROUPINFO("GPS_NUMSATS",   14, SITL,  gps_numsats,   10),
    AP_GROUPINFO("MAG_ERROR",     15, SITL,  mag_error,  0),
    AP_GROUPINFO("SERVO_SPEED",   16, SITL,  servo_speed,  0.14),
    AP_GROUPINFO("GPS_GLITCH",    17, SITL,  gps_glitch,  0),
    AP_GROUPINFO("GPS_HZ",        18, SITL,  gps_hertz,  5),
    AP_GROUPINFO("BATT_VOLTAGE",  19, SITL,  batt_voltage,  12.6f),
    AP_GROUPINFO("ARSPD_RND",     20, SITL,  arspd_noise,  0.5f),
    AP_GROUPINFO("ACCEL_FAIL",    21, SITL,  accel_fail,  0),
    AP_GROUPINFO("BARO_DRIFT",    22, SITL,  baro_drift,  0),
    AP_GROUPINFO("SONAR_GLITCH",  23, SITL,  sonar_glitch, 0),
    AP_GROUPINFO("SONAR_RND",     24, SITL,  sonar_noise, 0),
    AP_GROUPINFO("RC_FAIL",       25, SITL,  rc_fail, 0),
    AP_GROUPINFO("GPS2_ENABLE",   26, SITL,  gps2_enable, 0),
    AP_GROUPINFO("BARO_DISABLE",  27, SITL,  baro_disable, 0),
    AP_GROUPINFO("FLOAT_EXCEPT",  28, SITL,  float_exception, 1),
    AP_GROUPINFO("MAG_MOT",       29, SITL,  mag_mot, 0),
    AP_GROUPINFO("ACC_BIAS",      30, SITL,  accel_bias, 0),
    AP_GROUPINFO("BARO_GLITCH",   31, SITL,  baro_glitch, 0),
    AP_GROUPINFO("SONAR_SCALE",   32, SITL,  sonar_scale, 12.1212f),
    AP_GROUPINFO("FLOW_ENABLE",   33, SITL,  flow_enable, 0),
    AP_GROUPINFO("TERRAIN",       34, SITL,  terrain_enable, 1),
    AP_GROUPINFO("FLOW_RATE",     35, SITL,  flow_rate, 10),
    AP_GROUPINFO("FLOW_DELAY",    36, SITL,  flow_delay, 0),
    AP_GROUPINFO("GPS_DRIFTALT",  37, SITL,  gps_drift_alt, 0),
    AP_GROUPINFO("BARO_DELAY",    38, SITL,  baro_delay, 0),
    AP_GROUPINFO("MAG_DELAY",     39, SITL,  mag_delay, 0),
    AP_GROUPINFO("WIND_DELAY",    40, SITL,  wind_delay, 0),
    AP_GROUPINFO("MAG_OFS",       41, SITL,  mag_ofs, 0),
    AP_GROUPINFO("ACC2_RND",      42, SITL,  accel2_noise, 0),
    AP_GROUPINFO("ARSPD_FAIL",    43, SITL,  arspd_fail, 0),
    AP_GROUPINFO("GYR_SCALE",     44, SITL,  gyro_scale, 0),
    AP_GROUPINFO("ADSB_COUNT",    45, SITL,  adsb_plane_count, -1),
    AP_GROUPINFO("ADSB_RADIUS",   46, SITL,  adsb_radius_m, 10000),
    AP_GROUPINFO("ADSB_ALT",      47, SITL,  adsb_altitude_m, 1000),
    AP_GROUPINFO("MAG_ALY",       48, SITL,  mag_anomaly_ned, 0),
    AP_GROUPINFO("MAG_ALY_HGT",   49, SITL,  mag_anomaly_hgt, 1.0f),
    AP_GROUPINFO("PIN_MASK",      50, SITL,  pin_mask, 0),
    AP_GROUPINFO("ADSB_TX",       51, SITL,  adsb_tx, 0),
    AP_GROUPINFO("SPEEDUP",       52, SITL,  speedup, -1),
    AP_GROUPINFO("IMU_POS",       53, SITL,  imu_pos_offset, 0),
    AP_GROUPINFO("GPS_POS",       54, SITL,  gps_pos_offset, 0),
    AP_GROUPINFO("SONAR_POS",     55, SITL,  rngfnd_pos_offset, 0),
    AP_GROUPINFO("FLOW_POS",      56, SITL,  optflow_pos_offset, 0),
    AP_GROUPINFO("ACC2_BIAS",     57, SITL,  accel2_bias, 0),
    AP_GROUPINFO("GPS_NOISE",     58, SITL,  gps_noise, 0),
    AP_GROUPINFO("GP2_GLITCH",    59, SITL,  gps2_glitch,  0),
    AP_GROUPINFO("ENGINE_FAIL",   60, SITL,  engine_fail,  0),
    AP_GROUPINFO("GPS2_TYPE",     61, SITL,  gps2_type,  SITL::GPS_TYPE_UBLOX),
    AP_GROUPINFO("ODOM_ENABLE",   62, SITL,  odom_enable, 0),
    AP_SUBGROUPEXTENSION("",      63, SITL,  var_info2),
    AP_GROUPEND
};

// second table of user settable parameters for SITL. 
const AP_Param::GroupInfo SITL::var_info2[] = {
    AP_GROUPINFO("TEMP_START",   1, SITL,  temp_start,  25),
    AP_GROUPINFO("TEMP_FLIGHT",  2, SITL,  temp_flight, 35),
    AP_GROUPINFO("TEMP_TCONST",  3, SITL,  temp_tconst, 30),
    AP_GROUPINFO("TEMP_BFACTOR", 4, SITL,  temp_baro_factor, 0),
    AP_GROUPINFO("GPS_LOCKTIME", 5, SITL,  gps_lock_time, 0),
    AP_GROUPINFO("ARSPD_FAIL_P", 6, SITL,  arspd_fail_pressure, 0),
    AP_GROUPINFO("ARSPD_PITOT",  7, SITL,  arspd_fail_pitot_pressure, 0),
    AP_GROUPINFO("GPS_ALT_OFS",  8, SITL,  gps_alt_offset, 0),
    AP_GROUPINFO("ARSPD_SIGN",   9, SITL,  arspd_signflip, 0),
    AP_GROUPINFO("WIND_DIR_Z",  10, SITL,  wind_dir_z,     0),
    AP_GROUPINFO("ARSPD2_FAIL", 11, SITL,  arspd2_fail, 0),
    AP_GROUPINFO("ARSPD2_FAILP",12, SITL,  arspd2_fail_pressure, 0),
    AP_GROUPINFO("ARSPD2_PITOT",13, SITL,  arspd2_fail_pitot_pressure, 0),
    AP_GROUPINFO("VICON_HSTLEN",14, SITL,  vicon_observation_history_length, 0),
    AP_GROUPINFO("WIND_T"      ,15, SITL,  wind_type, SITL::WIND_TYPE_SQRT),
    AP_GROUPINFO("WIND_T_ALT"  ,16, SITL,  wind_type_alt, 60),
    AP_GROUPINFO("WIND_T_COEF", 17, SITL,  wind_type_coef, 0.01f),
    AP_GROUPINFO("MAG_DIA",     18, SITL,  mag_diag, 0),
    AP_GROUPINFO("MAG_ODI",     19, SITL,  mag_offdiag, 0),
    AP_GROUPINFO("MAG_ORIENT",  20, SITL,  mag_orient, 0),
    AP_GROUPEND
};
    

/* report SITL state via MAVLink */
void SITL::simstate_send(mavlink_channel_t chan)
{
    float yaw;

    // convert to same conventions as DCM
    yaw = state.yawDeg;
    if (yaw > 180) {
        yaw -= 360;
    }

    mavlink_msg_simstate_send(chan,
                              ToRad(state.rollDeg),
                              ToRad(state.pitchDeg),
                              ToRad(yaw),
                              state.xAccel,
                              state.yAccel,
                              state.zAccel,
                              radians(state.rollRate),
                              radians(state.pitchRate),
                              radians(state.yawRate),
                              state.latitude*1.0e7,
                              state.longitude*1.0e7);
}

/* report SITL state to DataFlash */
void SITL::Log_Write_SIMSTATE(DataFlash_Class *DataFlash)
{
    float yaw;

    // convert to same conventions as DCM
    yaw = state.yawDeg;
    if (yaw > 180) {
        yaw -= 360;
    }

    struct log_AHRS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SIMSTATE_MSG),
        time_us : AP_HAL::micros64(),
        roll    : (int16_t)(state.rollDeg*100),
        pitch   : (int16_t)(state.pitchDeg*100),
        yaw     : (uint16_t)(wrap_360_cd(yaw*100)),
        alt     : (float)state.altitude,
        lat     : (int32_t)(state.latitude*1.0e7),
        lng     : (int32_t)(state.longitude*1.0e7),
        q1      : state.quaternion.q1,
        q2      : state.quaternion.q2,
        q3      : state.quaternion.q3,
        q4      : state.quaternion.q4,
    };
    DataFlash->WriteBlock(&pkt, sizeof(pkt));
}

/*
 convert a set of roll rates from earth frame to body frame
 output values are in radians/second
*/
void SITL::convert_body_frame(double rollDeg, double pitchDeg,
                              double rollRate, double pitchRate, double yawRate,
                              double *p, double *q, double *r)
{
    double phi, theta, phiDot, thetaDot, psiDot;

    phi = ToRad(rollDeg);
    theta = ToRad(pitchDeg);
    phiDot = ToRad(rollRate);
    thetaDot = ToRad(pitchRate);
    psiDot = ToRad(yawRate);

    *p = phiDot - psiDot*sin(theta);
    *q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta);
    *r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot;
}


/*
  convert angular velocities from body frame to
  earth frame.

  all inputs and outputs are in radians/s
*/
Vector3f SITL::convert_earth_frame(const Matrix3f &dcm, const Vector3f &gyro)
{
    float p = gyro.x;
    float q = gyro.y;
    float r = gyro.z;

    float phi, theta, psi;
    dcm.to_euler(&phi, &theta, &psi);

    float phiDot = p + tanf(theta)*(q*sinf(phi) + r*cosf(phi));
    float thetaDot = q*cosf(phi) - r*sinf(phi);
    if (fabsf(cosf(theta)) < 1.0e-20f) {
        theta += 1.0e-10f;
    }
    float psiDot = (q*sinf(phi) + r*cosf(phi))/cosf(theta);
    return Vector3f(phiDot, thetaDot, psiDot);
}

} // namespace SITL


namespace AP {

SITL::SITL *sitl()
{
    return SITL::SITL::get_instance();
}

};
