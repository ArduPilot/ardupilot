#ifndef __AP_AHRS_NAVEKF_H__
#define __AP_AHRS_NAVEKF_H__
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
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include <AP_HAL.h>
#include <AP_AHRS.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
#include <AP_NavEKF.h>

#define AP_AHRS_NAVEKF_AVAILABLE 1
#define AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000     // time in milliseconds the ekf needs to settle after being started

class AP_AHRS_NavEKF : public AP_AHRS_DCM
{
public:
    // Constructor
    AP_AHRS_NavEKF(AP_InertialSensor &ins, AP_Baro &baro, AP_GPS &gps) :
    AP_AHRS_DCM(ins, baro, gps),
        EKF(this, baro),
        ekf_started(false),
        startup_delay_ms(10000)
        {
        }

    // return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const;
    const Matrix3f &get_dcm_matrix(void) const;

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift(void);

    void            update(void);
    void            reset(bool recover_eulers = false);

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw);

    // dead-reckoning support
    bool get_position(struct Location &loc) const;

    // status reporting of estimated error
    float           get_error_rp(void);
    float           get_error_yaw(void);

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void);

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret) const;

    // true if compass is being used
    bool use_compass(void);

    NavEKF &get_NavEKF(void) { return EKF; }

    // return secondary attitude solution if available, as eulers in radians
    bool get_secondary_attitude(Vector3f &eulers);

    // return secondary position solution if available
    bool get_secondary_position(struct Location &loc);

    // EKF has a better ground speed vector estimate
    Vector2f groundspeed_vector(void);

    const Vector3f &get_accel_ef(uint8_t i) const;
    const Vector3f &get_accel_ef() const { return get_accel_ef(_ins.get_primary_accel()); };

    // blended accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef_blended(void) const;

    // set home location
    void set_home(const Location &loc);

    bool have_inertial_nav(void) const;

    bool get_velocity_NED(Vector3f &vec) const;
    bool get_relative_position_NED(Vector3f &vec) const;

    // write optical flow measurements to EKF
    void writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, uint8_t &rangeHealth, float &rawSonarRange);

    // inibit GPS useage
    uint8_t setInhibitGPS(void);

    // get speed limit
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler);

    void set_ekf_use(bool setting) { _ekf_use.set(setting); }

    // is the AHRS subsystem healthy?
    bool healthy(void);

    // true if the AHRS has completed initialisation
    bool initialised(void) const;

private:
    bool using_EKF(void) const;

    NavEKF EKF;
    bool ekf_started;
    Matrix3f _dcm_matrix;
    Vector3f _dcm_attitude;
    Vector3f _gyro_bias;
    Vector3f _gyro_estimate;
    Vector3f _accel_ef_ekf[INS_MAX_INSTANCES];
    Vector3f _accel_ef_ekf_blended;
    const uint16_t startup_delay_ms;
    uint32_t start_time_ms;
};
#endif

#endif // __AP_AHRS_NAVEKF_H__
