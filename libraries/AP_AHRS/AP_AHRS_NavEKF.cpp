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

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_gyro();
    }
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_dcm_matrix();
    }
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::get_gyro_drift();
    }
    return _gyro_bias;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF.resetGyroBias();
}

void AP_AHRS_NavEKF::update(void)
{
    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    if (!ekf_started) {
        // wait 10 seconds
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf_started = true;
            EKF.InitialiseFilterDynamic();
        }
    }
    if (ekf_started) {
        EKF.UpdateFilter();
        EKF.getRotationBodyToNED(_dcm_matrix);
        if (using_EKF()) {
            Vector3f eulers;
            EKF.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF.getGyroBias(_gyro_bias);
            _gyro_bias = -_gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            _gyro_estimate.zero();
            uint8_t healthy_count = 0;    
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i)) {
                    _gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                _gyro_estimate /= healthy_count;
            }
            _gyro_estimate += _gyro_bias;

            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * _ins.get_accel(i);
                }
            }

            // update _accel_ef_ekf_blended
            EKF.getAccelNED(_accel_ef_ekf_blended);
        }
    }
}

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef(i);
    }
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    if(!using_EKF()) {
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
    return _accel_ef_ekf_blended;
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf_started) {
        EKF.InitialiseFilterBootstrap();        
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf_started) {
        EKF.InitialiseFilterBootstrap();        
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    if (using_EKF() && EKF.getLLH(loc)) {
        return true;
    }
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void)
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void)
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    if (!using_EKF()) {
        // EKF does not estimate wind speed when there is no airspeed
        // sensor active
        return AP_AHRS_DCM::wind_estimate();
    }
    Vector3f wind;
    EKF.getWind(wind);
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    if (using_EKF()) {
        return EKF.use_compass();
    }
    return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    if (using_EKF()) {
        // return DCM attitude
        eulers = _dcm_attitude;
        return true;
    }
    if (ekf_started) {
        // EKF is secondary
        EKF.getEulerAngles(eulers);
        return true;
    }
    // no secondary available
    return false;
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc)
{
    if (using_EKF()) {
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    }    
    if (ekf_started) {
        // EKF is secondary
        EKF.getLLH(loc);
        return true;
    }
    // no secondary available
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    if (!using_EKF()) {
        return AP_AHRS_DCM::groundspeed_vector();
    }
    Vector3f vec;
    EKF.getVelNED(vec);
    return Vector2f(vec.x, vec.y);
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const 
{
    return using_EKF();
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    if (using_EKF()) {
        EKF.getVelNED(vec);
        return true;
    }
    return false;
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    if (using_EKF()) {
        return EKF.getPosNED(vec);
    }
    return false;
}

bool AP_AHRS_NavEKF::using_EKF(void) const
{
    return ekf_started && _ekf_use && EKF.healthy();
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void)
{
    if (_ekf_use) {
        return ekf_started && EKF.healthy();
    }
    return AP_AHRS_DCM::healthy();    
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    // initialisation complete 10sec after ekf has started
    return (ekf_started && (hal.scheduler->millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
};

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, uint8_t &rangeHealth, float &rawSonarRange)
{
    EKF.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, rangeHealth, rawSonarRange);
}

// inhibit GPS useage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    return EKF.setInhibitGPS();
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    EKF.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

