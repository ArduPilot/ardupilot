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
const Vector3f AP_AHRS_NavEKF::get_gyro(void) const
{
    return AP_AHRS_DCM::get_gyro();
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    if (!ekf_started) {
        return AP_AHRS_DCM::get_dcm_matrix();
    }
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    return AP_AHRS_DCM::get_gyro_drift();
}

void AP_AHRS_NavEKF::update(void)
{
    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    if (!ekf_started) {
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        // if we have GPS lock and a compass set we can start the EKF
        if (get_compass() && _gps && _gps->status() >= GPS::GPS_OK_FIX_3D &&
            hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf_started = true;
            EKF.InitialiseFilter();
        }
    }
    if (ekf_started) {
        EKF.UpdateFilter();
        if (using_EKF()) {
            EKF.getRotationBodyToNED(_dcm_matrix);
            Vector3f eulers;
            EKF.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;
            roll_sensor  = degrees(roll) * 100;
            pitch_sensor = degrees(pitch) * 100;
            yaw_sensor   = degrees(yaw) * 100;
            if (yaw_sensor < 0)
                yaw_sensor += 36000;
        }
    }
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf_started) {
        EKF.InitialiseFilter();        
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf_started) {
        EKF.InitialiseFilter();        
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc)
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
        return AP_AHRS_DCM::wind_estimate();
    }
    Vector3f wind;
    EKF.getWind(wind);
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret)
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
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

#endif // AP_AHRS_NAVEKF_AVAILABLE
