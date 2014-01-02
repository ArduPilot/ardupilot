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

// return the smoothed gyro vector corrected for drift
const Vector3f AP_AHRS_NavEKF::get_gyro(void) const
{
    return AP_AHRS_DCM::get_gyro();
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    return AP_AHRS_DCM::get_dcm_matrix();
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    return AP_AHRS_DCM::get_gyro_drift();
}

void AP_AHRS_NavEKF::update(void)
{
    AP_AHRS_DCM::update();
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc)
{
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated error
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
    return AP_AHRS_DCM::wind_estimate();
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

#endif // AP_AHRS_NAVEKF_AVAILABLE
