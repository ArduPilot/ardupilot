/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    if (!active_EKF_type()) {
        return AP_AHRS_DCM::get_gyro();
    }
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    if (!active_EKF_type()) {
        return AP_AHRS_DCM::get_dcm_matrix();
    }
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    if (!active_EKF_type()) {
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
    EKF1.resetGyroBias();
    EKF2.resetGyroBias();
}

void AP_AHRS_NavEKF::update(void)
{
    update_DCM();
    update_EKF1();
    update_EKF2();
}

void AP_AHRS_NavEKF::update_DCM(void)
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
}

void AP_AHRS_NavEKF::update_EKF1(void)
{
    if (!ekf1_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf1_started = EKF1.InitialiseFilterDynamic();
        }
    }
    if (ekf1_started) {
        EKF1.UpdateFilter();
        EKF1.getRotationBodyToNED(_dcm_matrix);
        if (active_EKF_type() == EKF_TYPE1) {
            Vector3f eulers;
            EKF1.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF1.getGyroBias(_gyro_bias);
            _gyro_bias = -_gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            _gyro_estimate.zero();
            uint8_t healthy_count = 0;
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i) && healthy_count < 2) {
                    _gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                _gyro_estimate /= healthy_count;
            }
            _gyro_estimate += _gyro_bias;

            float abias1, abias2;
            EKF1.getAccelZBias(abias1, abias2);

            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==0) {
                    accel.z -= abias1;
                } else if (i==1) {
                    accel.z -= abias2;
                }
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * accel;
                }
            }

            if(_ins.use_accel(0) && _ins.use_accel(1)) {
                float IMU1_weighting;
                EKF1.getIMU1Weighting(IMU1_weighting);
                _accel_ef_ekf_blended = _accel_ef_ekf[0] * IMU1_weighting + _accel_ef_ekf[1] * (1.0f-IMU1_weighting);
            } else {
                _accel_ef_ekf_blended = _accel_ef_ekf[_ins.get_primary_accel()];
            }
        }
    }
}


void AP_AHRS_NavEKF::update_EKF2(void)
{
    if (!ekf2_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = hal.scheduler->millis();
        }
        if (hal.scheduler->millis() - start_time_ms > startup_delay_ms) {
            ekf2_started = EKF2.InitialiseFilter();
        }
    }
    if (ekf2_started) {
        EKF2.UpdateFilter();
        EKF2.getRotationBodyToNED(_dcm_matrix);
        if (active_EKF_type() == EKF_TYPE2) {
            Vector3f eulers;
            EKF2.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF2.getGyroBias(_gyro_bias);
            _gyro_bias = -_gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            _gyro_estimate.zero();
            uint8_t healthy_count = 0;
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i) && healthy_count < 2) {
                    _gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                _gyro_estimate /= healthy_count;
            }
            _gyro_estimate += _gyro_bias;

            float abias;
            EKF2.getAccelZBias(abias);

            // This EKF uses the primary IMU
            // Eventually we will run a separate instance of the EKF for each IMU and do the selection and blending of EKF outputs upstream
            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==_ins.get_primary_accel()) {
                    accel.z -= abias;
                }
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * accel;
                }
            }
            _accel_ef_ekf_blended = _accel_ef_ekf[_ins.get_primary_accel()];
        }
    }
}

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    if (active_EKF_type() == EKF_TYPE_NONE) {
        return AP_AHRS_DCM::get_accel_ef(i);
    }
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    if (active_EKF_type() == EKF_TYPE_NONE) {
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
    return _accel_ef_ekf_blended;
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf1_started) {
        ekf1_started = EKF1.InitialiseFilterBootstrap();
    }
    if (ekf2_started) {
        ekf2_started = EKF2.InitialiseFilter();
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf1_started) {
        ekf1_started = EKF1.InitialiseFilterBootstrap();
    }
    if (ekf2_started) {
        ekf2_started = EKF2.InitialiseFilter();
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    Vector3f ned_pos;
    switch (active_EKF_type()) {
    case EKF_TYPE1:
        if (EKF1.getLLH(loc) && EKF1.getPosNED(ned_pos)) {
            // fixup altitude using relative position from AHRS home, not
            // EKF origin
            loc.alt = get_home().alt - ned_pos.z*100;
            return true;
        }
        break;
    case EKF_TYPE2:
        if (EKF2.getLLH(loc) && EKF2.getPosNED(ned_pos)) {
            // fixup altitude using relative position from AHRS home, not
            // EKF origin
            loc.alt = get_home().alt - ned_pos.z*100;
            return true;
        }
        break;

    default:
        break;
    }
    return AP_AHRS_DCM::get_position(loc);
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void) const
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void) const
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    Vector3f wind;
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        wind = AP_AHRS_DCM::wind_estimate();
        break;

    case EKF_TYPE1:
        EKF1.getWind(wind);
        break;

    case EKF_TYPE2:
        EKF2.getWind(wind);
        break;
    }
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
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        break;
    case EKF_TYPE1:
        return EKF1.use_compass();
    case EKF_TYPE2:
        return EKF2.use_compass();
    }
    return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        // EKF is secondary
        EKF1.getEulerAngles(eulers);
        return ekf1_started;

    case EKF_TYPE1:
    case EKF_TYPE2:
    default:
        // DCM is secondary
        eulers = _dcm_attitude;
        return true;
    }
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc)
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        // EKF is secondary
        EKF1.getLLH(loc);
        return ekf1_started;

    case EKF_TYPE1:
    case EKF_TYPE2:
    default:
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    }
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    Vector3f vec;

    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return AP_AHRS_DCM::groundspeed_vector();

    case EKF_TYPE1:
    default:
        EKF1.getVelNED(vec);
        return Vector2f(vec.x, vec.y);

    case EKF_TYPE2:
        EKF2.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    }
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const
{
    return active_EKF_type() != EKF_TYPE_NONE;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE1:
    default:
        EKF1.getVelNED(vec);
        return true;

    case EKF_TYPE2:
        EKF2.getVelNED(vec);
        return true;
    }
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE1:
    default:
        return EKF1.getPosNED(vec);

    case EKF_TYPE2:
        return EKF2.getPosNED(vec);
    }
}

/*
  canonicalise _ekf_type, forcing it to be 0, 1 or 2
 */
uint8_t AP_AHRS_NavEKF::ekf_type(void) const
{
    uint8_t type = _ekf_type;
#if AHRS_EKF_USE_ALWAYS
    // on copters always use an EKF
    if (type == 0) {
        type = 1;
    }
#endif

    // check for invalid type
    if (type > 2) {
        type = 1;
    }
    return type;
}

AP_AHRS_NavEKF::EKF_TYPE AP_AHRS_NavEKF::active_EKF_type(void) const
{
    EKF_TYPE ret = EKF_TYPE_NONE;

    switch (ekf_type()) {
    case 0:
        return EKF_TYPE_NONE;

    case 1: {
        // do we have an EKF yet?
        if (!ekf1_started) {
            return EKF_TYPE_NONE;
        }
#if AHRS_EKF_USE_ALWAYS
        uint8_t ekf_faults;
        EKF1.getFilterFaults(ekf_faults);
        if (ekf_faults == 0) {
            ret = EKF_TYPE1;
        }
#else
        if (EKF1.healthy()) {
            ret = EKF_TYPE1;
        }
#endif
        break;
    }

    case 2: {
        // do we have an EKF2 yet?
        if (!ekf2_started) {
            return EKF_TYPE_NONE;
        }
#if AHRS_EKF_USE_ALWAYS
        uint8_t ekf2_faults;
        EKF2.getFilterFaults(ekf2_faults);
        if (ekf2_faults == 0) {
            ret = EKF_TYPE2;
        }
#else
        if (EKF2.healthy()) {
            ret = EKF_TYPE2;
        }
#endif
        break;
    }
    }

    if (ret != EKF_TYPE_NONE &&
            (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
             _vehicle_class == AHRS_VEHICLE_GROUND)) {
        nav_filter_status filt_state;
        if (ret == EKF_TYPE1) {
            EKF1.getFilterStatus(filt_state);
        } else {
            EKF2.getFilterStatus(filt_state);
        }
        if (hal.util->get_soft_armed() && !filt_state.flags.using_gps && _gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            // if the EKF is not fusing GPS and we have a 3D lock, then
            // plane and rover would prefer to use the GPS position from
            // DCM. This is a safety net while some issues with the EKF
            // get sorted out
            return EKF_TYPE_NONE;
        }
        if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
            return EKF_TYPE_NONE;
        }
        if (!filt_state.flags.attitude ||
                !filt_state.flags.horiz_vel ||
                !filt_state.flags.vert_vel ||
                !filt_state.flags.horiz_pos_abs ||
                !filt_state.flags.vert_pos) {
            return EKF_TYPE_NONE;
        }
    }
    return ret;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    switch (ekf_type()) {
    case 0:
        return AP_AHRS_DCM::healthy();

    case 1: {
        bool ret = ekf1_started && EKF1.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
                _vehicle_class == AHRS_VEHICLE_GROUND) &&
                active_EKF_type() != EKF_TYPE1) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }

    case 2: {
        bool ret = ekf2_started && EKF2.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
                _vehicle_class == AHRS_VEHICLE_GROUND) &&
                active_EKF_type() != EKF_TYPE2) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }
    }

    return AP_AHRS_DCM::healthy();
}

void AP_AHRS_NavEKF::set_ekf_use(bool setting)
{
    _ekf_type.set(setting?1:0);
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    switch (ekf_type()) {
    case 0:
        return true;

    case 1:
    default:
        // initialisation complete 10sec after ekf has started
        return (ekf1_started && (hal.scheduler->millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));

    case 2:
        // initialisation complete 10sec after ekf has started
        return (ekf2_started && (hal.scheduler->millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
    }
};

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    EKF1.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
    EKF2.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
}

// inhibit GPS useage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    switch (ekf_type()) {
    case 0:
    case 1:
    default:
        return EKF1.setInhibitGPS();

    case 2:
        return EKF2.setInhibitGPS();
    }
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    switch (ekf_type()) {
    case 0:
    case 1:
    default:
        EKF1.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;

    case 2:
        EKF2.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;
    }
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(Vector3f &magOffsets)
{
    switch (ekf_type()) {
    case 0:
    case 1:
    default:
        return EKF1.getMagOffsets(magOffsets);

    case 2:
        return EKF2.getMagOffsets(magOffsets);
    }
}

// report any reason for why the backend is refusing to initialise
const char *AP_AHRS_NavEKF::prearm_failure_reason(void) const
{
    switch (ekf_type()) {
    case 0:
        return nullptr;
    case 1:
        return EKF1.prearm_failure_reason();
    case 2:
        // not implemented yet
        return nullptr;
    }
    return nullptr;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastYawResetAngle(float &yawAng)
{
    switch (ekf_type()) {
    case 1:
        return EKF1.getLastYawResetAngle(yawAng);
    case 2:
        return EKF2.getLastYawResetAngle(yawAng);
    }
    return false;
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool AP_AHRS_NavEKF::resetHeightDatum(void)
{
    switch (ekf_type()) {
    case 1:
        EKF2.resetHeightDatum();
        return EKF1.resetHeightDatum();
    case 2:
        EKF1.resetHeightDatum();
        return EKF2.resetHeightDatum();
    }
    return false;    
}

// send a EKF_STATUS_REPORT for current EKF
void AP_AHRS_NavEKF::send_ekf_status_report(mavlink_channel_t chan)
{
    switch (active_EKF_type()) {
    case EKF_TYPE1:
    default:
        return EKF1.send_status_report(chan);

    case EKF_TYPE2:
        return EKF2.send_status_report(chan);
    }    
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

