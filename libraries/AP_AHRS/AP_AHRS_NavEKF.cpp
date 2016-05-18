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

// constructor
AP_AHRS_NavEKF::AP_AHRS_NavEKF(AP_InertialSensor &ins, AP_Baro &baro, AP_GPS &gps, RangeFinder &rng,
                               NavEKF &_EKF1, NavEKF2 &_EKF2, Flags flags) :
    AP_AHRS_DCM(ins, baro, gps),
    EKF1(_EKF1),
    EKF2(_EKF2),
    _ekf_flags(flags)
{
    _dcm_matrix.identity();
}

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    if (!active_EKF_type()) {
        return AP_AHRS_DCM::get_gyro();
    }
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS_NavEKF::get_rotation_body_to_ned(void) const
{
    if (!active_EKF_type()) {
        return AP_AHRS_DCM::get_rotation_body_to_ned();
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
#if !AP_AHRS_WITH_EKF1
    if (_ekf_type == 1) {
        _ekf_type.set(2);
    }
#endif
    update_DCM();
#if AP_AHRS_WITH_EKF1
    update_EKF1();
#endif
    update_EKF2();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    update_SITL();
#endif
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
#if AP_AHRS_WITH_EKF1
    if (!ekf1_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
        // slight extra delay on EKF1 to prioritise EKF2 for memory
        if (AP_HAL::millis() - start_time_ms > startup_delay_ms + 100U || force_ekf) {
            ekf1_started = EKF1.InitialiseFilterDynamic();
            if (force_ekf) {
                return;
            }
        }
    }
    if (ekf1_started) {
        EKF1.UpdateFilter();
        if (active_EKF_type() == EKF_TYPE1) {
            Vector3f eulers;
            EKF1.getRotationBodyToNED(_dcm_matrix);
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
                if (_ins.get_gyro_health(i) && healthy_count < 2 && _ins.use_gyro(i)) {
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
#endif
}


void AP_AHRS_NavEKF::update_EKF2(void)
{
    if (!ekf2_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
        if (AP_HAL::millis() - start_time_ms > startup_delay_ms || force_ekf) {
            ekf2_started = EKF2.InitialiseFilter();
            if (force_ekf) {
                return;
            }
        }
    }
    if (ekf2_started) {
        EKF2.UpdateFilter();
        if (active_EKF_type() == EKF_TYPE2) {
            Vector3f eulers;
            EKF2.getRotationBodyToNED(_dcm_matrix);
            EKF2.getEulerAngles(-1,eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF2.getGyroBias(-1,_gyro_bias);
            _gyro_bias = -_gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            _gyro_estimate.zero();
            uint8_t healthy_count = 0;
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i) && healthy_count < 2 && _ins.use_gyro(i)) {
                    _gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                _gyro_estimate /= healthy_count;
            }
            _gyro_estimate += _gyro_bias;

            float abias;
            EKF2.getAccelZBias(-1,abias);

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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_AHRS_NavEKF::update_SITL(void)
{
    if (_sitl == nullptr) {
        _sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    }
    if (_sitl && active_EKF_type() == EKF_TYPE_SITL) {
        const struct SITL::sitl_fdm &fdm = _sitl->state;

        roll  = radians(fdm.rollDeg);
        pitch = radians(fdm.pitchDeg);
        yaw   = radians(fdm.yawDeg);

        _dcm_matrix.from_euler(roll, pitch, yaw);

        update_cd_values();
        update_trig();

        _gyro_bias.zero();

        _gyro_estimate = Vector3f(radians(fdm.rollRate),
                                  radians(fdm.pitchRate),
                                  radians(fdm.yawRate));

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            _accel_ef_ekf[i] = Vector3f(fdm.xAccel,
                                        fdm.yAccel,
                                        fdm.zAccel);
        }
        _accel_ef_ekf_blended = _accel_ef_ekf[0];
    }
}
#endif // CONFIG_HAL_BOARD

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
    _dcm_attitude(roll, pitch, yaw);
#if AP_AHRS_WITH_EKF1
    if (ekf1_started) {
        ekf1_started = EKF1.InitialiseFilterBootstrap();
    }
#endif
    if (ekf2_started) {
        ekf2_started = EKF2.InitialiseFilter();
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    _dcm_attitude(roll, pitch, yaw);
#if AP_AHRS_WITH_EKF1
    if (ekf1_started) {
        ekf1_started = EKF1.InitialiseFilterBootstrap();
    }
#endif
    if (ekf2_started) {
        ekf2_started = EKF2.InitialiseFilter();
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    Vector3f ned_pos;
    Location origin;
    switch (active_EKF_type()) {
#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        if (EKF1.getLLH(loc) && EKF1.getPosNED(ned_pos) && EKF1.getOriginLLH(origin)) {
            // fixup altitude using relative position from EKF origin
            loc.alt = origin.alt - ned_pos.z*100;
            return true;
        }
        break;
#endif
    case EKF_TYPE2:
        if (EKF2.getLLH(loc) && EKF2.getPosNED(-1,ned_pos) && EKF2.getOriginLLH(origin)) {
            // fixup altitude using relative position from EKF origin
            loc.alt = origin.alt - ned_pos.z*100;
            return true;
        }
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        memset(&loc, 0, sizeof(loc));
        loc.lat = fdm.latitude * 1e7;
        loc.lng = fdm.longitude * 1e7;
        loc.alt = fdm.altitude*100;
        return true;
    }
#endif
        
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

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        EKF1.getWind(wind);
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        wind.zero();
        break;
#endif

    case EKF_TYPE2:
    default:
        EKF2.getWind(-1,wind);
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
#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        return EKF1.use_compass();
#endif
    case EKF_TYPE2:
        return EKF2.use_compass();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return true;
#endif
    }
    return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        // EKF is secondary
#if AP_AHRS_WITH_EKF1
        EKF1.getEulerAngles(eulers);
        return ekf1_started;
#else
        EKF2.getEulerAngles(-1, eulers);
        return ekf2_started;
#endif

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
#endif
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
#if AP_AHRS_WITH_EKF1
        EKF1.getLLH(loc);
        return ekf1_started;
#else
        EKF2.getLLH(loc);
        return ekf2_started;
#endif

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
#endif
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

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        EKF1.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
#endif

    case EKF_TYPE2:
    default:
        EKF2.getVelNED(-1,vec);
        return Vector2f(vec.x, vec.y);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        return Vector2f(fdm.speedN, fdm.speedE);
    }
#endif
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

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        EKF1.getVelNED(vec);
        return true;
#endif

    case EKF_TYPE2:
    default:
        EKF2.getVelNED(-1,vec);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        vec = Vector3f(fdm.speedN, fdm.speedE, fdm.speedD);
        return true;
    }
#endif
    }
}

// returns the expected NED magnetic field
bool AP_AHRS_NavEKF::get_mag_field_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
        case EKF_TYPE_NONE:
            return false;

#if AP_AHRS_WITH_EKF1
        case EKF_TYPE1:
            EKF1.getMagNED(vec);
            return true;
#endif

        case EKF_TYPE2:
        default:
            EKF2.getMagNED(-1,vec);
            return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case EKF_TYPE_SITL:
            return false;
#endif
    }
}

// returns the estimated magnetic field offsets in body frame
bool AP_AHRS_NavEKF::get_mag_field_correction(Vector3f &vec) const
{
    switch (active_EKF_type()) {
        case EKF_TYPE_NONE:
            return false;

#if AP_AHRS_WITH_EKF1
        case EKF_TYPE1:
            EKF1.getMagXYZ(vec);
            return true;
#endif

        case EKF_TYPE2:
        default:
            EKF2.getMagXYZ(-1,vec);
            return true;

            #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case EKF_TYPE_SITL:
            return false;
            #endif
    }
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
bool AP_AHRS_NavEKF::get_vert_pos_rate(float &velocity)
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        velocity = EKF1.getPosDownDerivative();
        return true;
#endif

    case EKF_TYPE2:
    default:
        velocity = EKF2.getPosDownDerivative(-1);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        velocity = fdm.speedD;
        return true;
    }
#endif
    }
}

// get latest height above ground level estimate in metres and a validity flag
bool AP_AHRS_NavEKF::get_hagl(float &height) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        return EKF1.getHAGL(height);
#endif

    case EKF_TYPE2:
    default:
        return EKF2.getHAGL(height);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        height = fdm.altitude - get_home().alt*0.01f;
        return true;
    }
#endif
    }
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        return EKF1.getPosNED(vec);
#endif

    case EKF_TYPE2:
    default:
        return EKF2.getPosNED(-1,vec);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        Location loc;
        get_position(loc);
        Vector2f diff2d = location_diff(get_home(), loc);
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        vec = Vector3f(diff2d.x, diff2d.y,
                       -(fdm.altitude - get_home().alt*0.01f));
        return true;
    }
#endif
    }
}

/*
  canonicalise _ekf_type, forcing it to be 0, 1 or 2
 */
uint8_t AP_AHRS_NavEKF::ekf_type(void) const
{
    uint8_t type = _ekf_type;
    if (always_use_EKF() && type == 0) {
        type = 1;
    }

#if !AP_AHRS_WITH_EKF1
    if (type == 1) {
        type = 2;
    }
#endif
    
    // check for invalid type
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (type > 2 && type != EKF_TYPE_SITL) {
        type = 2;
    }
#else
    if (type > 2) {
        type = 2;
    }
#endif
    return type;
}

AP_AHRS_NavEKF::EKF_TYPE AP_AHRS_NavEKF::active_EKF_type(void) const
{
    EKF_TYPE ret = EKF_TYPE_NONE;

    switch (ekf_type()) {
    case 0:
        return EKF_TYPE_NONE;

#if AP_AHRS_WITH_EKF1
    case 1: {
        // do we have an EKF yet?
        if (!ekf1_started) {
            return EKF_TYPE_NONE;
        }
        if (always_use_EKF()) {
            uint8_t ekf_faults;
            EKF1.getFilterFaults(ekf_faults);
            if (ekf_faults == 0) {
                ret = EKF_TYPE1;
            }
        } else if (EKF1.healthy()) {
            ret = EKF_TYPE1;
        }
        break;
    }
#endif

    case 2: {
        // do we have an EKF2 yet?
        if (!ekf2_started) {
            return EKF_TYPE_NONE;
        }
        if (always_use_EKF()) {
            uint8_t ekf2_faults;
            EKF2.getFilterFaults(-1,ekf2_faults);
            if (ekf2_faults == 0) {
                ret = EKF_TYPE2;
            }
        } else if (EKF2.healthy()) {
            ret = EKF_TYPE2;
        }
        break;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        ret = EKF_TYPE_SITL;
        break;
#endif
    }

    /*
      fixed wing and rover when in fly_forward mode will fall back to
      DCM if the EKF doesn't have GPS. This is the safest option as
      DCM is very robust
     */
    if (ret != EKF_TYPE_NONE &&
        (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
         _vehicle_class == AHRS_VEHICLE_GROUND) &&
        _flags.fly_forward) {
        nav_filter_status filt_state;
        if (ret == EKF_TYPE2) {
            EKF2.getFilterStatus(-1,filt_state);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        } else if (ret == EKF_TYPE_SITL) {
            get_filter_status(filt_state);
#endif
#if AP_AHRS_WITH_EKF1
        } else {
            EKF1.getFilterStatus(filt_state);
#endif
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

#if AP_AHRS_WITH_EKF1
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
#endif

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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return true;
#endif
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
        // initialisation complete 10sec after ekf has started
        return (ekf1_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));

    case 2:
    default:
        // initialisation complete 10sec after ekf has started
        return (ekf2_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return true;
#endif
    }
};

// get_filter_status : returns filter status as a series of flags
bool AP_AHRS_NavEKF::get_filter_status(nav_filter_status &status) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        EKF1.getFilterStatus(status);
        return true;
#endif

    case EKF_TYPE2:
    default:
        EKF2.getFilterStatus(-1,status);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        memset(&status, 0, sizeof(status));
        status.flags.attitude = 1;
        status.flags.horiz_vel = 1;
        status.flags.vert_vel = 1;
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_pos_abs = 1;
        status.flags.vert_pos = 1;
        status.flags.pred_horiz_pos_rel = 1;
        status.flags.pred_horiz_pos_abs = 1;
        status.flags.using_gps = 1;
        return true;
#endif
    }

}

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas)
{
    EKF1.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
    EKF2.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas);
}

// inhibit GPS usage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    switch (ekf_type()) {
    case 0:
    case 1:
        return EKF1.setInhibitGPS();

    case 2:
    default:
        return EKF2.setInhibitGPS();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return false;
#endif
    }
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler)
{
    switch (ekf_type()) {
    case 0:
    case 1:
        EKF1.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;

    case 2:
    default:
        EKF2.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        // same as EKF1 for no optical flow
        ekfGndSpdLimit = 400.0f;
        ekfNavVelGainScaler = 1.0f;
        break;
#endif
    }
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets)
{
    switch (ekf_type()) {
    case 0:
    case 1:
        return EKF1.getMagOffsets(mag_idx, magOffsets);

    case 2:
    default:
        return EKF2.getMagOffsets(mag_idx, magOffsets);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        magOffsets.zero();
        return true;
#endif
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
        return EKF2.prearm_failure_reason();
    }
    return nullptr;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastYawResetAngle(float &yawAng) const
{
    switch (ekf_type()) {
    case 1:
        return EKF1.getLastYawResetAngle(yawAng);
    case 2:
        return EKF2.getLastYawResetAngle(yawAng);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return 0;
#endif
    }
    return 0;
}

// return the amount of NE position change in metres due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastPosNorthEastReset(Vector2f &pos) const
{
    switch (ekf_type()) {
    case 1:
        return EKF1.getLastPosNorthEastReset(pos);
    case 2:
        return EKF2.getLastPosNorthEastReset(pos);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return 0;
#endif
    }
    return 0;
}

// return the amount of NE velocity change in metres/sec due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastVelNorthEastReset(Vector2f &vel) const
{
    switch (ekf_type()) {
    case 1:
        return EKF1.getLastVelNorthEastReset(vel);
    case 2:
        return EKF2.getLastVelNorthEastReset(vel);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return 0;
#endif
    }
    return 0;
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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return false;
#endif
    }
    return false;
}

// send a EKF_STATUS_REPORT for current EKF
void AP_AHRS_NavEKF::send_ekf_status_report(mavlink_channel_t chan)
{
    switch (active_EKF_type()) {
#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        return EKF1.send_status_report(chan);
#endif

    case EKF_TYPE2:
    default:
        return EKF2.send_status_report(chan);
    }    
}

// passes a reference to the location of the inertial navigation origin
// in WGS-84 coordinates
// returns a boolean true when the inertial navigation origin has been set
bool AP_AHRS_NavEKF::get_origin(Location &ret) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        if (!EKF1.getOriginLLH(ret)) {
            return false;
        }
        return true;
#endif

    case EKF_TYPE2:
    default:
        if (!EKF2.getOriginLLH(ret)) {
            return false;
        }
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        ret = get_home();
        return ret.lat != 0 || ret.lng != 0;
#endif
    }
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return invalid when no limiting is required
bool AP_AHRS_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        // We are not using an EKF so no limiting applies
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        return EKF1.getHeightControlLimit(limit);
#endif

    case EKF_TYPE2:
    default:
        return EKF2.getHeightControlLimit(limit);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return false;
#endif
    }
}

// get_location - updates the provided location with the latest calculated location
//  returns true on success (i.e. the EKF knows it's latest position), false on failure
bool AP_AHRS_NavEKF::get_location(struct Location &loc) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        // We are not using an EKF so no data
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        return EKF1.getLLH(loc);
#endif

    case EKF_TYPE2:
    default:
        return EKF2.getLLH(loc);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return get_position(loc);
#endif
    }
}

// get_variances - provides the innovations normalised using the innovation variance where a value of 0
// indicates prefect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
// inconsistency that will be accpeted by the filter
// boolean false is returned if variances are not available
bool AP_AHRS_NavEKF::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        // We are not using an EKF so no data
        return false;

#if AP_AHRS_WITH_EKF1
    case EKF_TYPE1:
        // use EKF to get variance
        EKF1.getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
        return true;
#endif

    case EKF_TYPE2:
    default:
        // use EKF to get variance
        EKF2.getVariances(-1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        velVar = 0;
        posVar = 0;
        hgtVar = 0;
        magVar.zero();
        tasVar = 0;
        offset.zero();
        return true;
#endif
    }
}

void AP_AHRS_NavEKF::setTakeoffExpected(bool val)
{
    switch (ekf_type()) {
#if AP_AHRS_WITH_EKF1
        case EKF_TYPE1:
            EKF1.setTakeoffExpected(val);
            break;
#endif
        case EKF_TYPE2:
            EKF2.setTakeoffExpected(val);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case EKF_TYPE_SITL:
            break;
#endif
    }
}

void AP_AHRS_NavEKF::setTouchdownExpected(bool val)
{
    switch (ekf_type()) {
#if AP_AHRS_WITH_EKF1
        case EKF_TYPE1:
            EKF1.setTouchdownExpected(val);
            break;
#endif
        case EKF_TYPE2:
            EKF2.setTouchdownExpected(val);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case EKF_TYPE_SITL:
            break;
#endif
    }
}

bool AP_AHRS_NavEKF::getGpsGlitchStatus()
{
    nav_filter_status ekf_status;
    get_filter_status(ekf_status);

    return ekf_status.flags.gps_glitching;
}


// is the EKF backend doing its own sensor logging?
bool AP_AHRS_NavEKF::have_ekf_logging(void) const
{
    switch (ekf_type()) {
    case 2:
        return EKF2.have_ekf_logging();
    default:
        break;
    }
    return false;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

