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
#include "AP_AHRS_View.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Module/AP_Module.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// constructor
AP_AHRS_NavEKF::AP_AHRS_NavEKF(NavEKF2 &_EKF2,
                               NavEKF3 &_EKF3,
                               Flags flags) :
    AP_AHRS_DCM(),
    EKF2(_EKF2),
    EKF3(_EKF3),
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
    return _gyro_drift;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF2.resetGyroBias();
    EKF3.resetGyroBias();
}

void AP_AHRS_NavEKF::update(bool skip_ins_update)
{
    // drop back to normal priority if we were boosted by the INS
    // calling delay_microseconds_boost()
    hal.scheduler->boost_end();
    
    // EKF1 is no longer supported - handle case where it is selected
    if (_ekf_type == 1) {
        _ekf_type.set(2);
    }


    update_DCM(skip_ins_update);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    update_SITL();
#endif

    if (_ekf_type == 2) {
        // if EK2 is primary then run EKF2 first to give it CPU
        // priority
        update_EKF2();
        update_EKF3();
    } else {
        // otherwise run EKF3 first
        update_EKF3();
        update_EKF2();
    }

#if AP_MODULE_SUPPORTED
    // call AHRS_update hook if any
    AP_Module::call_hook_AHRS_update(*this);
#endif

    // push gyros if optical flow present
    if (hal.opticalflow) {
        const Vector3f &exported_gyro_bias = get_gyro_drift();
        hal.opticalflow->push_gyro_bias(exported_gyro_bias.x, exported_gyro_bias.y);
    }

    if (_view != nullptr) {
        // update optional alternative attitude view
        _view->update(skip_ins_update);
    }
}

void AP_AHRS_NavEKF::update_DCM(bool skip_ins_update)
{
    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update(skip_ins_update);

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);
}

void AP_AHRS_NavEKF::update_EKF2(void)
{
    if (!_ekf2_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
        if (AP_HAL::millis() - start_time_ms > startup_delay_ms || _force_ekf) {
            _ekf2_started = EKF2.InitialiseFilter();
            if (_force_ekf) {
                return;
            }
        }
    }
    if (_ekf2_started) {
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

            // Use the primary EKF to select the primary gyro
            const int8_t primary_imu = EKF2.getPrimaryCoreIMUIndex();

            const AP_InertialSensor &_ins = AP::ins();

            // get gyro bias for primary EKF and change sign to give gyro drift
            // Note sign convention used by EKF is bias = measurement - truth
            _gyro_drift.zero();
            EKF2.getGyroBias(-1,_gyro_drift);
            _gyro_drift = -_gyro_drift;

            // calculate corrected gyro estimate for get_gyro()
            _gyro_estimate.zero();
            if (primary_imu == -1) {
                // the primary IMU is undefined so use an uncorrected default value from the INS library
                _gyro_estimate = _ins.get_gyro();
            } else {
                // use the same IMU as the primary EKF and correct for gyro drift
                _gyro_estimate = _ins.get_gyro(primary_imu) + _gyro_drift;
            }

            // get z accel bias estimate from active EKF (this is usually for the primary IMU)
            float abias = 0;
            EKF2.getAccelZBias(-1,abias);

            // This EKF is currently using primary_imu, and abias applies to only that IMU
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i == primary_imu) {
                    accel.z -= abias;
                }
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * accel;
                }
            }
            _accel_ef_ekf_blended = _accel_ef_ekf[primary_imu>=0?primary_imu:_ins.get_primary_accel()];
            nav_filter_status filt_state;
            EKF2.getFilterStatus(-1,filt_state);
            AP_Notify::flags.gps_fusion = filt_state.flags.using_gps; // Drives AP_Notify flag for usable GPS.
            AP_Notify::flags.gps_glitching = filt_state.flags.gps_glitching;
            AP_Notify::flags.have_pos_abs = filt_state.flags.horiz_pos_abs;
        }
    }
}


void AP_AHRS_NavEKF::update_EKF3(void)
{
    if (!_ekf3_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
        if (AP_HAL::millis() - start_time_ms > startup_delay_ms || _force_ekf) {
            _ekf3_started = EKF3.InitialiseFilter();
            if (_force_ekf) {
                return;
            }
        }
    }
    if (_ekf3_started) {
        EKF3.UpdateFilter();
        if (active_EKF_type() == EKF_TYPE3) {
            Vector3f eulers;
            EKF3.getRotationBodyToNED(_dcm_matrix);
            EKF3.getEulerAngles(-1,eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            const AP_InertialSensor &_ins = AP::ins();

            // Use the primary EKF to select the primary gyro
            const int8_t primary_imu = EKF3.getPrimaryCoreIMUIndex();

            // get gyro bias for primary EKF and change sign to give gyro drift
            // Note sign convention used by EKF is bias = measurement - truth
            _gyro_drift.zero();
            EKF3.getGyroBias(-1,_gyro_drift);
            _gyro_drift = -_gyro_drift;

            // calculate corrected gyro estimate for get_gyro()
            _gyro_estimate.zero();
            if (primary_imu == -1) {
                // the primary IMU is undefined so use an uncorrected default value from the INS library
                _gyro_estimate = _ins.get_gyro();
            } else {
                // use the same IMU as the primary EKF and correct for gyro drift
                _gyro_estimate = _ins.get_gyro(primary_imu) + _gyro_drift;
            }

            // get 3-axis accel bias festimates for active EKF (this is usually for the primary IMU)
            Vector3f abias;
            EKF3.getAccelBias(-1,abias);

            // This EKF uses the primary IMU
            // Eventually we will run a separate instance of the EKF for each IMU and do the selection and blending of EKF outputs upstream
            // update _accel_ef_ekf
            for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
                Vector3f accel = _ins.get_accel(i);
                if (i==_ins.get_primary_accel()) {
                    accel -= abias;
                }
                if (_ins.get_accel_health(i)) {
                    _accel_ef_ekf[i] = _dcm_matrix * accel;
                }
            }
            _accel_ef_ekf_blended = _accel_ef_ekf[_ins.get_primary_accel()];
            nav_filter_status filt_state;
            EKF3.getFilterStatus(-1,filt_state);
            AP_Notify::flags.gps_fusion = filt_state.flags.using_gps; // Drives AP_Notify flag for usable GPS.
            AP_Notify::flags.gps_glitching = filt_state.flags.gps_glitching;
            AP_Notify::flags.have_pos_abs = filt_state.flags.horiz_pos_abs;
        }
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_AHRS_NavEKF::update_SITL(void)
{
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        if (_sitl == nullptr) {
            return;
        }
    }

    const struct SITL::sitl_fdm &fdm = _sitl->state;

    if (active_EKF_type() == EKF_TYPE_SITL) {
        roll  = radians(fdm.rollDeg);
        pitch = radians(fdm.pitchDeg);
        yaw   = radians(fdm.yawDeg);

        fdm.quaternion.rotation_matrix(_dcm_matrix);

        update_cd_values();
        update_trig();

        _gyro_drift.zero();

        _gyro_estimate = Vector3f(radians(fdm.rollRate),
                                  radians(fdm.pitchRate),
                                  radians(fdm.yawRate));

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            const Vector3f accel(fdm.xAccel,
                           fdm.yAccel,
                           fdm.zAccel);
            _accel_ef_ekf[i] = _dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * accel;
        }
        _accel_ef_ekf_blended = _accel_ef_ekf[0];

    }

    if (_sitl->odom_enable) {
        // use SITL states to write body frame odometry data at 20Hz
        uint32_t timeStamp_ms = AP_HAL::millis();
        if (timeStamp_ms - _last_body_odm_update_ms > 50) {
            const float quality = 100.0f;
            const Vector3f posOffset(0.0f, 0.0f, 0.0f);
            const float delTime = 0.001f * (timeStamp_ms - _last_body_odm_update_ms);
            _last_body_odm_update_ms = timeStamp_ms;
            timeStamp_ms -= (timeStamp_ms - _last_body_odm_update_ms)/2; // correct for first order hold average delay
            Vector3f delAng(radians(fdm.rollRate),
                                       radians(fdm.pitchRate),
                                       radians(fdm.yawRate));
            delAng *= delTime;
            // rotate earth velocity into body frame and calculate delta position
            Matrix3f Tbn;
            Tbn.from_euler(radians(fdm.rollDeg),radians(fdm.pitchDeg),radians(fdm.yawDeg));
            const Vector3f earth_vel(fdm.speedN,fdm.speedE,fdm.speedD);
            const Vector3f delPos = Tbn.transposed() * (earth_vel * delTime);
            // write to EKF
            EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, posOffset);
        }
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
    if (_ekf2_started) {
        _ekf2_started = EKF2.InitialiseFilter();
    }
    if (_ekf3_started) {
        _ekf3_started = EKF3.InitialiseFilter();
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    _dcm_attitude(roll, pitch, yaw);
    if (_ekf2_started) {
        _ekf2_started = EKF2.InitialiseFilter();
    }
    if (_ekf3_started) {
        _ekf3_started = EKF3.InitialiseFilter();
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    Vector3f ned_pos;
    Location origin;
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return AP_AHRS_DCM::get_position(loc);

    case EKF_TYPE2:
        if (EKF2.getLLH(loc) && EKF2.getPosD(-1,ned_pos.z) && EKF2.getOriginLLH(-1,origin)) {
            // fixup altitude using relative position from EKF origin
            loc.alt = origin.alt - ned_pos.z*100;
            return true;
        }
        break;

    case EKF_TYPE3:
        if (EKF3.getLLH(loc) && EKF3.getPosD(-1,ned_pos.z) && EKF3.getOriginLLH(-1,origin)) {
            // fixup altitude using relative position from EKF origin
            loc.alt = origin.alt - ned_pos.z*100;
            return true;
        }
        break;
        
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            memset(&loc, 0, sizeof(loc));
            loc.lat = fdm.latitude * 1e7;
            loc.lng = fdm.longitude * 1e7;
            loc.alt = fdm.altitude*100;
            return true;
        }
        break;
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
Vector3f AP_AHRS_NavEKF::wind_estimate(void) const
{
    Vector3f wind;
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        wind = AP_AHRS_DCM::wind_estimate();
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        wind.zero();
        break;
#endif

    case EKF_TYPE2:
        EKF2.getWind(-1,wind);
        break;

    case EKF_TYPE3:
        EKF3.getWind(-1,wind);
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
    case EKF_TYPE2:
        return EKF2.use_compass();

    case EKF_TYPE3:
        return EKF3.use_compass();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return true;
#endif
    }
    return AP_AHRS_DCM::use_compass();
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        // EKF is secondary
        EKF2.getEulerAngles(-1, eulers);
        return _ekf2_started;

    case EKF_TYPE2:

    case EKF_TYPE3:

    default:
        // DCM is secondary
        eulers = _dcm_attitude;
        return true;
    }
}


// return secondary attitude solution if available, as quaternion
bool AP_AHRS_NavEKF::get_secondary_quaternion(Quaternion &quat) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        // EKF is secondary
        EKF2.getQuaternion(-1, quat);
        return _ekf2_started;

    case EKF_TYPE2:

    case EKF_TYPE3:

    default:
        // DCM is secondary
        quat.from_rotation_matrix(AP_AHRS_DCM::get_rotation_body_to_ned());
        return true;
    }
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        // EKF is secondary
        EKF2.getLLH(loc);
        return _ekf2_started;

    case EKF_TYPE2:

    case EKF_TYPE3:

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

    case EKF_TYPE2:
    default:
        EKF2.getVelNED(-1,vec);
        return Vector2f(vec.x, vec.y);

    case EKF_TYPE3:
        EKF3.getVelNED(-1,vec);
        return Vector2f(vec.x, vec.y);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            return Vector2f(fdm.speedN, fdm.speedE);
        } else {
            return AP_AHRS_DCM::groundspeed_vector();
        }
    }
#endif
    }
}

// set the EKF's origin location in 10e7 degrees.  This should only
// be called when the EKF has no absolute position reference (i.e. GPS)
// from which to decide the origin on its own
bool AP_AHRS_NavEKF::set_origin(const Location &loc)
{
    const bool ret2 = EKF2.setOriginLLH(loc);
    const bool ret3 = EKF3.setOriginLLH(loc);

    // return success if active EKF's origin was set
    switch (active_EKF_type()) {
    case EKF_TYPE2:
        return ret2;

    case EKF_TYPE3:
        return ret3;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        if (_sitl) {
            struct SITL::sitl_fdm &fdm = _sitl->state;
            fdm.home = loc;
            return true;
        } else {
            return false;
        }
#endif

    default:
        return false;
    }
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

    case EKF_TYPE2:
    default:
        EKF2.getVelNED(-1,vec);
        return true;

    case EKF_TYPE3:
        EKF3.getVelNED(-1,vec);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        vec = Vector3f(fdm.speedN, fdm.speedE, fdm.speedD);
        return true;
#endif
    }
}

// returns the expected NED magnetic field
bool AP_AHRS_NavEKF::get_mag_field_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE2:
    default:
        EKF2.getMagNED(-1,vec);
        return true;

    case EKF_TYPE3:
        EKF3.getMagNED(-1,vec);
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

    case EKF_TYPE2:
    default:
        EKF2.getMagXYZ(-1,vec);
        return true;

    case EKF_TYPE3:
        EKF3.getMagXYZ(-1,vec);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return false;
#endif
    }
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
bool AP_AHRS_NavEKF::get_vert_pos_rate(float &velocity) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE2:
    default:
        velocity = EKF2.getPosDownDerivative(-1);
        return true;

    case EKF_TYPE3:
        velocity = EKF3.getPosDownDerivative(-1);
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            velocity = fdm.speedD;
            return true;
        } else {
            return false;
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

    case EKF_TYPE2:
    default:
        return EKF2.getHAGL(height);
        
    case EKF_TYPE3:
        return EKF3.getHAGL(height);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        height = fdm.altitude - get_home().alt*0.01f;
        return true;
    }
#endif
    }
}

// return a relative ground position to the origin in meters
// North/East/Down order.
bool AP_AHRS_NavEKF::get_relative_position_NED_origin(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE2:
    default: {
        Vector2f posNE;
        float posD;
        if (EKF2.getPosNE(-1,posNE) && EKF2.getPosD(-1,posD)) {
            // position is valid
            vec.x = posNE.x;
            vec.y = posNE.y;
            vec.z = posD;
            return true;
        }
        return false;
    }

    case EKF_TYPE3: {
            Vector2f posNE;
            float posD;
            if (EKF3.getPosNE(-1,posNE) && EKF3.getPosD(-1,posD)) {
                // position is valid
                vec.x = posNE.x;
                vec.y = posNE.y;
                vec.z = posD;
                return true;
            }
            return false;
        }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        if (!_sitl) {
            return false;
        }
        Location loc;
        get_position(loc);
        const Vector2f diff2d = location_diff(get_home(), loc);
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        vec = Vector3f(diff2d.x, diff2d.y,
                       -(fdm.altitude - get_home().alt*0.01f));
        return true;
    }
#endif
    }
}

// return a relative ground position to the home in meters
// North/East/Down order.
bool AP_AHRS_NavEKF::get_relative_position_NED_home(Vector3f &vec) const
{
    Location originLLH;
    Vector3f originNED;
    if (!get_relative_position_NED_origin(originNED) ||
        !get_origin(originLLH)) {
        return false;
    }

    const Vector3f offset = location_3d_diff_NED(originLLH, _home);

    vec.x = originNED.x - offset.x;
    vec.y = originNED.y - offset.y;
    vec.z = originNED.z - offset.z;
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order
// return true if estimate is valid
bool AP_AHRS_NavEKF::get_relative_position_NE_origin(Vector2f &posNE) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE2:
    default: {
        bool position_is_valid = EKF2.getPosNE(-1,posNE);
        return position_is_valid;
    }

    case EKF_TYPE3: {
        bool position_is_valid = EKF3.getPosNE(-1,posNE);
        return position_is_valid;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        Location loc;
        get_position(loc);
        posNE = location_diff(get_home(), loc);
        return true;
    }
#endif
    }
}

// return a relative ground position to the home in meters
// North/East order.
bool AP_AHRS_NavEKF::get_relative_position_NE_home(Vector2f &posNE) const
{
    Location originLLH;
    Vector2f originNE;
    if (!get_relative_position_NE_origin(originNE) ||
        !get_origin(originLLH)) {
        return false;
    }

    const Vector2f offset = location_diff(originLLH, _home);

    posNE.x = originNE.x - offset.x;
    posNE.y = originNE.y - offset.y;
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order


// write a relative ground position to the origin in meters, Down
// return true if the estimate is valid
bool AP_AHRS_NavEKF::get_relative_position_D_origin(float &posD) const
{
    switch (active_EKF_type()) {
    case EKF_TYPE_NONE:
        return false;

    case EKF_TYPE2:
    default: {
        bool position_is_valid = EKF2.getPosD(-1,posD);
        return position_is_valid;
    }

    case EKF_TYPE3: {
        bool position_is_valid = EKF3.getPosD(-1,posD);
        return position_is_valid;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL: {
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        posD = -(fdm.altitude - get_home().alt*0.01f);
        return true;
    }
#endif
    }
}

// write a relative ground position to home in meters, Down
// will use the barometer if the EKF isn't available
void AP_AHRS_NavEKF::get_relative_position_D_home(float &posD) const
{
    Location originLLH;
    float originD;
    if (!get_relative_position_D_origin(originD) ||
        !get_origin(originLLH)) {
        posD = -AP::baro().get_altitude();
        return;
    }

    posD = originD - ((originLLH.alt - _home.alt) * 0.01f);
    return;
}
/*
  canonicalise _ekf_type, forcing it to be 0, 2 or 3
  type 1 has been deprecated
 */
uint8_t AP_AHRS_NavEKF::ekf_type(void) const
{
    uint8_t type = _ekf_type;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (type == EKF_TYPE_SITL) {
        return type;
    }
#endif
    if ((always_use_EKF() && (type == 0)) || (type == 1) || (type > 3))  {
        type = 2;
    }
    return type;
}

AP_AHRS_NavEKF::EKF_TYPE AP_AHRS_NavEKF::active_EKF_type(void) const
{
    EKF_TYPE ret = EKF_TYPE_NONE;

    switch (ekf_type()) {
    case 0:
        return EKF_TYPE_NONE;

    case 2: {
        // do we have an EKF2 yet?
        if (!_ekf2_started) {
            return EKF_TYPE_NONE;
        }
        if (always_use_EKF()) {
            uint16_t ekf2_faults;
            EKF2.getFilterFaults(-1,ekf2_faults);
            if (ekf2_faults == 0) {
                ret = EKF_TYPE2;
            }
        } else if (EKF2.healthy()) {
            ret = EKF_TYPE2;
        }
        break;
    }

    case 3: {
        // do we have an EKF3 yet?
        if (!_ekf3_started) {
            return EKF_TYPE_NONE;
        }
        if (always_use_EKF()) {
            uint16_t ekf3_faults;
            EKF3.getFilterFaults(-1,ekf3_faults);
            if (ekf3_faults == 0) {
                ret = EKF_TYPE3;
            }
        } else if (EKF3.healthy()) {
            ret = EKF_TYPE3;
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
      DCM is very robust. Note that we also check the filter status
      when fly_forward is false and we are disarmed. This is to ensure
      that the arming checks do wait for good GPS position on fixed
      wing and rover
     */
    if (ret != EKF_TYPE_NONE &&
        (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
         _vehicle_class == AHRS_VEHICLE_GROUND) &&
        (_flags.fly_forward || !hal.util->get_soft_armed())) {
        nav_filter_status filt_state;
        if (ret == EKF_TYPE2) {
            EKF2.getFilterStatus(-1,filt_state);
        } else if (ret == EKF_TYPE3) {
            EKF3.getFilterStatus(-1,filt_state);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        } else if (ret == EKF_TYPE_SITL) {
            get_filter_status(filt_state);
#endif
        }
        if (hal.util->get_soft_armed() && !filt_state.flags.using_gps && AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
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
            !filt_state.flags.vert_vel ||
            !filt_state.flags.vert_pos) {
            return EKF_TYPE_NONE;
        }
        if (!filt_state.flags.horiz_vel ||
            (!filt_state.flags.horiz_pos_abs && !filt_state.flags.horiz_pos_rel)) {
            if ((!_compass || !_compass->use_for_yaw()) &&
                AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D &&
                AP::gps().ground_speed() < 2) {
                /*
                  special handling for non-compass mode when sitting
                  still. The EKF may not yet have aligned its yaw. We
                  accept EKF as healthy to allow arming. Once we reach
                  speed the EKF should get yaw alignment
                */
                if (filt_state.flags.gps_quality_good) {
                    return ret;
                }
            }
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

    case 2: {
        bool ret = _ekf2_started && EKF2.healthy();
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

    case 3: {
        bool ret = _ekf3_started && EKF3.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
                _vehicle_class == AHRS_VEHICLE_GROUND) &&
                active_EKF_type() != EKF_TYPE3) {
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

    case 2:
    default:
        // initialisation complete 10sec after ekf has started
        return (_ekf2_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));

    case 3:
        // initialisation complete 10sec after ekf has started
        return (_ekf3_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));

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

    case EKF_TYPE2:
    default:
        EKF2.getFilterStatus(-1,status);
        return true;

    case EKF_TYPE3:
        EKF3.getFilterStatus(-1,status);
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
void  AP_AHRS_NavEKF::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, const Vector3f &posOffset)
{
    EKF2.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
    EKF3.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
}

// write body frame odometry measurements to the EKF
void  AP_AHRS_NavEKF::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset)
{
    EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, posOffset);
}

// Write position and quaternion data from an external navigation system
void AP_AHRS_NavEKF::writeExtNavData(const Vector3f &sensOffset, const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint32_t resetTime_ms)
{
    EKF2.writeExtNavData(sensOffset, pos, quat, posErr, angErr, timeStamp_ms, resetTime_ms);
}


// inhibit GPS usage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    switch (ekf_type()) {
    case 0:

    case 2:
    default:
        return EKF2.setInhibitGPS();

    case 3:
        return EKF3.setInhibitGPS();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return false;
#endif
    }
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    switch (ekf_type()) {
    case 0:

    case 2:
        EKF2.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;

    case 3:
        EKF3.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        // same as EKF2 for no optical flow
        ekfGndSpdLimit = 400.0f;
        ekfNavVelGainScaler = 1.0f;
        break;
#endif
    }
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS_NavEKF::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    switch (ekf_type()) {
    case 0:

    case 2:
    default:
        return EKF2.getMagOffsets(mag_idx, magOffsets);

    case 3:
        return EKF3.getMagOffsets(mag_idx, magOffsets);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        magOffsets.zero();
        return true;
#endif
    }
}

// Retrieves the NED delta velocity corrected
void AP_AHRS_NavEKF::getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const
{
    const EKF_TYPE type = active_EKF_type();
    if (type == EKF_TYPE2 || type == EKF_TYPE3) {
        int8_t imu_idx = 0;
        Vector3f accel_bias;
        if (type == EKF_TYPE2) {
            accel_bias.zero();
            imu_idx = EKF2.getPrimaryCoreIMUIndex();
            EKF2.getAccelZBias(-1,accel_bias.z);
        } else if (type == EKF_TYPE3) {
            imu_idx = EKF3.getPrimaryCoreIMUIndex();
            EKF3.getAccelBias(-1,accel_bias);
        }
        if (imu_idx == -1) {
            // should never happen, call parent implementation in this scenario
            AP_AHRS::getCorrectedDeltaVelocityNED(ret, dt);
            return;
        }
        ret.zero();
        const AP_InertialSensor &_ins = AP::ins();
        _ins.get_delta_velocity((uint8_t)imu_idx, ret);
        dt = _ins.get_delta_velocity_dt((uint8_t)imu_idx);
        ret -= accel_bias*dt;
        ret = _dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * ret;
        ret.z += GRAVITY_MSS*dt;
    } else {
        AP_AHRS::getCorrectedDeltaVelocityNED(ret, dt);
    }
}

// report any reason for why the backend is refusing to initialise
const char *AP_AHRS_NavEKF::prearm_failure_reason(void) const
{
    switch (ekf_type()) {
    case 0:
        return nullptr;

    case 2:
    default:
        return EKF2.prearm_failure_reason();

    case 3:
        return EKF3.prearm_failure_reason();

    }
    return nullptr;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastYawResetAngle(float &yawAng) const
{
    switch (ekf_type()) {

    case 2:
    default:
        return EKF2.getLastYawResetAngle(yawAng);

    case 3:
        return EKF3.getLastYawResetAngle(yawAng);

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

    case 2:
    default:
        return EKF2.getLastPosNorthEastReset(pos);

    case 3:
        return EKF3.getLastPosNorthEastReset(pos);

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

    case 2:
    default:
        return EKF2.getLastVelNorthEastReset(vel);

    case 3:
        return EKF3.getLastVelNorthEastReset(vel);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return 0;
#endif
    }
    return 0;
}


// return the amount of vertical position change due to the last reset in meters
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastPosDownReset(float &posDelta) const
{
    switch (ekf_type()) {
    case EKF_TYPE2:
        return EKF2.getLastPosDownReset(posDelta);

    case EKF_TYPE3:
        return EKF3.getLastPosDownReset(posDelta);

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

    case 2:
    default: {
        EKF3.resetHeightDatum();
        return EKF2.resetHeightDatum();
    }

    case 3: {
        EKF2.resetHeightDatum();
        return EKF3.resetHeightDatum();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        return false;
#endif
    }
    return false;
}

// send a EKF_STATUS_REPORT for current EKF
void AP_AHRS_NavEKF::send_ekf_status_report(mavlink_channel_t chan) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        // send zero status report
        mavlink_msg_ekf_status_report_send(chan, 0, 0, 0, 0, 0, 0, 0);
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        // send zero status report
        mavlink_msg_ekf_status_report_send(chan, 0, 0, 0, 0, 0, 0, 0);
        break;
#endif
        
    case EKF_TYPE2:
        return EKF2.send_status_report(chan);

    case EKF_TYPE3:
        return EKF3.send_status_report(chan);

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

    case EKF_TYPE2:
    default:
        if (!EKF2.getOriginLLH(-1,ret)) {
            return false;
        }
        return true;

    case EKF_TYPE3:
        if (!EKF3.getOriginLLH(-1,ret)) {
            return false;
        }
        return true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKF_TYPE_SITL:
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        ret = fdm.home;
        return true;
#endif
    }
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return false when no limiting is required
bool AP_AHRS_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    switch (ekf_type()) {
    case EKF_TYPE_NONE:
        // We are not using an EKF so no limiting applies
        return false;

    case EKF_TYPE2:
    default:
        return EKF2.getHeightControlLimit(limit);

    case EKF_TYPE3:
        return EKF3.getHeightControlLimit(limit);

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

    case EKF_TYPE2:
    default:
        return EKF2.getLLH(loc);

    case EKF_TYPE3:
        return EKF3.getLLH(loc);

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

    case EKF_TYPE2:
    default:
        // use EKF to get variance
        EKF2.getVariances(-1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        return true;

    case EKF_TYPE3:
        // use EKF to get variance
        EKF3.getVariances(-1,velVar, posVar, hgtVar, magVar, tasVar, offset);
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
        case EKF_TYPE2:
        default:
            EKF2.setTakeoffExpected(val);
            break;

        case EKF_TYPE3:
            EKF3.setTakeoffExpected(val);
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
        case EKF_TYPE2:
        default:
            EKF2.setTouchdownExpected(val);
            break;

        case EKF_TYPE3:
            EKF3.setTouchdownExpected(val);
            break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case EKF_TYPE_SITL:
            break;
#endif
    }
}

bool AP_AHRS_NavEKF::getGpsGlitchStatus() const
{
    nav_filter_status ekf_status {};
    if (!get_filter_status(ekf_status)) {
        return false;
    }
    return ekf_status.flags.gps_glitching;
}

// is the EKF backend doing its own sensor logging?
bool AP_AHRS_NavEKF::have_ekf_logging(void) const
{
    switch (ekf_type()) {
    case 2:
        return EKF2.have_ekf_logging();

    case 3:
        return EKF3.have_ekf_logging();

    default:
        break;
    }
    return false;
}

// get the index of the current primary IMU
uint8_t AP_AHRS_NavEKF::get_primary_IMU_index() const
{
    int8_t imu = -1;
    switch (ekf_type()) {
    case 2:
        // let EKF2 choose primary IMU
        imu = EKF2.getPrimaryCoreIMUIndex();
        break;
    case 3:
        // let EKF2 choose primary IMU
        imu = EKF3.getPrimaryCoreIMUIndex();
        break;
    default:
        break;
    }
    if (imu == -1) {
        imu = AP::ins().get_primary_accel();
    }
    return imu;
}

// get earth-frame accel vector for primary IMU
const Vector3f &AP_AHRS_NavEKF::get_accel_ef() const
{
    return get_accel_ef(get_primary_accel_index());
}


// get the index of the current primary accelerometer sensor
uint8_t AP_AHRS_NavEKF::get_primary_accel_index(void) const
{
    if (ekf_type() != 0) {
        return get_primary_IMU_index();
    }
    return AP::ins().get_primary_accel();
}

// get the index of the current primary gyro sensor
uint8_t AP_AHRS_NavEKF::get_primary_gyro_index(void) const
{
    if (ekf_type() != 0) {
        return get_primary_IMU_index();
    }
    return AP::ins().get_primary_gyro();
}


AP_AHRS_NavEKF &AP::ahrs_navekf()
{
    return static_cast<AP_AHRS_NavEKF&>(*AP_AHRS::get_singleton());
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

