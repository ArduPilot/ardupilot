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
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

#if AP_AHRS_NAVEKF_AVAILABLE

#define ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD radians(10)
#define ATTITUDE_CHECK_THRESH_YAW_RAD radians(20)

extern const AP_HAL::HAL& hal;

// constructor
AP_AHRS_NavEKF::AP_AHRS_NavEKF(uint8_t flags) :
    AP_AHRS_DCM(),
    _ekf_flags(flags)
{
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
    // Copter and Sub force the use of EKF
    _ekf_flags |= AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF;
#endif
    _dcm_matrix.identity();
}

// init sets up INS board orientation
void AP_AHRS_NavEKF::init()
{
#if !HAL_NAVEKF2_AVAILABLE && HAL_NAVEKF3_AVAILABLE
    if (_ekf_type.get() == 2) {
        _ekf_type.set(3);
        EKF3.set_enable(true);
    }
#elif !HAL_NAVEKF3_AVAILABLE && HAL_NAVEKF2_AVAILABLE
    if (_ekf_type.get() == 3) {
        _ekf_type.set(2);
        EKF2.set_enable(true);
    }
#endif
    // init parent class
    AP_AHRS_DCM::init();
}

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    if (active_EKF_type() == EKFType::NONE) {
        return AP_AHRS_DCM::get_gyro();
    }
    return _gyro_estimate;
}

const Matrix3f &AP_AHRS_NavEKF::get_rotation_body_to_ned(void) const
{
    if (active_EKF_type() == EKFType::NONE) {
        return AP_AHRS_DCM::get_rotation_body_to_ned();
    }
    return _dcm_matrix;
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    if (active_EKF_type() == EKFType::NONE) {
        return AP_AHRS_DCM::get_gyro_drift();
    }
    return _gyro_drift;
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
#if HAL_NAVEKF2_AVAILABLE
    EKF2.resetGyroBias();
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.resetGyroBias();
#endif
}

void AP_AHRS_NavEKF::update(bool skip_ins_update)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
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
#if HAL_NAVEKF2_AVAILABLE
        update_EKF2();
#endif
#if HAL_NAVEKF3_AVAILABLE
        update_EKF3();
#endif
    } else {
        // otherwise run EKF3 first
#if HAL_NAVEKF3_AVAILABLE
        update_EKF3();
#endif
#if HAL_NAVEKF2_AVAILABLE
        update_EKF2();
#endif
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

#if !HAL_MINIMIZE_FEATURES && AP_AHRS_NAVEKF_AVAILABLE
    // update NMEA output
    update_nmea_out();
#endif
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

#if HAL_NAVEKF2_AVAILABLE
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
        if (active_EKF_type() == EKFType::TWO) {
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
            if (primary_imu == -1 || !_ins.get_gyro_health(primary_imu)) {
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
#endif

#if HAL_NAVEKF3_AVAILABLE
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
        if (active_EKF_type() == EKFType::THREE) {
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
            if (primary_imu == -1 || !_ins.get_gyro_health(primary_imu)) {
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
#endif

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
    const AP_InertialSensor &_ins = AP::ins();

    if (active_EKF_type() == EKFType::SITL) {

        fdm.quaternion.rotation_matrix(_dcm_matrix);
        _dcm_matrix = _dcm_matrix * get_rotation_vehicle_body_to_autopilot_body();
        _dcm_matrix.to_euler(&roll, &pitch, &yaw);

        update_cd_values();
        update_trig();

        _gyro_drift.zero();

        _gyro_estimate = _ins.get_gyro();

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            const Vector3f &accel = _ins.get_accel(i);
            _accel_ef_ekf[i] = _dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * accel;
        }
        _accel_ef_ekf_blended = _accel_ef_ekf[0];

    }

#if HAL_NAVEKF3_AVAILABLE
    if (_sitl->odom_enable) {
        // use SITL states to write body frame odometry data at 20Hz
        uint32_t timeStamp_ms = AP_HAL::millis();
        if (timeStamp_ms - _last_body_odm_update_ms > 50) {
            const float quality = 100.0f;
            const Vector3f posOffset(0.0f, 0.0f, 0.0f);
            const float delTime = 0.001f * (timeStamp_ms - _last_body_odm_update_ms);
            _last_body_odm_update_ms = timeStamp_ms;
            timeStamp_ms -= (timeStamp_ms - _last_body_odm_update_ms)/2; // correct for first order hold average delay
            Vector3f delAng = _ins.get_gyro();
            
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
#endif // HAL_NAVEKF3_AVAILABLE
}
#endif // CONFIG_HAL_BOARD

// accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef(uint8_t i) const
{
    if (active_EKF_type() == EKFType::NONE) {
        return AP_AHRS_DCM::get_accel_ef(i);
    }
    return _accel_ef_ekf[i];
}

// blended accelerometer values in the earth frame in m/s/s
const Vector3f &AP_AHRS_NavEKF::get_accel_ef_blended(void) const
{
    if (active_EKF_type() == EKFType::NONE) {
        return AP_AHRS_DCM::get_accel_ef_blended();
    }
    return _accel_ef_ekf_blended;
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    AP_AHRS_DCM::reset(recover_eulers);
    _dcm_attitude(roll, pitch, yaw);
#if HAL_NAVEKF2_AVAILABLE
    if (_ekf2_started) {
        _ekf2_started = EKF2.InitialiseFilter();
    }
#endif
#if HAL_NAVEKF3_AVAILABLE
    if (_ekf3_started) {
        _ekf3_started = EKF3.InitialiseFilter();
    }
#endif
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    _dcm_attitude(roll, pitch, yaw);
#if HAL_NAVEKF2_AVAILABLE
    if (_ekf2_started) {
        _ekf2_started = EKF2.InitialiseFilter();
    }
#endif
#if HAL_NAVEKF3_AVAILABLE
    if (_ekf3_started) {
        _ekf3_started = EKF3.InitialiseFilter();
    }
#endif
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return AP_AHRS_DCM::get_position(loc);

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (EKF2.getLLH(loc)) {
            return true;
        }
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (EKF3.getLLH(loc)) {
            return true;
        }
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL: {
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            loc = {};
            loc.lat = fdm.latitude * 1e7;
            loc.lng = fdm.longitude * 1e7;
            loc.alt = fdm.altitude*100;
            return true;
        }
        break;
    }
#endif
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
    case EKFType::NONE:
        wind = AP_AHRS_DCM::wind_estimate();
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        wind.zero();
        break;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getWind(-1,wind);
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getWind(-1,wind);
        break;
#endif

    }
    return wind;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float &airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.use_compass();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.use_compass();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return true;
#endif
    }
    return AP_AHRS_DCM::use_compass();
}

// return the quaternion defining the rotation from NED to XYZ (body) axes
bool AP_AHRS_NavEKF::get_quaternion(Quaternion &quat) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return AP_AHRS_DCM::get_quaternion(quat);
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getQuaternion(-1, quat);
        return _ekf2_started;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getQuaternion(-1, quat);
        return _ekf3_started;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            quat = fdm.quaternion;
            return true;
        } else {
            return false;
        }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        // EKF is secondary
#if HAL_NAVEKF2_AVAILABLE
        EKF2.getEulerAngles(-1, eulers);
        return _ekf2_started;
#else
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
#endif
        // DCM is secondary
        eulers = _dcm_attitude;
        return true;
    }
    // since there is no default case above, this is unreachable
    return false;
}


// return secondary attitude solution if available, as quaternion
bool AP_AHRS_NavEKF::get_secondary_quaternion(Quaternion &quat) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        // EKF is secondary
#if HAL_NAVEKF2_AVAILABLE
        EKF2.getQuaternion(-1, quat);
        return _ekf2_started;
#else
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
#endif
        // DCM is secondary
        quat.from_rotation_matrix(AP_AHRS_DCM::get_rotation_body_to_ned());
        return true;
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        // EKF is secondary
#if HAL_NAVEKF2_AVAILABLE
        EKF2.getLLH(loc);
        return _ekf2_started;
#else
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
#endif
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    }
    // since there is no default case above, this is unreachable
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    Vector3f vec;

    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getVelNED(-1,vec);
        return Vector2f(vec.x, vec.y);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getVelNED(-1,vec);
        return Vector2f(vec.x, vec.y);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL: {
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            return Vector2f(fdm.speedN, fdm.speedE);
        }
        break;
    }
#endif
    }
    return AP_AHRS_DCM::groundspeed_vector();
}

// set the EKF's origin location in 10e7 degrees.  This should only
// be called when the EKF has no absolute position reference (i.e. GPS)
// from which to decide the origin on its own
bool AP_AHRS_NavEKF::set_origin(const Location &loc)
{
#if HAL_NAVEKF2_AVAILABLE
    const bool ret2 = EKF2.setOriginLLH(loc);
#endif
#if HAL_NAVEKF3_AVAILABLE
    const bool ret3 = EKF3.setOriginLLH(loc);
#endif

    // return success if active EKF's origin was set
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return ret2;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return ret3;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        if (_sitl) {
            struct SITL::sitl_fdm &fdm = _sitl->state;
            fdm.home = loc;
            return true;
        } else {
            return false;
        }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const
{
    return active_EKF_type() != EKFType::NONE;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getVelNED(-1,vec);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getVelNED(-1,vec);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        vec = Vector3f(fdm.speedN, fdm.speedE, fdm.speedD);
        return true;
#endif
    }
    return AP_AHRS_DCM::get_velocity_NED(vec);
}

// returns the expected NED magnetic field
bool AP_AHRS_NavEKF::get_mag_field_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getMagNED(-1,vec);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getMagNED(-1,vec);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return false;
#endif
    }
    return false;
}

// returns the estimated magnetic field offsets in body frame
bool AP_AHRS_NavEKF::get_mag_field_correction(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getMagXYZ(-1,vec);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getMagXYZ(-1,vec);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
bool AP_AHRS_NavEKF::get_vert_pos_rate(float &velocity) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        velocity = EKF2.getPosDownDerivative(-1);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        velocity = EKF3.getPosDownDerivative(-1);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        if (_sitl) {
            const struct SITL::sitl_fdm &fdm = _sitl->state;
            velocity = fdm.speedD;
            return true;
        } else {
            return false;
        }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// get latest height above ground level estimate in metres and a validity flag
bool AP_AHRS_NavEKF::get_hagl(float &height) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getHAGL(height);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getHAGL(height);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL: {
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        height = fdm.altitude - get_home().alt*0.01f;
        return true;
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// return a relative ground position to the origin in meters
// North/East/Down order.
bool AP_AHRS_NavEKF::get_relative_position_NED_origin(Vector3f &vec) const
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
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
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
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
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL: {
        if (!_sitl) {
            return false;
        }
        Location loc;
        get_position(loc);
        const Vector2f diff2d = get_home().get_distance_NE(loc);
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        vec = Vector3f(diff2d.x, diff2d.y,
                       -(fdm.altitude - get_home().alt*0.01f));
        return true;
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
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

    const Vector3f offset = originLLH.get_distance_NED(_home);

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
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        bool position_is_valid = EKF2.getPosNE(-1,posNE);
        return position_is_valid;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        bool position_is_valid = EKF3.getPosNE(-1,posNE);
        return position_is_valid;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL: {
        Location loc;
        get_position(loc);
        posNE = get_home().get_distance_NE(loc);
        return true;
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
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

    const Vector2f offset = originLLH.get_distance_NE(_home);

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
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        bool position_is_valid = EKF2.getPosD(-1,posD);
        return position_is_valid;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        bool position_is_valid = EKF3.getPosD(-1,posD);
        return position_is_valid;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL: {
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        posD = -(fdm.altitude - get_home().alt*0.01f);
        return true;
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
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
AP_AHRS_NavEKF::EKFType AP_AHRS_NavEKF::ekf_type(void) const
{
    EKFType type = (EKFType)_ekf_type.get();
    switch (type) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return type;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return type;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return type;
#endif
    case EKFType::NONE:
        if (always_use_EKF()) {
#if HAL_NAVEKF2_AVAILABLE
            return EKFType::TWO;
#elif HAL_NAVEKF3_AVAILABLE
            return EKFType::THREE;
#endif
        }
        return EKFType::NONE;
    }
    // we can get to here if the user has mis-set AHRS_EKF_TYPE - any
    // value above 3 will get to here.  TWO is returned here for no
    // better reason than "tradition".
#if HAL_NAVEKF2_AVAILABLE
    return EKFType::TWO;
#elif HAL_NAVEKF3_AVAILABLE
    return EKFType::THREE;
#else
    return EKFType::NONE;
#endif
}

AP_AHRS_NavEKF::EKFType AP_AHRS_NavEKF::active_EKF_type(void) const
{
    EKFType ret = EKFType::NONE;

    switch (ekf_type()) {
    case EKFType::NONE:
        return EKFType::NONE;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        // do we have an EKF2 yet?
        if (!_ekf2_started) {
            return EKFType::NONE;
        }
        if (always_use_EKF()) {
            uint16_t ekf2_faults;
            EKF2.getFilterFaults(-1,ekf2_faults);
            if (ekf2_faults == 0) {
                ret = EKFType::TWO;
            }
        } else if (EKF2.healthy()) {
            ret = EKFType::TWO;
        }
        break;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        // do we have an EKF3 yet?
        if (!_ekf3_started) {
            return EKFType::NONE;
        }
        if (always_use_EKF()) {
            uint16_t ekf3_faults;
            EKF3.getFilterFaults(-1,ekf3_faults);
            if (ekf3_faults == 0) {
                ret = EKFType::THREE;
            }
        } else if (EKF3.healthy()) {
            ret = EKFType::THREE;
        }
        break;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        ret = EKFType::SITL;
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
    if (ret != EKFType::NONE &&
        (_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
         _vehicle_class == AHRS_VEHICLE_GROUND) &&
        (_flags.fly_forward || !hal.util->get_soft_armed())) {
        nav_filter_status filt_state;
#if HAL_NAVEKF2_AVAILABLE
        if (ret == EKFType::TWO) {
            EKF2.getFilterStatus(-1,filt_state);
        }
#endif
#if HAL_NAVEKF3_AVAILABLE
        if (ret == EKFType::THREE) {
            EKF3.getFilterStatus(-1,filt_state);
        }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (ret == EKFType::SITL) {
            get_filter_status(filt_state);
        }
#endif
        if (hal.util->get_soft_armed() && !filt_state.flags.using_gps && AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
            // if the EKF is not fusing GPS and we have a 3D lock, then
            // plane and rover would prefer to use the GPS position from
            // DCM. This is a safety net while some issues with the EKF
            // get sorted out
            return EKFType::NONE;
        }
        if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
            return EKFType::NONE;
        }
        if (!filt_state.flags.attitude ||
            !filt_state.flags.vert_vel ||
            !filt_state.flags.vert_pos) {
            return EKFType::NONE;
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
            return EKFType::NONE;
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
    case EKFType::NONE:
        return AP_AHRS_DCM::healthy();

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        bool ret = _ekf2_started && EKF2.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
                _vehicle_class == AHRS_VEHICLE_GROUND) &&
                active_EKF_type() != EKFType::TWO) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        bool ret = _ekf3_started && EKF3.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == AHRS_VEHICLE_FIXED_WING ||
                _vehicle_class == AHRS_VEHICLE_GROUND) &&
                active_EKF_type() != EKFType::THREE) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return true;
#endif
    }

    return AP_AHRS_DCM::healthy();
}

bool AP_AHRS_NavEKF::prearm_healthy(void) const
{
    bool prearm_health = false;
    switch (ekf_type()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
#endif
    case EKFType::NONE:
        prearm_health = true;
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (_ekf2_started && EKF2.all_cores_healthy()) {
            prearm_health = true;
        }
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (_ekf3_started && EKF3.all_cores_healthy()) {
            prearm_health = true;
        }
        break;
#endif
    }
   return prearm_health && healthy();
}

void AP_AHRS_NavEKF::set_ekf_use(bool setting)
{
    _ekf_type.set(setting?1:0);
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return true;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // initialisation complete 10sec after ekf has started
        return (_ekf2_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // initialisation complete 10sec after ekf has started
        return (_ekf3_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return true;
#endif
    }
    return false;
};

// get_filter_status : returns filter status as a series of flags
bool AP_AHRS_NavEKF::get_filter_status(nav_filter_status &status) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getFilterStatus(-1,status);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getFilterStatus(-1,status);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
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

    return false;
}

// write optical flow data to EKF
void  AP_AHRS_NavEKF::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset);
#endif
}

// write body frame odometry measurements to the EKF
void  AP_AHRS_NavEKF::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset)
{
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, posOffset);
#endif
}

// Write position and quaternion data from an external navigation system
void AP_AHRS_NavEKF::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint32_t resetTime_ms)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, resetTime_ms);
#endif
}

// Writes the default equivalent airspeed in m/s to be used in forward flight if a measured airspeed is required and not available.
void AP_AHRS_NavEKF::writeDefaultAirSpeed(float airspeed)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeDefaultAirSpeed(airspeed);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeDefaultAirSpeed(airspeed);
#endif
}

// inhibit GPS usage
uint8_t AP_AHRS_NavEKF::setInhibitGPS(void)
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.setInhibitGPS();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.setInhibitGPS();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return 0;
#endif
    }
    // since there is no default case above, this is unreachable
    return 0;
}

// get speed limit
void AP_AHRS_NavEKF::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        break;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
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
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getMagOffsets(mag_idx, magOffsets);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getMagOffsets(mag_idx, magOffsets);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        magOffsets.zero();
        return true;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// Retrieves the NED delta velocity corrected
void AP_AHRS_NavEKF::getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const
{
    int8_t imu_idx = -1;
    Vector3f accel_bias;
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        imu_idx = EKF2.getPrimaryCoreIMUIndex();
        EKF2.getAccelZBias(-1,accel_bias.z);
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        imu_idx = EKF3.getPrimaryCoreIMUIndex();
        EKF3.getAccelBias(-1,accel_bias);
        break;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        break;
#endif
    }
    if (imu_idx == -1) {
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
}

// report any reason for why the backend is refusing to initialise
const char *AP_AHRS_NavEKF::prearm_failure_reason(void) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return nullptr;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.prearm_failure_reason();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.prearm_failure_reason();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return nullptr;
#endif
    }

    return nullptr;
}

// check all cores providing consistent attitudes for prearm checks
bool AP_AHRS_NavEKF::attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const
{
    // get primary attitude source's attitude as quaternion
    Quaternion primary_quat;
    get_quat_body_to_ned(primary_quat);
    // only check yaw if compasses are being used
    bool check_yaw = _compass && _compass->use_for_yaw();

#if HAL_NAVEKF2_AVAILABLE
    // check primary vs ekf2
    for (uint8_t i = 0; i < EKF2.activeCores(); i++) {
        Quaternion ekf2_quat;
        Vector3f angle_diff;
        EKF2.getQuaternionBodyToNED(i, ekf2_quat);
        primary_quat.angular_difference(ekf2_quat).to_axis_angle(angle_diff);
        float diff = safe_sqrt(sq(angle_diff.x)+sq(angle_diff.y));
        if (diff > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 Roll/Pitch inconsistent by %d deg", (int)degrees(diff));
            return false;
        }
        diff = fabsf(angle_diff.z);
        if (check_yaw && (diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 Yaw inconsistent by %d deg", (int)degrees(diff));
            return false;
        }
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    // check primary vs ekf3
    for (uint8_t i = 0; i < EKF3.activeCores(); i++) {
        Quaternion ekf3_quat;
        Vector3f angle_diff;
        EKF3.getQuaternionBodyToNED(i, ekf3_quat);
        primary_quat.angular_difference(ekf3_quat).to_axis_angle(angle_diff);
        float diff = safe_sqrt(sq(angle_diff.x)+sq(angle_diff.y));
        if (diff > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF3 Roll/Pitch inconsistent by %d deg", (int)degrees(diff));
            return false;
        }
        diff = fabsf(angle_diff.z);
        if (check_yaw && (diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF3 Yaw inconsistent by %d deg", (int)degrees(diff));
            return false;
        }
    }
#endif

    // check primary vs dcm
    Quaternion dcm_quat;
    Vector3f angle_diff;
    dcm_quat.from_rotation_matrix(get_DCM_rotation_body_to_ned());
    primary_quat.angular_difference(dcm_quat).to_axis_angle(angle_diff);
    float diff = safe_sqrt(sq(angle_diff.x)+sq(angle_diff.y));
    if (diff > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
        hal.util->snprintf(failure_msg, failure_msg_len, "DCM Roll/Pitch inconsistent by %d deg", (int)degrees(diff));
        return false;
    }

    // we only check yaw against DCM if we are not using external yaw
    // for EKF3. DCM can't use external yaw, so we don't expect it's
    // yaw to align with EKF3 when EKF3 is using an external yaw
    // source
    bool using_external_yaw = false;
#if HAL_NAVEKF3_AVAILABLE
    using_external_yaw = ekf_type() == EKFType::THREE && EKF3.using_external_yaw();
#endif
    if (!using_external_yaw) {
        diff = fabsf(angle_diff.z);
        if (check_yaw && (diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "DCM Yaw inconsistent by %d deg", (int)degrees(diff));
            return false;
        }
    }

    return true;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastYawResetAngle(float &yawAng)
{
    switch (ekf_type()) {

    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastYawResetAngle(yawAng);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastYawResetAngle(yawAng);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return 0;
#endif
    }
    return 0;
}

// return the amount of NE position change in metres due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastPosNorthEastReset(Vector2f &pos)
{
    switch (ekf_type()) {

    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastPosNorthEastReset(pos);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastPosNorthEastReset(pos);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
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

    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastVelNorthEastReset(vel);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastVelNorthEastReset(vel);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return 0;
#endif
    }
    return 0;
}


// return the amount of vertical position change due to the last reset in meters
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS_NavEKF::getLastPosDownReset(float &posDelta)
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return 0;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastPosDownReset(posDelta);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastPosDownReset(posDelta);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
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
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    switch (ekf_type()) {

    case EKFType::NONE:
#if HAL_NAVEKF3_AVAILABLE
        EKF3.resetHeightDatum();
#endif
#if HAL_NAVEKF2_AVAILABLE
        EKF2.resetHeightDatum();
#endif
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#if HAL_NAVEKF3_AVAILABLE
        EKF3.resetHeightDatum();
#endif
        return EKF2.resetHeightDatum();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#if HAL_NAVEKF2_AVAILABLE
        EKF2.resetHeightDatum();
#endif
        return EKF3.resetHeightDatum();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return false;
#endif
    }
    return false;
}

// send a EKF_STATUS_REPORT for current EKF
void AP_AHRS_NavEKF::send_ekf_status_report(mavlink_channel_t chan) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // send zero status report
        mavlink_msg_ekf_status_report_send(chan, 0, 0, 0, 0, 0, 0, 0);
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        {
        // send status report with everything looking good
        const uint16_t flags =
        EKF_ATTITUDE | /* Set if EKF's attitude estimate is good. | */
        EKF_VELOCITY_HORIZ | /* Set if EKF's horizontal velocity estimate is good. | */
        EKF_VELOCITY_VERT | /* Set if EKF's vertical velocity estimate is good. | */
        EKF_POS_HORIZ_REL | /* Set if EKF's horizontal position (relative) estimate is good. | */
        EKF_POS_HORIZ_ABS | /* Set if EKF's horizontal position (absolute) estimate is good. | */
        EKF_POS_VERT_ABS | /* Set if EKF's vertical position (absolute) estimate is good. | */
        EKF_POS_VERT_AGL | /* Set if EKF's vertical position (above ground) estimate is good. | */
        //EKF_CONST_POS_MODE | /* EKF is in constant position mode and does not know it's absolute or relative position. | */
        EKF_PRED_POS_HORIZ_REL | /* Set if EKF's predicted horizontal position (relative) estimate is good. | */
        EKF_PRED_POS_HORIZ_ABS; /* Set if EKF's predicted horizontal position (absolute) estimate is good. | */
        mavlink_msg_ekf_status_report_send(chan, flags, 0, 0, 0, 0, 0, 0);
        }
        break;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.send_status_report(chan);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.send_status_report(chan);
#endif

    }
}

// passes a reference to the location of the inertial navigation origin
// in WGS-84 coordinates
// returns a boolean true when the inertial navigation origin has been set
bool AP_AHRS_NavEKF::get_origin(Location &ret) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (!EKF2.getOriginLLH(-1,ret)) {
            return false;
        }
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (!EKF3.getOriginLLH(-1,ret)) {
            return false;
        }
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        if (!_sitl) {
            return false;
        }
        const struct SITL::sitl_fdm &fdm = _sitl->state;
        ret = fdm.home;
        return true;
#endif
    }

    return false;
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return false when no limiting is required
bool AP_AHRS_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no limiting applies
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getHeightControlLimit(limit);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getHeightControlLimit(limit);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return false;
#endif
    }

    return false;
}

// get_location - updates the provided location with the latest calculated location
//  returns true on success (i.e. the EKF knows it's latest position), false on failure
bool AP_AHRS_NavEKF::get_location(struct Location &loc) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no data
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLLH(loc);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLLH(loc);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return get_position(loc);
#endif
    }

    return false;
}

// return the innovations for the primariy EKF
// boolean false is returned if innovations are not available
bool AP_AHRS_NavEKF::get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no data
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // use EKF to get innovations
        EKF2.getInnovations(-1, velInnov, posInnov, magInnov, tasInnov, yawInnov);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // use EKF to get innovations
        EKF3.getInnovations(-1, velInnov, posInnov, magInnov, tasInnov, yawInnov);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        velInnov.zero();
        posInnov.zero();
        magInnov.zero();
        tasInnov = 0.0f;
        yawInnov = 0.0f;
        return true;
#endif
    }

    return false;
}

// get_variances - provides the innovations normalised using the innovation variance where a value of 0
// indicates prefect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
// inconsistency that will be accpeted by the filter
// boolean false is returned if variances are not available
bool AP_AHRS_NavEKF::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const
{
    switch (ekf_type()) {
    case EKFType::NONE:
        // We are not using an EKF so no data
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // use EKF to get variance
        EKF2.getVariances(-1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // use EKF to get variance
        EKF3.getVariances(-1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        return true;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        velVar = 0;
        posVar = 0;
        hgtVar = 0;
        magVar.zero();
        tasVar = 0;
        offset.zero();
        return true;
#endif
    }

    return false;
}

void AP_AHRS_NavEKF::setTakeoffExpected(bool val)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.setTakeoffExpected(val);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.setTakeoffExpected(val);
#endif
}

void AP_AHRS_NavEKF::setTouchdownExpected(bool val)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.setTouchdownExpected(val);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.setTouchdownExpected(val);
#endif
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
    case EKFType::NONE:
        return false;

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.have_ekf_logging();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.have_ekf_logging();
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// get the index of the current primary IMU
uint8_t AP_AHRS_NavEKF::get_primary_IMU_index() const
{
    int8_t imu = -1;
    switch (ekf_type()) {
    case EKFType::NONE:
        break;
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // let EKF2 choose primary IMU
        imu = EKF2.getPrimaryCoreIMUIndex();
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // let EKF2 choose primary IMU
        imu = EKF3.getPrimaryCoreIMUIndex();
        break;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        break;
#endif
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
    if (ekf_type() != EKFType::NONE) {
        return get_primary_IMU_index();
    }
    return AP::ins().get_primary_accel();
}

// get the index of the current primary gyro sensor
uint8_t AP_AHRS_NavEKF::get_primary_gyro_index(void) const
{
    if (ekf_type() != EKFType::NONE) {
        return get_primary_IMU_index();
    }
    return AP::ins().get_primary_gyro();
}

// see if EKF lane switching is possible to avoid EKF failsafe
void AP_AHRS_NavEKF::check_lane_switch(void)
{
    switch (active_EKF_type()) {
    case EKFType::NONE:
        break;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
        break;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.checkLaneSwitch();
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.checkLaneSwitch();
        break;
#endif
    }
}

void AP_AHRS_NavEKF::Log_Write()
{
#if HAL_NAVEKF2_AVAILABLE
    get_NavEKF2().Log_Write();
#endif
#if HAL_NAVEKF3_AVAILABLE
    get_NavEKF3().Log_Write();
#endif
}

AP_AHRS_NavEKF &AP::ahrs_navekf()
{
    return static_cast<AP_AHRS_NavEKF&>(*AP_AHRS::get_singleton());
}

// check whether compass can be bypassed for arming check in case when external navigation data is available 
bool AP_AHRS_NavEKF::is_ext_nav_used_for_yaw(void) const
{
    switch (active_EKF_type()) {
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.isExtNavUsedForYaw();
#endif
    case EKFType::NONE:
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    case EKFType::SITL:
#endif
        return false; 
    }
    // since there is no default case above, this is unreachable
    return false;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

