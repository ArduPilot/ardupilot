/*
 *       APM_AHRS_DCM.cpp
 *
 *       AHRS system using DCM matrices
 *
 *       Based on DCM code by Doug Weibel, Jordi Munoz and Jose Julio. DIYDrones.com
 *
 *       Adapted for the general ArduPilot AHRS interface by Andrew Tridgell

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
#include "AP_AHRS.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// this is the speed in m/s above which we first get a yaw lock with
// the GPS
#define GPS_SPEED_MIN 3

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void
AP_AHRS_DCM::reset_gyro_drift(void)
{
    _omega_I.zero();
    _omega_I_sum.zero();
    _omega_I_sum_time = 0;
}


/* if this was a watchdog reset then get home from backup registers */
void AP_AHRS::load_watchdog_home()
{
    const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    if (hal.util->was_watchdog_reset() && (pd.home_lat != 0 || pd.home_lon != 0)) {
        _home.lat = pd.home_lat;
        _home.lng = pd.home_lon;
        _home.set_alt_cm(pd.home_alt_cm, Location::AltFrame::ABSOLUTE);
        _home_is_set = true;
        _home_locked = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Restored watchdog home");
    }
}


// run a full DCM update round
void
AP_AHRS_DCM::update()
{
    AP_InertialSensor &_ins = AP::ins();

    // ask the IMU how much time this sensor reading represents
    const float delta_t = _ins.get_delta_time();

    // if the update call took more than 0.2 seconds then discard it,
    // otherwise we may move too far. This happens when arming motors
    // in ArduCopter
    if (delta_t > 0.2f) {
        memset((void *)&_ra_sum[0], 0, sizeof(_ra_sum));
        _ra_deltat = 0;
        return;
    }

    // Integrate the DCM matrix using gyro inputs
    matrix_update(delta_t);

    // Normalize the DCM matrix
    normalize();

    // Perform drift correction
    drift_correction(delta_t);

    // paranoid check for bad values in the DCM matrix
    check_matrix();

    // calculate the euler angles and DCM matrix which will be used
    // for high level navigation control. Apply trim such that a
    // positive trim value results in a positive vehicle rotation
    // about that axis (ie a negative offset)
    _body_dcm_matrix = _dcm_matrix * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body();
    _body_dcm_matrix.to_euler(&roll, &pitch, &yaw);

    // pre-calculate some trig for CPU purposes:
    _cos_yaw = cosf(yaw);
    _sin_yaw = sinf(yaw);

    backup_attitude();

    // remember the last origin for fallback support
    IGNORE_RETURN(AP::ahrs().get_origin(last_origin));
}

void AP_AHRS_DCM::get_results(AP_AHRS_Backend::Estimates &results)
{
    results.roll_rad = roll;
    results.pitch_rad = pitch;
    results.yaw_rad = yaw;

    results.dcm_matrix = _body_dcm_matrix;
    results.gyro_estimate = _omega;
    results.gyro_drift = _omega_I;

    const uint8_t to_copy = MIN(sizeof(results.accel_ef), sizeof(_accel_ef));
    memcpy(results.accel_ef, _accel_ef, to_copy);
    results.accel_ef_blended = _accel_ef_blended;
}

/*
  backup attitude to persistent_data for use in watchdog reset
 */
void AP_AHRS_DCM::backup_attitude(void)
{
    AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    pd.roll_rad = roll;
    pd.pitch_rad = pitch;
    pd.yaw_rad = yaw;
}

// update the DCM matrix using only the gyros
void
AP_AHRS_DCM::matrix_update(float _G_Dt)
{
    // note that we do not include the P terms in _omega. This is
    // because the spin_rate is calculated from _omega.length(),
    // and including the P terms would give positive feedback into
    // the _P_gain() calculation, which can lead to a very large P
    // value
    _omega.zero();

    // average across first two healthy gyros. This reduces noise on
    // systems with more than one gyro. We don't use the 3rd gyro
    // unless another is unhealthy as 3rd gyro on PH2 has a lot more
    // noise
    uint8_t healthy_count = 0;
    Vector3f delta_angle;
    const AP_InertialSensor &_ins = AP::ins();
    for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
        if (_ins.use_gyro(i) && healthy_count < 2) {
            Vector3f dangle;
            float dangle_dt;
            if (_ins.get_delta_angle(i, dangle, dangle_dt)) {
                healthy_count++;
                delta_angle += dangle;
            }
        }
    }
    if (healthy_count > 1) {
        delta_angle /= healthy_count;
    }
    if (_G_Dt > 0) {
        _omega = delta_angle / _G_Dt;
        _omega += _omega_I;
        _dcm_matrix.rotate((_omega + _omega_P + _omega_yaw_P) * _G_Dt);
    }
}


/*
 *  reset the DCM matrix and omega. Used on ground start, and on
 *  extreme errors in the matrix
 */
void
AP_AHRS_DCM::reset(bool recover_eulers)
{
    // reset the integration terms
    _omega_I.zero();
    _omega_P.zero();
    _omega_yaw_P.zero();
    _omega.zero();

    // if the caller wants us to try to recover to the current
    // attitude then calculate the dcm matrix from the current
    // roll/pitch/yaw values
    if (hal.util->was_watchdog_reset() && AP_HAL::millis() < 10000) {
        const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
        roll = pd.roll_rad;
        pitch = pd.pitch_rad;
        yaw = pd.yaw_rad;
        _dcm_matrix.from_euler(roll, pitch, yaw);
        gcs().send_text(MAV_SEVERITY_INFO, "Restored watchdog attitude %.0f %.0f %.0f",
                        degrees(roll), degrees(pitch), degrees(yaw));
    } else if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
        _dcm_matrix.from_euler(roll, pitch, yaw);
    } else {

        // Use the measured accel due to gravity to calculate an initial
        // roll and pitch estimate

        AP_InertialSensor &_ins = AP::ins();

        // Get body frame accel vector
        Vector3f initAccVec = _ins.get_accel();
        uint8_t counter = 0;

        // the first vector may be invalid as the filter starts up
        while ((initAccVec.length() < 9.0f || initAccVec.length() > 11) && counter++ < 20) {
            _ins.wait_for_sample();
            _ins.update();
            initAccVec = _ins.get_accel();
        }

        // normalise the acceleration vector
        if (initAccVec.length() > 5.0f) {
            // calculate initial pitch angle
            pitch = atan2f(initAccVec.x, norm(initAccVec.y, initAccVec.z));
            // calculate initial roll angle
            roll = atan2f(-initAccVec.y, -initAccVec.z);
        } else {
            // If we can't use the accel vector, then align flat
            roll = 0.0f;
            pitch = 0.0f;
        }
        _dcm_matrix.from_euler(roll, pitch, 0);

    }

    // pre-calculate some trig for CPU purposes:
    _cos_yaw = cosf(yaw);
    _sin_yaw = sinf(yaw);

    _last_startup_ms = AP_HAL::millis();
}

/*
 *  check the DCM matrix for pathological values
 */
void
AP_AHRS_DCM::check_matrix(void)
{
    if (_dcm_matrix.is_nan()) {
        //Serial.printf("ERROR: DCM matrix NAN\n");
        AP_AHRS_DCM::reset(true);
        return;
    }
    // some DCM matrix values can lead to an out of range error in
    // the pitch calculation via asin().  These NaN values can
    // feed back into the rest of the DCM matrix via the
    // error_course value.
    if (!(_dcm_matrix.c.x < 1.0f &&
            _dcm_matrix.c.x > -1.0f)) {
        // We have an invalid matrix. Force a normalisation.
        normalize();

        if (_dcm_matrix.is_nan() ||
                fabsf(_dcm_matrix.c.x) > 10.0) {
            // See Issue #20284: regarding the selection of 10.0 for DCM reset
            // This won't be lowered without evidence of an issue or mathematical proof & testing of a lower bound

            // normalisation didn't fix the problem! We're
            // in real trouble. All we can do is reset
            //Serial.printf("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
            //	   _dcm_matrix.c.x);
            AP_AHRS_DCM::reset(true);
        }
    }
}

// renormalise one vector component of the DCM matrix
// this will return false if renormalization fails
bool
AP_AHRS_DCM::renorm(Vector3f const &a, Vector3f &result)
{
    // numerical errors will slowly build up over time in DCM,
    // causing inaccuracies. We can keep ahead of those errors
    // using the renormalization technique from the DCM IMU paper
    // (see equations 18 to 21).

    // For APM we don't bother with the taylor expansion
    // optimisation from the paper as on our 2560 CPU the cost of
    // the sqrt() is 44 microseconds, and the small time saving of
    // the taylor expansion is not worth the potential of
    // additional error buildup.

    // Note that we can get significant renormalisation values
    // when we have a larger delta_t due to a glitch eleswhere in
    // APM, such as a I2c timeout or a set of EEPROM writes. While
    // we would like to avoid these if possible, if it does happen
    // we don't want to compound the error by making DCM less
    // accurate.

    const float renorm_val = 1.0f / a.length();

    // keep the average for reporting
    _renorm_val_sum += renorm_val;
    _renorm_val_count++;

    if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
        // this is larger than it should get - log it as a warning
        if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
            // we are getting values which are way out of
            // range, we will reset the matrix and hope we
            // can recover our attitude using drift
            // correction before we hit the ground!
            //Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
            //	   renorm_val);
            return false;
        }
    }

    result = a * renorm_val;
    return true;
}

/*************************************************
 *  Direction Cosine Matrix IMU: Theory
 *  William Premerlani and Paul Bizard
 *
 *  Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
 *  to approximations rather than identities. In effect, the axes in the two frames of reference no
 *  longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
 *  simple matter to stay ahead of it.
 *  We call the process of enforcing the orthogonality conditions: renormalization.
 */
void
AP_AHRS_DCM::normalize(void)
{
    const float error = _dcm_matrix.a * _dcm_matrix.b;                                 // eq.18

    const Vector3f t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));              // eq.19
    const Vector3f t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));              // eq.19
    const Vector3f t2 = t0 % t1;                                                       // c= a x b // eq.20

    if (!renorm(t0, _dcm_matrix.a) ||
            !renorm(t1, _dcm_matrix.b) ||
            !renorm(t2, _dcm_matrix.c)) {
        // Our solution is blowing up and we will force back
        // to last euler angles
        _last_failure_ms = AP_HAL::millis();
        AP_AHRS_DCM::reset(true);
    }
}


// produce a yaw error value. The returned value is proportional
// to sin() of the current heading error in earth frame
float
AP_AHRS_DCM::yaw_error_compass(Compass &compass)
{
    const Vector3f &mag = compass.get_field();
    // get the mag vector in the earth frame
    Vector2f rb = _dcm_matrix.mulXY(mag);

    if (rb.length() < FLT_EPSILON) {
        return 0.0f;
    }

    rb.normalize();
    if (rb.is_inf()) {
        // not a valid vector
        return 0.0f;
    }

    // update vector holding earths magnetic field (if required)
    if( !is_equal(_last_declination, compass.get_declination()) ) {
        _last_declination = compass.get_declination();
        _mag_earth.x = cosf(_last_declination);
        _mag_earth.y = sinf(_last_declination);
    }

    // calculate the error term in earth frame
    // calculate the Z component of the cross product of rb and _mag_earth
    return rb % _mag_earth;
}

// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float
AP_AHRS_DCM::_P_gain(float spin_rate)
{
    if (spin_rate < ToRad(50)) {
        return 1.0f;
    }
    if (spin_rate > ToRad(500)) {
        return 10.0f;
    }
    return spin_rate/ToRad(50);
}

// _yaw_gain reduces the gain of the PI controller applied to heading errors
// when observability from change of velocity is good (eg changing speed or turning)
// This reduces unwanted roll and pitch coupling due to compass errors for planes.
// High levels of noise on _accel_ef will cause the gain to drop and could lead to
// increased heading drift during straight and level flight, however some gain is
// always available. TODO check the necessity of adding adjustable acc threshold
// and/or filtering accelerations before getting magnitude
float
AP_AHRS_DCM::_yaw_gain(void) const
{
    const float VdotEFmag = _accel_ef[_active_accel_instance].xy().length();

    if (VdotEFmag <= 4.0f) {
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}


// return true if we have and should use GPS
bool AP_AHRS_DCM::have_gps(void) const
{
    if (AP::gps().status() <= AP_GPS::NO_FIX || _gps_use == GPSUse::Disable) {
        return false;
    }
    return true;
}

/*
  when we are getting the initial attitude we want faster gains so
  that if the board starts upside down we quickly approach the right
  attitude.
  We don't want to keep those high gains for too long though as high P
  gains cause slow gyro offset learning. So we keep the high gains for
  a maximum of 20 seconds
 */
bool AP_AHRS_DCM::use_fast_gains(void) const
{
    return !hal.util->get_soft_armed() && (AP_HAL::millis() - _last_startup_ms) < 20000U;
}


// return true if we should use the compass for yaw correction
bool AP_AHRS_DCM::use_compass(void)
{
    const Compass &compass = AP::compass();

    if (!compass.use_for_yaw()) {
        // no compass available
        return false;
    }
    if (!AP::ahrs().get_fly_forward() || !have_gps()) {
        // we don't have any alterative to the compass
        return true;
    }
    if (AP::gps().ground_speed() < GPS_SPEED_MIN) {
        // we are not going fast enough to use the GPS
        return true;
    }

    // if the current yaw differs from the GPS yaw by more than 45
    // degrees and the estimated wind speed is less than 80% of the
    // ground speed, then switch to GPS navigation. This will help
    // prevent flyaways with very bad compass offsets
    const float error = fabsf(wrap_180(degrees(yaw) - AP::gps().ground_course()));
    if (error > 45 && _wind.length() < AP::gps().ground_speed()*0.8f) {
        if (AP_HAL::millis() - _last_consistent_heading > 2000) {
            // start using the GPS for heading if the compass has been
            // inconsistent with the GPS for 2 seconds
            return false;
        }
    } else {
        _last_consistent_heading = AP_HAL::millis();
    }

    // use the compass
    return true;
}

// return the quaternion defining the rotation from NED to XYZ (body) axes
bool AP_AHRS_DCM::get_quaternion(Quaternion &quat) const
{
    quat.from_rotation_matrix(_dcm_matrix);
    return true;
}

// yaw drift correction using the compass or GPS
// this function prodoces the _omega_yaw_P vector, and also
// contributes to the _omega_I.z long term yaw drift estimate
void
AP_AHRS_DCM::drift_correction_yaw(void)
{
    bool new_value = false;
    float yaw_error;
    float yaw_deltat;

    const AP_GPS &_gps = AP::gps();

    Compass &compass = AP::compass();

    if (compass.is_calibrating()) {
        // don't do any yaw correction while calibrating
        return;
    }
    
    if (AP_AHRS_DCM::use_compass()) {
        /*
          we are using compass for yaw
         */
        if (compass.last_update_usec() != _compass_last_update) {
            yaw_deltat = (compass.last_update_usec() - _compass_last_update) * 1.0e-6f;
            _compass_last_update = compass.last_update_usec();
            // we force an additional compass read()
            // here. This has the effect of throwing away
            // the first compass value, which can be bad
            if (!have_initial_yaw && compass.read()) {
                const float heading = compass.calculate_heading(_dcm_matrix);
                _dcm_matrix.from_euler(roll, pitch, heading);
                _omega_yaw_P.zero();
                have_initial_yaw = true;
            }
            new_value = true;
            yaw_error = yaw_error_compass(compass);

            // also update the _gps_last_update, so if we later
            // disable the compass due to significant yaw error we
            // don't suddenly change yaw with a reset
            _gps_last_update = _gps.last_fix_time_ms();
        }
    } else if (AP::ahrs().get_fly_forward() && have_gps()) {
        /*
          we are using GPS for yaw
         */
        if (_gps.last_fix_time_ms() != _gps_last_update &&
                _gps.ground_speed() >= GPS_SPEED_MIN) {
            yaw_deltat = (_gps.last_fix_time_ms() - _gps_last_update) * 1.0e-3f;
            _gps_last_update = _gps.last_fix_time_ms();
            new_value = true;
            const float gps_course_rad = ToRad(_gps.ground_course());
            const float yaw_error_rad = wrap_PI(gps_course_rad - yaw);
            yaw_error = sinf(yaw_error_rad);

            /* reset yaw to match GPS heading under any of the
               following 3 conditions:

               1) if we have reached GPS_SPEED_MIN and have never had
               yaw information before

               2) if the last time we got yaw information from the GPS
               is more than 20 seconds ago, which means we may have
               suffered from considerable gyro drift

               3) if we are over 3*GPS_SPEED_MIN (which means 9m/s)
               and our yaw error is over 60 degrees, which means very
               poor yaw. This can happen on bungee launch when the
               operator pulls back the plane rapidly enough then on
               release the GPS heading changes very rapidly
            */
            if (!have_initial_yaw ||
                    yaw_deltat > 20 ||
                    (_gps.ground_speed() >= 3*GPS_SPEED_MIN && fabsf(yaw_error_rad) >= 1.047f)) {
                // reset DCM matrix based on current yaw
                _dcm_matrix.from_euler(roll, pitch, gps_course_rad);
                _omega_yaw_P.zero();
                have_initial_yaw = true;
                yaw_error = 0;
            }
        }
    }

    if (!new_value) {
        // we don't have any new yaw information
        // slowly decay _omega_yaw_P to cope with loss
        // of our yaw source
        _omega_yaw_P *= 0.97f;
        return;
    }

    // convert the error vector to body frame
    const float error_z = _dcm_matrix.c.z * yaw_error;

    // the spin rate changes the P gain, and disables the
    // integration at higher rates
    const float spin_rate = _omega.length();

    // sanity check _kp_yaw
    if (_kp_yaw < AP_AHRS_YAW_P_MIN) {
        _kp_yaw.set(AP_AHRS_YAW_P_MIN);
    }

    // update the proportional control to drag the
    // yaw back to the right value. We use a gain
    // that depends on the spin rate. See the fastRotations.pdf
    // paper from Bill Premerlani
    // We also adjust the gain depending on the rate of change of horizontal velocity which
    // is proportional to how observable the heading is from the acceerations and GPS velocity
    // The accelration derived heading will be more reliable in turns than compass or GPS

    _omega_yaw_P.z = error_z * _P_gain(spin_rate) * _kp_yaw * _yaw_gain();
    if (use_fast_gains()) {
        _omega_yaw_P.z *= 8;
    }

    // don't update the drift term if we lost the yaw reference
    // for more than 2 seconds
    if (yaw_deltat < 2.0f && spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        // also add to the I term
        _omega_I_sum.z += error_z * _ki_yaw * yaw_deltat;
    }

    _error_yaw = 0.8f * _error_yaw + 0.2f * fabsf(yaw_error);
}


/**
   return an accel vector delayed by AHRS_ACCEL_DELAY samples for a
   specific accelerometer instance
 */
Vector3f AP_AHRS_DCM::ra_delayed(uint8_t instance, const Vector3f &ra)
{
    // get the old element, and then fill it with the new element
    const Vector3f ret = _ra_delay_buffer[instance];
    _ra_delay_buffer[instance] = ra;
    if (ret.is_zero()) {
        // use the current vector if the previous vector is exactly
        // zero. This prevents an error on initialisation
        return ra;
    }
    return ret;
}

/* returns true if attitude should be corrected from GPS-derived
 * velocity-deltas.  We turn this off for Copter and other similar
 * vehicles while the vehicle is disarmed to avoid the HUD bobbing
 * around while the vehicle is disarmed.
 */
bool AP_AHRS_DCM::should_correct_centrifugal() const
{
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduSub) || APM_BUILD_TYPE(APM_BUILD_Blimp)
    return hal.util->get_soft_armed();
#endif

    return true;
}

// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This drift correction implementation is based on a paper
// by Bill Premerlani from here:
//   http://gentlenav.googlecode.com/files/RollPitchDriftCompensation.pdf
void
AP_AHRS_DCM::drift_correction(float deltat)
{
    Vector3f velocity;
    uint32_t last_correction_time;

    // perform yaw drift correction if we have a new yaw reference
    // vector
    drift_correction_yaw();

    const AP_InertialSensor &_ins = AP::ins();

    // rotate accelerometer values into the earth frame
    uint8_t healthy_count = 0;
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        if (_ins.use_accel(i) && healthy_count < 2) {
            /*
              by using get_delta_velocity() instead of get_accel() the
              accel value is sampled over the right time delta for
              each sensor, which prevents an aliasing effect
             */
            Vector3f delta_velocity;
            float delta_velocity_dt;
            _ins.get_delta_velocity(i, delta_velocity, delta_velocity_dt);
            if (delta_velocity_dt > 0) {
                _accel_ef[i] = _dcm_matrix * (delta_velocity / delta_velocity_dt);
                // integrate the accel vector in the earth frame between GPS readings
                _ra_sum[i] += _accel_ef[i] * deltat;
            }
            healthy_count++;
        }
    }

    //update _accel_ef_blended
#if INS_MAX_INSTANCES > 1
    if (_ins.get_accel_count() == 2 && _ins.use_accel(0) && _ins.use_accel(1)) {
        const float imu1_weight_target = _active_accel_instance == 0 ? 1.0f : 0.0f;
        // slew _imu1_weight over one second
        _imu1_weight += constrain_float(imu1_weight_target-_imu1_weight, -deltat, deltat);
        _accel_ef_blended = _accel_ef[0] * _imu1_weight + _accel_ef[1] * (1.0f - _imu1_weight);
    } else
#endif
    {
        _accel_ef_blended = _accel_ef[_ins.get_primary_accel()];
    }

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;

    const AP_GPS &_gps = AP::gps();
    const bool fly_forward = AP::ahrs().get_fly_forward();

    if (!have_gps() ||
            _gps.status() < AP_GPS::GPS_OK_FIX_3D ||
            _gps.num_sats() < _gps_minsats) {
        // no GPS, or not a good lock. From experience we need at
        // least 6 satellites to get a really reliable velocity number
        // from the GPS.
        //
        // As a fallback we use the fixed wing acceleration correction
        // if we have an airspeed estimate (which we only have if
        // _fly_forward is set), otherwise no correction
        if (_ra_deltat < 0.2f) {
            // not enough time has accumulated
            return;
        }

        float airspeed = _last_airspeed;
#if AP_AIRSPEED_ENABLED
        if (airspeed_sensor_enabled()) {
            airspeed = AP::airspeed()->get_airspeed();
        }
#endif

        // use airspeed to estimate our ground velocity in
        // earth frame by subtracting the wind
        velocity = _dcm_matrix.colx() * airspeed;

        // add in wind estimate
        velocity += _wind;

        last_correction_time = AP_HAL::millis();
        _have_gps_lock = false;
    } else {
        if (_gps.last_fix_time_ms() == _ra_sum_start) {
            // we don't have a new GPS fix - nothing more to do
            return;
        }
        velocity = _gps.velocity();
        last_correction_time = _gps.last_fix_time_ms();
        if (_have_gps_lock == false) {
            // if we didn't have GPS lock in the last drift
            // correction interval then set the velocities equal
            _last_velocity = velocity;
        }
        _have_gps_lock = true;

        // keep last airspeed estimate for dead-reckoning purposes
        Vector3f airspeed = velocity - _wind;

        // rotate vector to body frame
        airspeed = _body_dcm_matrix.mul_transpose(airspeed);

        // take positive component in X direction. This mimics a pitot
        // tube
        _last_airspeed = MAX(airspeed.x, 0);
    }

    if (have_gps()) {
        // use GPS for positioning with any fix, even a 2D fix
        _last_lat = _gps.location().lat;
        _last_lng = _gps.location().lng;
        _last_pos_ms = AP_HAL::millis();
        _position_offset_north = 0;
        _position_offset_east = 0;

        // once we have a single GPS lock, we can update using
        // dead-reckoning from then on
        _have_position = true;
    } else {
        // update dead-reckoning position estimate
        _position_offset_north += velocity.x * _ra_deltat;
        _position_offset_east  += velocity.y * _ra_deltat;
    }

    // see if this is our first time through - in which case we
    // just setup the start times and return
    if (_ra_sum_start == 0) {
        _ra_sum_start = last_correction_time;
        _last_velocity = velocity;
        return;
    }

    // equation 9: get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    Vector3f GA_e(0.0f, 0.0f, -1.0f);

    if (_ra_deltat <= 0) {
        // waiting for more data
        return;
    }
    
    bool using_gps_corrections = false;
    float ra_scale = 1.0f/(_ra_deltat*GRAVITY_MSS);

    if (should_correct_centrifugal() && (_have_gps_lock || fly_forward)) {
        const float v_scale = gps_gain.get() * ra_scale;
        const Vector3f vdelta = (velocity - _last_velocity) * v_scale;
        GA_e += vdelta;
        GA_e.normalize();
        if (GA_e.is_inf()) {
            // wait for some non-zero acceleration information
            _last_failure_ms = AP_HAL::millis();
            return;
        }
        using_gps_corrections = true;
    }

    // calculate the error term in earth frame.
    // we do this for each available accelerometer then pick the
    // accelerometer that leads to the smallest error term. This takes
    // advantage of the different sample rates on different
    // accelerometers to dramatically reduce the impact of aliasing
    // due to harmonics of vibrations that match closely the sampling
    // rate of our accelerometers. On the Pixhawk we have the LSM303D
    // running at 800Hz and the MPU6000 running at 1kHz, by combining
    // the two the effects of aliasing are greatly reduced.
    Vector3f error[INS_MAX_INSTANCES];
    float error_dirn[INS_MAX_INSTANCES];
    Vector3f GA_b[INS_MAX_INSTANCES];
    int8_t besti = -1;
    float best_error = 0;
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        if (!_ins.get_accel_health(i)) {
            // only use healthy sensors
            continue;
        }
        _ra_sum[i] *= ra_scale;

        // get the delayed ra_sum to match the GPS lag
        if (using_gps_corrections) {
            GA_b[i] = ra_delayed(i, _ra_sum[i]);
        } else {
            GA_b[i] = _ra_sum[i];
        }
        if (GA_b[i].is_zero()) {
            // wait for some non-zero acceleration information
            continue;
        }
        GA_b[i].normalize();
        if (GA_b[i].is_inf()) {
            // wait for some non-zero acceleration information
            continue;
        }
        error[i] = GA_b[i] % GA_e;
        // Take dot product to catch case vectors are opposite sign and parallel
        error_dirn[i] = GA_b[i] * GA_e;
        const float error_length = error[i].length();
        if (besti == -1 || error_length < best_error) {
            besti = i;
            best_error = error_length;
        }
        // Catch case where orientation is 180 degrees out
        if (error_dirn[besti] < 0.0f) {
            best_error = 1.0f;
        }

    }

    if (besti == -1) {
        // no healthy accelerometers!
        _last_failure_ms = AP_HAL::millis();
        return;
    }

    _active_accel_instance = besti;

#define YAW_INDEPENDENT_DRIFT_CORRECTION 0
#if YAW_INDEPENDENT_DRIFT_CORRECTION
    // step 2 calculate earth_error_Z
    const float earth_error_Z = error.z;

    // equation 10
    const float tilt = GA_e.xy().length();

    // equation 11
    const float theta = atan2f(GA_b[besti].y, GA_b[besti].x);

    // equation 12
    const Vector3f GA_e2 = Vector3f(cosf(theta)*tilt, sinf(theta)*tilt, GA_e.z);

    // step 6
    error = GA_b[besti] % GA_e2;
    error.z = earth_error_Z;
#endif // YAW_INDEPENDENT_DRIFT_CORRECTION

    // to reduce the impact of two competing yaw controllers, we
    // reduce the impact of the gps/accelerometers on yaw when we are
    // flat, but still allow for yaw correction using the
    // accelerometers at high roll angles as long as we have a GPS
    if (AP_AHRS_DCM::use_compass()) {
        if (have_gps() && is_equal(gps_gain.get(), 1.0f)) {
            error[besti].z *= sinf(fabsf(roll));
        } else {
            error[besti].z = 0;
        }
    }

    // if ins is unhealthy then stop attitude drift correction and
    // hope the gyros are OK for a while. Just slowly reduce _omega_P
    // to prevent previous bad accels from throwing us off
    if (!_ins.healthy()) {
        error[besti].zero();
    } else {
        // convert the error term to body frame
        error[besti] = _dcm_matrix.mul_transpose(error[besti]);
    }

    if (error[besti].is_nan() || error[besti].is_inf()) {
        // don't allow bad values
        check_matrix();
        _last_failure_ms = AP_HAL::millis();
        return;
    }

    _error_rp = 0.8f * _error_rp + 0.2f * best_error;

    // base the P gain on the spin rate
    const float spin_rate = _omega.length();

    // sanity check _kp value
    if (_kp < AP_AHRS_RP_P_MIN) {
        _kp.set(AP_AHRS_RP_P_MIN);
    }

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P = error[besti] * _P_gain(spin_rate) * _kp;
    if (use_fast_gains()) {
        _omega_P *= 8;
    }

    if (fly_forward && _gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
            _gps.ground_speed() < GPS_SPEED_MIN &&
            _ins.get_accel().x >= 7 &&
        pitch > radians(-30) && pitch < radians(30)) {
        // assume we are in a launch acceleration, and reduce the
        // rp gain by 50% to reduce the impact of GPS lag on
        // takeoff attitude when using a catapult
        _omega_P *= 0.5f;
    }

    // accumulate some integrator error
    if (spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        _omega_I_sum += error[besti] * _ki * _ra_deltat;
        _omega_I_sum_time += _ra_deltat;
    }

    if (_omega_I_sum_time >= 5) {
        // limit the rate of change of omega_I to the hardware
        // reported maximum gyro drift rate. This ensures that
        // short term errors don't cause a buildup of omega_I
        // beyond the physical limits of the device
        const float change_limit = AP::ins().get_gyro_drift_rate() * _omega_I_sum_time;
        _omega_I_sum.x = constrain_float(_omega_I_sum.x, -change_limit, change_limit);
        _omega_I_sum.y = constrain_float(_omega_I_sum.y, -change_limit, change_limit);
        _omega_I_sum.z = constrain_float(_omega_I_sum.z, -change_limit, change_limit);
        _omega_I += _omega_I_sum;
        _omega_I_sum.zero();
        _omega_I_sum_time = 0;
    }

    // zero our accumulator ready for the next GPS step
    memset((void *)&_ra_sum[0], 0, sizeof(_ra_sum));
    _ra_deltat = 0;
    _ra_sum_start = last_correction_time;

    // remember the velocity for next time
    _last_velocity = velocity;
}


// update our wind speed estimate
void AP_AHRS_DCM::estimate_wind(void)
{
    if (!AP::ahrs().get_wind_estimation_enabled()) {
        return;
    }
    const Vector3f &velocity = _last_velocity;

    // this is based on the wind speed estimation code from MatrixPilot by
    // Bill Premerlani. Adaption for ArduPilot by Jon Challinger
    // See http://gentlenav.googlecode.com/files/WindEstimation.pdf
    const Vector3f fuselageDirection = _dcm_matrix.colx();
    const Vector3f fuselageDirectionDiff = fuselageDirection - _last_fuse;
    const uint32_t now = AP_HAL::millis();

    // scrap our data and start over if we're taking too long to get a direction change
    if (now - _last_wind_time > 10000) {
        _last_wind_time = now;
        _last_fuse = fuselageDirection;
        _last_vel = velocity;
        return;
    }

    float diff_length = fuselageDirectionDiff.length();
    if (diff_length > 0.2f) {
        // when turning, use the attitude response to estimate
        // wind speed
        float V;
        const Vector3f velocityDiff = velocity - _last_vel;

        // estimate airspeed it using equation 6
        V = velocityDiff.length() / diff_length;

        const Vector3f fuselageDirectionSum = fuselageDirection + _last_fuse;
        const Vector3f velocitySum = velocity + _last_vel;

        _last_fuse = fuselageDirection;
        _last_vel = velocity;

        const float theta = atan2f(velocityDiff.y, velocityDiff.x) - atan2f(fuselageDirectionDiff.y, fuselageDirectionDiff.x);
        const float sintheta = sinf(theta);
        const float costheta = cosf(theta);

        Vector3f wind = Vector3f();
        wind.x = velocitySum.x - V * (costheta * fuselageDirectionSum.x - sintheta * fuselageDirectionSum.y);
        wind.y = velocitySum.y - V * (sintheta * fuselageDirectionSum.x + costheta * fuselageDirectionSum.y);
        wind.z = velocitySum.z - V * fuselageDirectionSum.z;
        wind *= 0.5f;

        if (wind.length() < _wind.length() + 20) {
            _wind = _wind * 0.95f + wind * 0.05f;
        }

        _last_wind_time = now;

        return;
    }

#if AP_AIRSPEED_ENABLED
    if (now - _last_wind_time > 2000 && airspeed_sensor_enabled()) {
        // when flying straight use airspeed to get wind estimate if available
        const Vector3f airspeed = _dcm_matrix.colx() * AP::airspeed()->get_airspeed();
        const Vector3f wind = velocity - (airspeed * get_EAS2TAS());
        _wind = _wind * 0.92f + wind * 0.08f;
    }
#endif
}


// return our current position estimate using
// dead-reckoning or GPS
bool AP_AHRS_DCM::get_location(struct Location &loc) const
{
    loc.lat = _last_lat;
    loc.lng = _last_lng;
    const auto &baro = AP::baro();
    const auto &gps = AP::gps();
    if (_gps_use == GPSUse::EnableWithHeight &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        loc.alt = gps.location().alt;
    } else {
        loc.alt = baro.get_altitude() * 100 + AP::ahrs().get_home().alt;
    }
    loc.relative_alt = 0;
    loc.terrain_alt = 0;
    loc.offset(_position_offset_north, _position_offset_east);
    if (_have_position) {
        const uint32_t now = AP_HAL::millis();
        float dt = 0;
        gps.get_lag(dt);
        dt += constrain_float((now - _last_pos_ms) * 0.001, 0, 0.5);
        Vector2f dpos = _last_velocity.xy() * dt;
        loc.offset(dpos.x, dpos.y);
    }
    return _have_position;
}

bool AP_AHRS_DCM::airspeed_estimate(float &airspeed_ret) const
{
#if AP_AIRSPEED_ENABLED
    const auto *airspeed = AP::airspeed();
    if (airspeed != nullptr) {
        return airspeed_estimate(airspeed->get_primary(), airspeed_ret);
    }
#endif
    // airspeed_estimate will also make this nullptr check and act
    // appropriately when we call it with a dummy sensor ID.
    return airspeed_estimate(0, airspeed_ret);
}

// return an airspeed estimate:
//  - from a real sensor if available
//  - otherwise from a GPS-derived wind-triangle estimate (if GPS available)
//  - otherwise from a cached wind-triangle estimate value (but returning false)
bool AP_AHRS_DCM::airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const
{
    // airspeed_ret: will always be filled-in by get_unconstrained_airspeed_estimate which fills in airspeed_ret in this order:
    //               airspeed as filled-in by an enabled airsped sensor
    //               if no airspeed sensor: airspeed estimated using the GPS speed & wind_speed_estimation
    //               Or if none of the above, fills-in using the previous airspeed estimate
    // Return false: if we are using the previous airspeed estimate
    if (!get_unconstrained_airspeed_estimate(airspeed_index, airspeed_ret)) {
        return false;
    }

    const float _wind_max = AP::ahrs().get_max_wind();
    if (_wind_max > 0 && AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // constrain the airspeed by the ground speed
        // and AHRS_WIND_MAX
        const float gnd_speed = AP::gps().ground_speed();
        float true_airspeed = airspeed_ret * get_EAS2TAS();
        true_airspeed = constrain_float(true_airspeed,
                                        gnd_speed - _wind_max,
                                        gnd_speed + _wind_max);
        airspeed_ret = true_airspeed / get_EAS2TAS();
    }

    return true;
}

// airspeed_ret: will always be filled-in by get_unconstrained_airspeed_estimate which fills in airspeed_ret in this order:
//               airspeed as filled-in by an enabled airsped sensor
//               if no airspeed sensor: airspeed estimated using the GPS speed & wind_speed_estimation
//               Or if none of the above, fills-in using the previous airspeed estimate
// Return false: if we are using the previous airspeed estimate
bool AP_AHRS_DCM::get_unconstrained_airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const
{
#if AP_AIRSPEED_ENABLED
    if (airspeed_sensor_enabled(airspeed_index)) {
        airspeed_ret = AP::airspeed()->get_airspeed(airspeed_index);
        return true;
    }
#endif

    if (AP::ahrs().get_wind_estimation_enabled() && have_gps()) {
        // estimated via GPS speed and wind
        airspeed_ret = _last_airspeed;
        return true;
    }

    // Else give the last estimate, but return false.
    // This is used by the dead-reckoning code
    airspeed_ret = _last_airspeed;
    return false;
}

bool AP_AHRS::set_home(const Location &loc)
{
    WITH_SEMAPHORE(_rsem);
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0 && loc.alt == 0) {
        return false;
    }
    if (!loc.check_latlng()) {
        return false;
    }
    // home must always be global frame at the moment as .alt is
    // accessed directly by the vehicles and they may not be rigorous
    // in checking the frame type.
    Location tmp = loc;
    if (!tmp.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (!_home_is_set) {
        // record home is set
        AP::logger().Write_Event(LogEvent::SET_HOME);
    }
#endif

    _home = tmp;
    _home_is_set = true;

    Log_Write_Home_And_Origin();

    // send new home and ekf origin to GCS
    gcs().send_message(MSG_HOME);
    gcs().send_message(MSG_ORIGIN);

    AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    pd.home_lat = loc.lat;
    pd.home_lon = loc.lng;
    pd.home_alt_cm = loc.alt;

    return true;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_DCM::healthy(void) const
{
    // consider ourselves healthy if there have been no failures for 5 seconds
    return (_last_failure_ms == 0 || AP_HAL::millis() - _last_failure_ms > 5000);
}

/*
  return NED velocity if we have GPS lock
 */
bool AP_AHRS_DCM::get_velocity_NED(Vector3f &vec) const
{
    const AP_GPS &_gps = AP::gps();
    if (_gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return false;
    }
    vec = _gps.velocity();
    return true;
}

// Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
bool AP_AHRS_DCM::get_vert_pos_rate(float &velocity) const
{
    Vector3f velned;
    if (!get_velocity_NED(velned)) {
        return false;
    }
    velocity = velned.z;
    return true;
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
// requires_position should be true if horizontal position configuration should be checked (not used)
bool AP_AHRS_DCM::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not healthy");
        return false;
    }
    return true;
}

/*
  relative-origin functions for fallback in AP_InertialNav
*/
bool AP_AHRS_DCM::get_origin(Location &ret) const
{
    ret = last_origin;
    if (ret.is_zero()) {
        // use home if we never have had an origin
        ret = AP::ahrs().get_home();
    }
    return !ret.is_zero();
}

bool AP_AHRS_DCM::get_relative_position_NED_origin(Vector3f &posNED) const
{
    Location origin;
    if (!AP_AHRS_DCM::get_origin(origin)) {
        return false;
    }
    Location loc;
    if (!AP_AHRS_DCM::get_location(loc)) {
        return false;
    }
    posNED = origin.get_distance_NED(loc);
    return true;
}

bool AP_AHRS_DCM::get_relative_position_NE_origin(Vector2f &posNE) const
{
    Vector3f posNED;
    if (!AP_AHRS_DCM::get_relative_position_NED_origin(posNED)) {
        return false;
    }
    posNE = posNED.xy();
    return true;
}

bool AP_AHRS_DCM::get_relative_position_D_origin(float &posD) const
{
    Vector3f posNED;
    if (!AP_AHRS_DCM::get_relative_position_NED_origin(posNED)) {
        return false;
    }
    posD = posNED.z;
    return true;
}

void AP_AHRS_DCM::send_ekf_status_report(mavlink_channel_t chan) const
{
}
