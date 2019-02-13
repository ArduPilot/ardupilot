/*
 *       APM_AHRS_DCM.cpp
 *
 *       AHRS system using DCM matrices
 *
 *       Based on DCM code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
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

// run a full DCM update round
void
AP_AHRS_DCM::update(bool skip_ins_update)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    float delta_t;

    if (_last_startup_ms == 0) {
        _last_startup_ms = AP_HAL::millis();
    }

    if (!skip_ins_update) {
        // tell the IMU to grab some data
        AP::ins().update();
    }

    const AP_InertialSensor &_ins = AP::ins();

    // ask the IMU how much time this sensor reading represents
    delta_t = _ins.get_delta_time();

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

    // Calculate pitch, roll, yaw for stabilization and navigation
    euler_angles();

    // update trig values including _cos_roll, cos_pitch
    update_trig();

    // update AOA and SSA
    update_AOA_SSA();
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
        if (_ins.get_gyro_health(i) && healthy_count < 2) {
            Vector3f dangle;
            if (_ins.get_delta_angle(i, dangle)) {
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
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    // reset the integration terms
    _omega_I.zero();
    _omega_P.zero();
    _omega_yaw_P.zero();
    _omega.zero();

    // if the caller wants us to try to recover to the current
    // attitude then calculate the dcm matrix from the current
    // roll/pitch/yaw values
    if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
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

    _last_startup_ms = AP_HAL::millis();
}

// reset the current attitude, used by HIL
void AP_AHRS_DCM::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    _dcm_matrix.from_euler(_roll, _pitch, _yaw);
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
                fabsf(_dcm_matrix.c.x) > 10) {
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
 *  We call the process of enforcing the orthogonality conditions ÒrenormalizationÓ.
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
AP_AHRS_DCM::yaw_error_compass(void)
{
    const Vector3f &mag = _compass->get_field();
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
    if( !is_equal(_last_declination,_compass->get_declination()) ) {
        _last_declination = _compass->get_declination();
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
    const float VdotEFmag = norm(_accel_ef[_active_accel_instance].x,
                                   _accel_ef[_active_accel_instance].y);
    if (VdotEFmag <= 4.0f) {
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}


// return true if we have and should use GPS
bool AP_AHRS_DCM::have_gps(void) const
{
    if (AP::gps().status() <= AP_GPS::NO_FIX || !_gps_use) {
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
    if (!_compass || !_compass->use_for_yaw()) {
        // no compass available
        return false;
    }
    if (!_flags.fly_forward || !have_gps()) {
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
    const int32_t error = abs(wrap_180_cd(yaw_sensor - AP::gps().ground_course_cd()));
    if (error > 4500 && _wind.length() < AP::gps().ground_speed()*0.8f) {
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

    if (_compass && _compass->is_calibrating()) {
        // don't do any yaw correction while calibrating
        return;
    }
    
    if (AP_AHRS_DCM::use_compass()) {
        /*
          we are using compass for yaw
         */
        if (_compass->last_update_usec() != _compass_last_update) {
            yaw_deltat = (_compass->last_update_usec() - _compass_last_update) * 1.0e-6f;
            _compass_last_update = _compass->last_update_usec();
            // we force an additional compass read()
            // here. This has the effect of throwing away
            // the first compass value, which can be bad
            if (!_flags.have_initial_yaw && _compass->read()) {
                const float heading = _compass->calculate_heading(_dcm_matrix);
                _dcm_matrix.from_euler(roll, pitch, heading);
                _omega_yaw_P.zero();
                _flags.have_initial_yaw = true;
            }
            new_value = true;
            yaw_error = yaw_error_compass();

            // also update the _gps_last_update, so if we later
            // disable the compass due to significant yaw error we
            // don't suddenly change yaw with a reset
            _gps_last_update = _gps.last_fix_time_ms();
        }
    } else if (_flags.fly_forward && have_gps()) {
        /*
          we are using GPS for yaw
         */
        if (_gps.last_fix_time_ms() != _gps_last_update &&
                _gps.ground_speed() >= GPS_SPEED_MIN) {
            yaw_deltat = (_gps.last_fix_time_ms() - _gps_last_update) * 1.0e-3f;
            _gps_last_update = _gps.last_fix_time_ms();
            new_value = true;
            const float gps_course_rad = ToRad(_gps.ground_course_cd() * 0.01f);
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
            if (!_flags.have_initial_yaw ||
                    yaw_deltat > 20 ||
                    (_gps.ground_speed() >= 3*GPS_SPEED_MIN && fabsf(yaw_error_rad) >= 1.047f)) {
                // reset DCM matrix based on current yaw
                _dcm_matrix.from_euler(roll, pitch, gps_course_rad);
                _omega_yaw_P.zero();
                _flags.have_initial_yaw = true;
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
        _kp_yaw = AP_AHRS_YAW_P_MIN;
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
    for (uint8_t i=0; i<_ins.get_accel_count(); i++) {
        if (_ins.get_accel_health(i)) {
            /*
              by using get_delta_velocity() instead of get_accel() the
              accel value is sampled over the right time delta for
              each sensor, which prevents an aliasing effect
             */
            Vector3f delta_velocity;
            _ins.get_delta_velocity(i, delta_velocity);
            const float delta_velocity_dt = _ins.get_delta_velocity_dt(i);
            if (delta_velocity_dt > 0) {
                _accel_ef[i] = _dcm_matrix * (delta_velocity / delta_velocity_dt);
                // integrate the accel vector in the earth frame between GPS readings
                _ra_sum[i] += _accel_ef[i] * deltat;
            }
        }
    }

    //update _accel_ef_blended
    if (_ins.get_accel_count() == 2 && _ins.use_accel(0) && _ins.use_accel(1)) {
        const float imu1_weight_target = _active_accel_instance == 0 ? 1.0f : 0.0f;
        // slew _imu1_weight over one second
        _imu1_weight += constrain_float(imu1_weight_target-_imu1_weight, -deltat, deltat);
        _accel_ef_blended = _accel_ef[0] * _imu1_weight + _accel_ef[1] * (1.0f - _imu1_weight);
    } else {
        _accel_ef_blended = _accel_ef[_ins.get_primary_accel()];
    }

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;

    const AP_GPS &_gps = AP::gps();

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
        float airspeed;
        if (airspeed_sensor_enabled()) {
            airspeed = _airspeed->get_airspeed();
        } else {
            airspeed = _last_airspeed;
        }
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
        const Matrix3f &rot = get_rotation_body_to_ned();
        airspeed = rot.mul_transpose(airspeed);

        // take positive component in X direction. This mimics a pitot
        // tube
        _last_airspeed = MAX(airspeed.x, 0);
    }

    if (have_gps()) {
        // use GPS for positioning with any fix, even a 2D fix
        _last_lat = _gps.location().lat;
        _last_lng = _gps.location().lng;
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

    if (_flags.correct_centrifugal && (_have_gps_lock || _flags.fly_forward)) {
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
    const float tilt = norm(GA_e.x, GA_e.y);

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
        _kp = AP_AHRS_RP_P_MIN;
    }

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P = error[besti] * _P_gain(spin_rate) * _kp;
    if (use_fast_gains()) {
        _omega_P *= 8;
    }

    if (_flags.fly_forward && _gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
            _gps.ground_speed() < GPS_SPEED_MIN &&
            _ins.get_accel().x >= 7 &&
            pitch_sensor > -3000 && pitch_sensor < 3000) {
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
        const float change_limit = _gyro_drift_limit * _omega_I_sum_time;
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
    if (!_flags.wind_estimation) {
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
    } else if (now - _last_wind_time > 2000 && airspeed_sensor_enabled()) {
        // when flying straight use airspeed to get wind estimate if available
        const Vector3f airspeed = _dcm_matrix.colx() * _airspeed->get_airspeed();
        const Vector3f wind = velocity - (airspeed * get_EAS2TAS());
        _wind = _wind * 0.92f + wind * 0.08f;
    }
}



// calculate the euler angles and DCM matrix which will be used for high level
// navigation control. Apply trim such that a positive trim value results in a
// positive vehicle rotation about that axis (ie a negative offset)
void
AP_AHRS_DCM::euler_angles(void)
{
    _body_dcm_matrix = _dcm_matrix * get_rotation_vehicle_body_to_autopilot_body();
    _body_dcm_matrix.to_euler(&roll, &pitch, &yaw);

    update_cd_values();
}

// return our current position estimate using
// dead-reckoning or GPS
bool AP_AHRS_DCM::get_position(struct Location &loc) const
{
    loc.lat = _last_lat;
    loc.lng = _last_lng;
    loc.alt = AP::baro().get_altitude() * 100 + _home.alt;
    loc.relative_alt = 0;
    loc.terrain_alt = 0;
    location_offset(loc, _position_offset_north, _position_offset_east);
    const AP_GPS &_gps = AP::gps();
    if (_flags.fly_forward && _have_position) {
        float gps_delay_sec = 0;
        _gps.get_lag(gps_delay_sec);
        location_update(loc, _gps.ground_course_cd() * 0.01f, _gps.ground_speed() * gps_delay_sec);
    }
    return _have_position;
}

// return an airspeed estimate if available
bool AP_AHRS_DCM::airspeed_estimate(float *airspeed_ret) const
{
    bool ret = false;
    if (airspeed_sensor_enabled()) {
        *airspeed_ret = _airspeed->get_airspeed();
        return true;
    }

    if (!_flags.wind_estimation) {
        return false;
    }

    // estimate it via GPS speed and wind
    if (have_gps()) {
        *airspeed_ret = _last_airspeed;
        ret = true;
    }

    if (ret && _wind_max > 0 && AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // constrain the airspeed by the ground speed
        // and AHRS_WIND_MAX
        const float gnd_speed = AP::gps().ground_speed();
        float true_airspeed = *airspeed_ret * get_EAS2TAS();
        true_airspeed = constrain_float(true_airspeed,
                                        gnd_speed - _wind_max,
                                        gnd_speed + _wind_max);
        *airspeed_ret = true_airspeed / get_EAS2TAS();
    }
    if (!ret) {
        // give the last estimate, but return false. This is used by
        // dead-reckoning code
        *airspeed_ret = _last_airspeed;
    }
    return ret;
}

bool AP_AHRS_DCM::set_home(const Location &loc)
{
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0 && loc.alt == 0) {
        return false;
    }
    if (!check_latlng(loc)) {
        return false;
    }

    _home = loc;
    _home_is_set = true;

    // log ahrs home and ekf origin dataflash
    Log_Write_Home_And_Origin();

    // send new home and ekf origin to GCS
    gcs().send_home();
    gcs().send_ekf_origin();

    return true;
}

//  a relative ground position to home in meters, Down
void AP_AHRS_DCM::get_relative_position_D_home(float &posD) const
{
    posD = -AP::baro().get_altitude();
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
  return amount of time that AHRS has been up
 */
uint32_t AP_AHRS_DCM::uptime_ms(void) const
{
    if (_last_startup_ms == 0) {
        return 0;
    }
    return AP_HAL::millis() - _last_startup_ms;
}
