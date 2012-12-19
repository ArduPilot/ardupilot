/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       APM_AHRS_DCM.cpp
 *
 *       AHRS system using DCM matrices
 *
 *       Based on DCM code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Adapted for the general ArduPilot AHRS interface by Andrew Tridgell
 *
 *       This library is free software; you can redistribute it and/or
 *       modify it under the terms of the GNU Lesser General Public License
 *       as published by the Free Software Foundation; either version 2.1
 *       of the License, or (at your option) any later version.
 */
#include <AP_AHRS.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// this is the speed in cm/s above which we first get a yaw lock with
// the GPS
#define GPS_SPEED_MIN 300

// this is the speed in cm/s at which we stop using drift correction
// from the GPS and wait for the ground speed to get above GPS_SPEED_MIN
#define GPS_SPEED_RESET 100

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20


// run a full DCM update round
void
AP_AHRS_DCM::update(void)
{
    float delta_t;

    // tell the IMU to grab some data
    _ins->update();

    // ask the IMU how much time this sensor reading represents
    delta_t = _ins->get_delta_time();

    // if the update call took more than 0.2 seconds then discard it,
    // otherwise we may move too far. This happens when arming motors 
    // in ArduCopter
    if (delta_t > 0.2) {
        _ra_sum.zero();
        _ra_deltat = 0;
        return;
    }

    // Get current values for gyros
    _gyro_vector  = _ins->get_gyro();
    _accel_vector = _ins->get_accel();

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
    _omega = _gyro_vector + _omega_I;

    _dcm_matrix.rotate((_omega + _omega_P + _omega_yaw_P) * _G_Dt);
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
    if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
        _dcm_matrix.from_euler(roll, pitch, yaw);
    } else {
        // otherwise make it flat
        _dcm_matrix.from_euler(0, 0, 0);
    }
}

/*
 *  check the DCM matrix for pathological values
 */
void
AP_AHRS_DCM::check_matrix(void)
{
    if (_dcm_matrix.is_nan()) {
        //Serial.printf("ERROR: DCM matrix NAN\n");
        renorm_blowup_count++;
        reset(true);
        return;
    }
    // some DCM matrix values can lead to an out of range error in
    // the pitch calculation via asin().  These NaN values can
    // feed back into the rest of the DCM matrix via the
    // error_course value.
    if (!(_dcm_matrix.c.x < 1.0 &&
          _dcm_matrix.c.x > -1.0)) {
        // We have an invalid matrix. Force a normalisation.
        renorm_range_count++;
        normalize();

        if (_dcm_matrix.is_nan() ||
            fabs(_dcm_matrix.c.x) > 10) {
            // normalisation didn't fix the problem! We're
            // in real trouble. All we can do is reset
            //Serial.printf("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
            //	   _dcm_matrix.c.x);
            renorm_blowup_count++;
            reset(true);
        }
    }
}

// renormalise one vector component of the DCM matrix
// this will return false if renormalization fails
bool
AP_AHRS_DCM::renorm(Vector3f const &a, Vector3f &result)
{
    float renorm_val;

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

    renorm_val = 1.0 / a.length();

    // keep the average for reporting
    _renorm_val_sum += renorm_val;
    _renorm_val_count++;

    if (!(renorm_val < 2.0 && renorm_val > 0.5)) {
        // this is larger than it should get - log it as a warning
        renorm_range_count++;
        if (!(renorm_val < 1.0e6 && renorm_val > 1.0e-6)) {
            // we are getting values which are way out of
            // range, we will reset the matrix and hope we
            // can recover our attitude using drift
            // correction before we hit the ground!
            //Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
            //	   renorm_val);
            renorm_blowup_count++;
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
    float error;
    Vector3f t0, t1, t2;

    error = _dcm_matrix.a * _dcm_matrix.b;                                              // eq.18

    t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));              // eq.19
    t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));              // eq.19
    t2 = t0 % t1;                                                       // c= a x b // eq.20

    if (!renorm(t0, _dcm_matrix.a) ||
        !renorm(t1, _dcm_matrix.b) ||
        !renorm(t2, _dcm_matrix.c)) {
        // Our solution is blowing up and we will force back
        // to last euler angles
        reset(true);
    }
}


// produce a yaw error value. The returned value is proportional
// to sin() of the current heading error in earth frame
float
AP_AHRS_DCM::yaw_error_compass(void)
{
    Vector3f mag = Vector3f(_compass->mag_x, _compass->mag_y, _compass->mag_z);
    // get the mag vector in the earth frame
    Vector3f rb = _dcm_matrix * mag;

    rb.normalize();
    if (rb.is_inf()) {
        // not a valid vector
        return 0.0;
    }

    // get the earths magnetic field (only X and Y components needed)
    Vector3f mag_earth = Vector3f(cos(_compass->get_declination()),
                                  sin(_compass->get_declination()), 0);

    // calculate the error term in earth frame
    Vector3f error = rb % mag_earth;

    return error.z;
}

// produce a yaw error value using the GPS. The returned value is proportional
// to sin() of the current heading error in earth frame
float
AP_AHRS_DCM::yaw_error_gps(void)
{
    return sin(ToRad(_gps->ground_course * 0.01) - yaw);
}


// the _P_gain raises the gain of the PI controller
// when we are spinning fast. See the fastRotations
// paper from Bill.
float
AP_AHRS_DCM::_P_gain(float spin_rate)
{
    if (spin_rate < ToDeg(50)) {
        return 1.0;
    }
    if (spin_rate > ToDeg(500)) {
        return 10.0;
    }
    return spin_rate/ToDeg(50);
}

// return true if we have and should use GPS
bool AP_AHRS_DCM::have_gps(void)
{
    if (!_gps || _gps->status() != GPS::GPS_OK || !_gps_use) {
        return false;
    }
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

    if (_compass && _compass->use_for_yaw()) {
        if (_compass->last_update != _compass_last_update) {
            yaw_deltat = (_compass->last_update - _compass_last_update) * 1.0e-6;
            _compass_last_update = _compass->last_update;
            // we force an additional compass read()
            // here. This has the effect of throwing away
            // the first compass value, which can be bad
            if (!_have_initial_yaw && _compass->read()) {
                float heading = _compass->calculate_heading(_dcm_matrix);
                _dcm_matrix.from_euler(roll, pitch, heading);
                _omega_yaw_P.zero();
                _have_initial_yaw = true;
            }
            new_value = true;
            yaw_error = yaw_error_compass();
        }
    } else if (_fly_forward && have_gps()) {
        if (_gps->last_fix_time != _gps_last_update &&
            _gps->ground_speed >= GPS_SPEED_MIN) {
            yaw_deltat = (_gps->last_fix_time - _gps_last_update) * 1.0e-3;
            _gps_last_update = _gps->last_fix_time;
            if (!_have_initial_yaw) {
                _dcm_matrix.from_euler(roll, pitch, ToRad(_gps->ground_course*0.01));
                _omega_yaw_P.zero();
                _have_initial_yaw = true;
            }
            new_value = true;
            yaw_error = yaw_error_gps();
        }
    }

    if (!new_value) {
        // we don't have any new yaw information
        // slowly decay _omega_yaw_P to cope with loss
        // of our yaw source
        _omega_yaw_P *= 0.97;
        return;
    }

    // the yaw error is a vector in earth frame
    Vector3f error = Vector3f(0,0, yaw_error);

    // convert the error vector to body frame
    error = _dcm_matrix.mul_transpose(error);

    // the spin rate changes the P gain, and disables the
    // integration at higher rates
    float spin_rate = _omega.length();

    // update the proportional control to drag the
    // yaw back to the right value. We use a gain
    // that depends on the spin rate. See the fastRotations.pdf
    // paper from Bill Premerlani

    _omega_yaw_P.z = error.z * _P_gain(spin_rate) * _kp_yaw;
    if (_fast_ground_gains) {
        _omega_yaw_P.z *= 8;
    }

    // don't update the drift term if we lost the yaw reference
    // for more than 2 seconds
    if (yaw_deltat < 2.0 && spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        // also add to the I term
        _omega_I_sum.z += error.z * _ki_yaw * yaw_deltat;
    }

    _error_yaw_sum += fabs(yaw_error);
    _error_yaw_count++;
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
    Matrix3f temp_dcm = _dcm_matrix;
    Vector3f velocity;
    uint32_t last_correction_time;

    // perform yaw drift correction if we have a new yaw reference
    // vector
    drift_correction_yaw();

    // apply trim
    temp_dcm.rotate(_trim);

    // integrate the accel vector in the earth frame between GPS readings
    _ra_sum += temp_dcm * (_accel_vector * deltat);

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;

    if (!have_gps()) {
        // no GPS, or not a good lock. From experience we need at
        // least 6 satellites to get a really reliable velocity number
        // from the GPS.
        //
        // As a fallback we use the fixed wing acceleration correction
        // if we have an airspeed estimate (which we only have if
        // _fly_forward is set), otherwise no correction
        if (_ra_deltat < 0.2) {
            // not enough time has accumulated
            return;
        }
        float airspeed;
        if (_airspeed && _airspeed->use()) {
            airspeed = _airspeed->get_airspeed();
        } else {
            airspeed = _last_airspeed;
        }
        // use airspeed to estimate our ground velocity in
        // earth frame by subtracting the wind
        velocity = _dcm_matrix.colx() * airspeed;

        // add in wind estimate
        velocity += _wind;

        last_correction_time = hal.scheduler->millis();
        _have_gps_lock = false;

        // update position delta for get_position()
        _position_offset_north += velocity.x * _ra_deltat;
        _position_offset_east  += velocity.y * _ra_deltat;
    } else {
        if (_gps->last_fix_time == _ra_sum_start) {
            // we don't have a new GPS fix - nothing more to do
            return;
        }
        velocity = Vector3f(_gps->velocity_north(), _gps->velocity_east(), _gps->velocity_down());
        last_correction_time = _gps->last_fix_time;
        if (_have_gps_lock == false) {
            // if we didn't have GPS lock in the last drift
            // correction interval then set the velocities equal
            _last_velocity = velocity;
        }
        _have_gps_lock = true;

        // remember position for get_position()
        _last_lat = _gps->latitude;
        _last_lng = _gps->longitude;
        _position_offset_north = 0;
        _position_offset_east = 0;

        // once we have a single GPS lock, we update using
        // dead-reckoning from then on
        _have_position = true;

        // keep last airspeed estimate for dead-reckoning purposes
        Vector3f airspeed = velocity - _wind;
        airspeed.z = 0;
        _last_airspeed = airspeed.length();
    }

    /*
     *  The barometer for vertical velocity is only enabled if we got
     *  at least 5 pressure samples for the reading. This ensures we
     *  don't use very noisy climb rate data
     */
    if (_baro_use && _barometer != NULL && _barometer->get_pressure_samples() >= 5) {
        // Z velocity is down
        velocity.z = -_barometer->get_climb_rate();
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
    Vector3f GA_e;
    float v_scale = gps_gain.get()/(_ra_deltat*_gravity);
    Vector3f vdelta = (velocity - _last_velocity) * v_scale;
    // limit vertical acceleration correction to 0.5 gravities. The
    // barometer sometimes gives crazy acceleration changes. 
    vdelta.z = constrain(vdelta.z, -0.5, 0.5);
    GA_e = Vector3f(0, 0, -1.0) + vdelta;
    GA_e.normalize();
    if (GA_e.is_inf()) {
        // wait for some non-zero acceleration information
        return;
    }

    // calculate the error term in earth frame.
    Vector3f GA_b = _ra_sum / (_ra_deltat * _gravity);
    float length = GA_b.length();
    if (length > 1.0) {
        GA_b /= length;
        if (GA_b.is_inf()) {
            // wait for some non-zero acceleration information
            return;
        }
    }
    Vector3f error = GA_b % GA_e;

#define YAW_INDEPENDENT_DRIFT_CORRECTION 0
#if YAW_INDEPENDENT_DRIFT_CORRECTION
    // step 2 calculate earth_error_Z
    float earth_error_Z = error.z;

    // equation 10
    float tilt = pythagorous2(GA_e.x, GA_e.y);

    // equation 11
    float theta = atan2(GA_b.y, GA_b.x);

    // equation 12
    Vector3f GA_e2 = Vector3f(cos(theta)*tilt, sin(theta)*tilt, GA_e.z);

    // step 6
    error = GA_b % GA_e2;
    error.z = earth_error_Z;
#endif // YAW_INDEPENDENT_DRIFT_CORRECTION

    // to reduce the impact of two competing yaw controllers, we
    // reduce the impact of the gps/accelerometers on yaw when we are
    // flat, but still allow for yaw correction using the
    // accelerometers at high roll angles as long as we have a GPS
    if (_compass && _compass->use_for_yaw()) {
        if (have_gps() && gps_gain == 1.0) {
            error.z *= sin(fabs(roll));
        } else {
            error.z = 0;
        }
    }

    // convert the error term to body frame
    error = _dcm_matrix.mul_transpose(error);

    if (error.is_nan() || error.is_inf()) {
        // don't allow bad values
        check_matrix();
        return;
    }

    _error_rp_sum += error.length();
    _error_rp_count++;

    // base the P gain on the spin rate
    float spin_rate = _omega.length();

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P = error * _P_gain(spin_rate) * _kp;
    if (_fast_ground_gains) {
        _omega_P *= 8;
    }

    // accumulate some integrator error
    if (spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        _omega_I_sum += error * _ki * _ra_deltat;
        _omega_I_sum_time += _ra_deltat;
    }

    if (_omega_I_sum_time >= 5) {
        // limit the rate of change of omega_I to the hardware
        // reported maximum gyro drift rate. This ensures that
        // short term errors don't cause a buildup of omega_I
        // beyond the physical limits of the device
        float change_limit = _gyro_drift_limit * _omega_I_sum_time;
        _omega_I_sum.x = constrain(_omega_I_sum.x, -change_limit, change_limit);
        _omega_I_sum.y = constrain(_omega_I_sum.y, -change_limit, change_limit);
        _omega_I_sum.z = constrain(_omega_I_sum.z, -change_limit, change_limit);
        _omega_I += _omega_I_sum;
        _omega_I_sum.zero();
        _omega_I_sum_time = 0;
    }

    // zero our accumulator ready for the next GPS step
    _ra_sum.zero();
    _ra_deltat = 0;
    _ra_sum_start = last_correction_time;

    // remember the velocity for next time
    _last_velocity = velocity;

    if (_have_gps_lock && _fly_forward) {
        // update wind estimate
        estimate_wind(velocity);
    }
}


// update our wind speed estimate
void AP_AHRS_DCM::estimate_wind(Vector3f &velocity)
{
    // this is based on the wind speed estimation code from MatrixPilot by
    // Bill Premerlani. Adaption for ArduPilot by Jon Challinger
    // See http://gentlenav.googlecode.com/files/WindEstimation.pdf
    Vector3f fuselageDirection = _dcm_matrix.colx();
    Vector3f fuselageDirectionDiff = fuselageDirection - _last_fuse;
    uint32_t now = hal.scheduler->millis();

    // scrap our data and start over if we're taking too long to get a direction change
    if (now - _last_wind_time > 10000) {
        _last_wind_time = now;
        _last_fuse = fuselageDirection;
        _last_vel = velocity;
        return;
    }

    float diff_length = fuselageDirectionDiff.length();
    if (diff_length > 0.2) {
        // when turning, use the attitude response to estimate
        // wind speed
        float V;
        Vector3f velocityDiff = velocity - _last_vel;

        // estimate airspeed it using equation 6
        V = velocityDiff.length() / diff_length;

        _last_fuse = fuselageDirection;
        _last_vel = velocity;

        Vector3f fuselageDirectionSum = fuselageDirection + _last_fuse;
        Vector3f velocitySum = velocity + _last_vel;

        float theta = atan2(velocityDiff.y, velocityDiff.x) - atan2(fuselageDirectionDiff.y, fuselageDirectionDiff.x);
        float sintheta = sin(theta);
        float costheta = cos(theta);

        Vector3f wind = Vector3f();
        wind.x = velocitySum.x - V * (costheta * fuselageDirectionSum.x - sintheta * fuselageDirectionSum.y);
        wind.y = velocitySum.y - V * (sintheta * fuselageDirectionSum.x + costheta * fuselageDirectionSum.y);
        wind.z = velocitySum.z - V * fuselageDirectionSum.z;
        wind *= 0.5;

	if (wind.length() < _wind.length() + 20) {
		_wind = _wind * 0.95 + wind * 0.05;
	}

        _last_wind_time = now;
    } else if (now - _last_wind_time > 2000 && _airspeed && _airspeed->use()) {
        // when flying straight use airspeed to get wind estimate if available
        Vector3f airspeed = _dcm_matrix.colx() * _airspeed->get_airspeed();
        Vector3f wind = velocity - airspeed;
        _wind = _wind * 0.92 + wind * 0.08;
    }    
}



// calculate the euler angles which will be used for high level
// navigation control
void
AP_AHRS_DCM::euler_angles(void)
{
    _dcm_matrix.to_euler(&roll, &pitch, &yaw);

    roll_sensor     = degrees(roll)  * 100;
    pitch_sensor    = degrees(pitch) * 100;
    yaw_sensor      = degrees(yaw)   * 100;

    if (yaw_sensor < 0)
        yaw_sensor += 36000;
}

/* reporting of DCM state for MAVLink */

// average error_roll_pitch since last call
float AP_AHRS_DCM::get_error_rp(void)
{
    if (_error_rp_count == 0) {
        // this happens when telemetry is setup on two
        // serial ports
        return _error_rp_last;
    }
    _error_rp_last = _error_rp_sum / _error_rp_count;
    _error_rp_sum = 0;
    _error_rp_count = 0;
    return _error_rp_last;
}

// average error_yaw since last call
float AP_AHRS_DCM::get_error_yaw(void)
{
    if (_error_yaw_count == 0) {
        // this happens when telemetry is setup on two
        // serial ports
        return _error_yaw_last;
    }
    _error_yaw_last = _error_yaw_sum / _error_yaw_count;
    _error_yaw_sum = 0;
    _error_yaw_count = 0;
    return _error_yaw_last;
}

// return our current position estimate using
// dead-reckoning or GPS
bool AP_AHRS_DCM::get_position(struct Location *loc)
{
    if (!_have_position) {
        return false;
    }
    loc->lat = _last_lat;
    loc->lng = _last_lng;
    location_offset(loc, _position_offset_north, _position_offset_east);
    return true;
}

// return an airspeed estimate if available
bool AP_AHRS_DCM::airspeed_estimate(float *airspeed_ret)
{
	bool ret = false;
	if (_airspeed && _airspeed->use()) {
		*airspeed_ret = _airspeed->get_airspeed();
		ret = true;
	}

	// estimate it via GPS speed and wind
	if (have_gps()) {
		*airspeed_ret = _last_airspeed;
		ret = true;
	}

	if (ret && _wind_max > 0 && _gps && _gps->status() == GPS::GPS_OK) {
		// constrain the airspeed by the ground speed
		// and AHRS_WIND_MAX
		*airspeed_ret = constrain(*airspeed_ret, 
					  _gps->ground_speed*0.01 - _wind_max, 
					  _gps->ground_speed*0.01 + _wind_max);
	}
	return ret;
}
