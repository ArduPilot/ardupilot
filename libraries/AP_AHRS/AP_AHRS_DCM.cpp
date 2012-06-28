/*
	APM_AHRS_DCM.cpp

	AHRS system using DCM matrices

	Based on DCM code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com

	Adapted for the general ArduPilot AHRS interface by Andrew Tridgell

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public License
	as published by the Free Software Foundation; either version 2.1
	of the License, or (at your option) any later version.
*/
#include <FastSerial.h>
#include <AP_AHRS.h>

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

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] PROGMEM = {
	// @Param: YAW_P
	// @DisplayName: Yaw P
	// @Description: This controls the weight the compass has on the overall heading
	// @Range: 0 .4
	// @Increment: .01
    AP_GROUPINFO("YAW_P", 0, AP_AHRS_DCM, _kp_yaw),
    AP_GROUPEND
};

// run a full DCM update round
void
AP_AHRS_DCM::update(void)
{
	float delta_t;

	// tell the IMU to grab some data
	_imu->update();

	// ask the IMU how much time this sensor reading represents
	delta_t = _imu->get_delta_time();

	// Get current values for gyros
	_gyro_vector  = _imu->get_gyro();
	_accel_vector = _imu->get_accel();

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
	// Equation 16, adding proportional and integral correction terms
	_omega = _gyro_vector + _omega_I + _omega_P + _omega_yaw_P;

	_dcm_matrix.rotate(_omega * _G_Dt);
}


/*
  reset the DCM matrix and omega. Used on ground start, and on
  extreme errors in the matrix
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
  check the DCM matrix for pathological values
 */
void
AP_AHRS_DCM::check_matrix(void)
{
	if (_dcm_matrix.is_nan()) {
		//Serial.printf("ERROR: DCM matrix NAN\n");
		SITL_debug("ERROR: DCM matrix NAN\n");
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
			SITL_debug("ERROR: DCM matrix error. _dcm_matrix.c.x=%f\n",
				   _dcm_matrix.c.x);
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
	float	renorm_val;

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
			SITL_debug("ERROR: DCM renormalisation error. renorm_val=%f\n",
				   renorm_val);
			renorm_blowup_count++;
			return false;
		}
	}

	result = a * renorm_val;
	return true;
}

/*************************************************
Direction Cosine Matrix IMU: Theory
William Premerlani and Paul Bizard

Numerical errors will gradually reduce the orthogonality conditions expressed by equation 5
to approximations rather than identities. In effect, the axes in the two frames of reference no
longer describe a rigid body. Fortunately, numerical error accumulates very slowly, so it is a
simple matter to stay ahead of it.
We call the process of enforcing the orthogonality conditions ÒrenormalizationÓ.
*/
void
AP_AHRS_DCM::normalize(void)
{
	float error;
	Vector3f t0, t1, t2;

	error = _dcm_matrix.a * _dcm_matrix.b; 						// eq.18

	t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));		// eq.19
	t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));		// eq.19
	t2 = t0 % t1;					                // c= a x b // eq.20

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
			if (!_have_initial_yaw) {
				float heading = _compass->calculate_heading(_dcm_matrix);
				_dcm_matrix.from_euler(roll, pitch, heading);
				_omega_yaw_P.zero();
				_have_initial_yaw = true;
			}
			new_value = true;
			yaw_error = yaw_error_compass();
		}
	} else if (_gps && _gps->status() == GPS::GPS_OK) {
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
		_omega_yaw_P.z *= 0.97;
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
	_omega_yaw_P = error * _P_gain(spin_rate) * _kp_yaw.get();

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
    Vector3f error;
    Vector3f velocity;
    uint32_t last_correction_time;
    bool use_gps = true;

    // perform yaw drift correction if we have a new yaw reference
    // vector
    drift_correction_yaw();

    // integrate the accel vector in the earth frame between GPS readings
    _ra_sum += _dcm_matrix * (_accel_vector * deltat);

    // keep a sum of the deltat values, so we know how much time
    // we have integrated over
    _ra_deltat += deltat;

    // check if we have GPS lock
    if (_gps == NULL || _gps->status() != GPS::GPS_OK) {
	    use_gps = false;
    }

    // a copter (which has _fly_forward false) which doesn't have a
    // compass for yaw can't rely on the GPS velocity lining up with
    // the earth frame from DCM, so it needs to assume zero velocity
    // in the drift correction
    if (!_fly_forward && !(_compass && _compass->use_for_yaw())) {
	    use_gps = false;
    }

    if (use_gps == false) {
        // no GPS, or no lock. We assume zero velocity. This at
        // least means we can cope with gyro drift while sitting
        // on a bench with no GPS lock
        if (_ra_deltat < 0.1) {
            // not enough time has accumulated
            return;
        }
        velocity.zero();
	_last_velocity.zero();
        last_correction_time = millis();
    } else {
        if (_gps->last_fix_time == _ra_sum_start) {
            // we don't have a new GPS fix - nothing more to do
            return;
        }
        velocity = Vector3f(_gps->velocity_north(), _gps->velocity_east(), 0);
        last_correction_time = _gps->last_fix_time;
    }

#if 0
    /* 
       NOTE: The barometric vertical acceleration correction is disabled 
       until we work out how to filter it sufficiently to be usable
       on ArduCopter
    */
    if (_barometer != NULL) {
	    // Z velocity is down
	    velocity.z = - _barometer->get_climb_rate();
    }
#endif

    // see if this is our first time through - in which case we
    // just setup the start times and return
    if (_ra_sum_start == 0) {
        _ra_sum_start = last_correction_time;
        _last_velocity = velocity;
        return;
    }

    // get the corrected acceleration vector in earth frame. Units
    // are m/s/s
    Vector3f ge;
    float v_scale = 1.0/(_ra_deltat*_gravity);
    ge = Vector3f(0, 0, -1.0) + ((velocity - _last_velocity) * v_scale);

    // calculate the error term in earth frame.
    ge.normalize();
    _ra_sum.normalize();
    if (_ra_sum.is_inf() || ge.is_inf()) {
	    // the _ra_sum length is zero - we are falling with
	    // no apparent gravity. This gives us no information
	    // about which way up we are, so treat the error as zero
	    error = Vector3f(0,0,0);
    } else {
	    error = _ra_sum % ge;
    }

    _error_rp_sum += error.length();
    _error_rp_count++;

    // convert the error term to body frame
    error = _dcm_matrix.mul_transpose(error);

    // base the P gain on the spin rate
    float spin_rate = _omega.length();

    // we now want to calculate _omega_P and _omega_I. The
    // _omega_P value is what drags us quickly to the
    // accelerometer reading.
    _omega_P = error * _P_gain(spin_rate) * _kp;

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
}


// calculate the euler angles which will be used for high level
// navigation control
void
AP_AHRS_DCM::euler_angles(void)
{
	_dcm_matrix.to_euler(&roll, &pitch, &yaw);

	roll_sensor 	= degrees(roll)  * 100;
	pitch_sensor 	= degrees(pitch) * 100;
	yaw_sensor 	= degrees(yaw)   * 100;

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
