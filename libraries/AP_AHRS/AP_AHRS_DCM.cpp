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
	// _omega_integ_corr is used for _centripetal correction
	// (theoretically better than _omega)
	_omega_integ_corr = _gyro_vector + _omega_I;

	// Equation 16, adding proportional and integral correction terms
	_omega = _omega_integ_corr + _omega_P + _omega_yaw_P;

	// this is a replacement of the DCM matrix multiply (equation
	// 17), with known zero elements removed and the matrix
	// operations inlined. This runs much faster than the original
	// version of this code, as the compiler was doing a terrible
	// job of realising that so many of the factors were in common
	// or zero. It also uses much less stack, as we no longer need
	// two additional local matrices

	Vector3f r = _omega * _G_Dt;
	_dcm_matrix.rotate(r);
}


// adjust an accelerometer vector for known acceleration forces
void
AP_AHRS_DCM::accel_adjust(Vector3f &accel)
{
	float veloc;
	// compensate for linear acceleration. This makes a
	// surprisingly large difference in the pitch estimate when
	// turning, plus on takeoff and landing
	float acceleration = _gps->acceleration();
	accel.x -= acceleration;

	// compensate for centripetal acceleration
	veloc = _gps->ground_speed * 0.01;

	// We are working with a modified version of equation 26 as
	// our IMU object reports acceleration in the positive axis
	// direction as positive

	// Equation 26 broken up into separate pieces
	accel.y -= _omega_integ_corr.z * veloc;
	accel.z += _omega_integ_corr.y * veloc;
}

/*
  reset the DCM matrix and omega. Used on ground start, and on
  extreme errors in the matrix
 */
void
AP_AHRS_DCM::reset(bool recover_eulers)
{
	if (_compass != NULL) {
		_compass->null_offsets_disable();
	}

	// reset the integration terms
	_omega_I.zero();
	_omega_P.zero();
	_omega_yaw_P.zero();
	_omega_integ_corr.zero();
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

	if (_compass != NULL) {
		_compass->null_offsets_enable();	// This call is needed to restart the nulling
		// Otherwise the reset in the DCM matrix can mess up
		// the nulling
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


// perform drift correction. This function aims to update _omega_P and
// _omega_I with our best estimate of the short term and long term
// gyro error. The _omega_P value is what pulls our attitude solution
// back towards the reference vector quickly. The _omega_I term is an
// attempt to learn the long term drift rate of the gyros.
//
// This function also updates _omega_yaw_P with a yaw correction term
// from our yaw reference vector
void
AP_AHRS_DCM::drift_correction(float deltat)
{
	float error_course = 0;
	Vector3f accel;
	Vector3f error;
	float error_norm = 0;
	float yaw_deltat = 0;

	accel = _accel_vector;

	// if enabled, use the GPS to correct our accelerometer vector
	// for centripetal forces
	if(_centripetal &&
	   _gps != NULL &&
	   _gps->status() == GPS::GPS_OK) {
		accel_adjust(accel);
	}


	//*****Roll and Pitch***************

	// normalise the accelerometer vector to a standard length
	// this is important to reduce the impact of noise on the
	// drift correction, as very noisy vectors tend to have
	// abnormally high lengths. By normalising the length we
	// reduce their impact.
	float accel_length = accel.length();
	accel *= (_gravity / accel_length);
	if (accel.is_inf()) {
		// we can't do anything useful with this sample
		_omega_P.zero();
		return;
	}

	// calculate the error, in m/2^2, between the attitude
	// implied by the accelerometers and the attitude
	// in the current DCM matrix
	error =  _dcm_matrix.c % accel;

	// Limit max error to limit the effect of noisy values
	// on the algorithm. This limits the error to about 11
	// degrees
	error_norm = error.length();
	if (error_norm > 2) {
		error *= (2 / error_norm);
	}

	// we now want to calculate _omega_P and _omega_I. The
	// _omega_P value is what drags us quickly to the
	// accelerometer reading.
	_omega_P = error * _kp_roll_pitch;

	// the _omega_I is the long term accumulated gyro
	// error. This determines how much gyro drift we can
	// handle.
	_omega_I_sum += error * (_ki_roll_pitch * deltat);
	_omega_I_sum_time += deltat;

	// these sums support the reporting of the DCM state via MAVLink
	_error_rp_sum += error_norm;
	_error_rp_count++;

	// yaw drift correction

	// we only do yaw drift correction when we get a new yaw
	// reference vector. In between times we rely on the gyros for
	// yaw. Avoiding this calculation on every call to
	// update_DCM() saves a lot of time
	if (_compass && _compass->use_for_yaw()) {
		if (_compass->last_update != _compass_last_update) {
			yaw_deltat = 1.0e-6*(_compass->last_update - _compass_last_update);
			if (_have_initial_yaw && yaw_deltat < 2.0) {
				// Equation 23, Calculating YAW error
				// We make the gyro YAW drift correction based
				// on compass magnetic heading
				error_course = (_dcm_matrix.a.x * _compass->heading_y) - (_dcm_matrix.b.x * _compass->heading_x);
				_compass_last_update = _compass->last_update;
			} else {
				// this is our first estimate of the yaw,
				// or the compass has come back online after
				// no readings for 2 seconds.
				//
				// construct a DCM matrix based on the current
				// roll/pitch and the compass heading.
				// First ensure the compass heading has been
				// calculated
				_compass->calculate(_dcm_matrix);

				// now construct a new DCM matrix
				_compass->null_offsets_disable();
				_dcm_matrix.from_euler(roll, pitch, _compass->heading);
				_compass->null_offsets_enable();
				_have_initial_yaw = true;
				_compass_last_update = _compass->last_update;
				error_course = 0;
			}
		}
	} else if (_gps && _gps->status() == GPS::GPS_OK) {
		if (_gps->last_fix_time != _gps_last_update) {
			// Use GPS Ground course to correct yaw gyro drift
			if (_gps->ground_speed >= GPS_SPEED_MIN) {
				yaw_deltat = 1.0e-3*(_gps->last_fix_time - _gps_last_update);
				if (_have_initial_yaw && yaw_deltat < 2.0) {
					float course_over_ground_x = cos(ToRad(_gps->ground_course/100.0));
					float course_over_ground_y = sin(ToRad(_gps->ground_course/100.0));
					// Equation 23, Calculating YAW error
					error_course = (_dcm_matrix.a.x * course_over_ground_y) - (_dcm_matrix.b.x * course_over_ground_x);
					_gps_last_update = _gps->last_fix_time;
				} else  {
					// when we first start moving, set the
					// DCM matrix to the current
					// roll/pitch values, but with yaw
					// from the GPS
					if (_compass) {
						_compass->null_offsets_disable();
					}
					_dcm_matrix.from_euler(roll, pitch, ToRad(_gps->ground_course));
					if (_compass) {
						_compass->null_offsets_enable();
					}
					_have_initial_yaw =  true;
					error_course = 0;
					_gps_last_update = _gps->last_fix_time;
				}
			} else if (_gps->ground_speed >= GPS_SPEED_RESET) {
				// we are not going fast enough to use GPS for
				// course correction, but we won't reset
				// _have_initial_yaw yet, instead we just let
				// the gyro handle yaw
				error_course = 0;
			} else {
				// we are moving very slowly. Reset
				// _have_initial_yaw and adjust our heading
				// rapidly next time we get a good GPS ground
				// speed
				error_course = 0;
				_have_initial_yaw = false;
			}
		}
	}

	// see if there is any error in our heading relative to the
	// yaw reference. This will be zero most of the time, as we
	// only calculate it when we get new data from the yaw
	// reference source
	if (yaw_deltat == 0 || error_course == 0) {
		// we don't have a new reference heading. Slowly
		// decay the _omega_yaw_P to ensure that if we have
		// lost the yaw reference sensor completely we don't
		// keep using a stale offset
		_omega_yaw_P *= 0.97;
		goto check_sum_time;
	}

	// ensure the course error is scaled from -PI to PI
	if (error_course > PI) {
		error_course -= 2*PI;
	} else if (error_course < -PI) {
		error_course += 2*PI;
	}

	// Equation 24, Applys the yaw correction to the XYZ rotation of the aircraft
	// this gives us an error in radians
	error = _dcm_matrix.c * error_course;

	// Adding yaw correction to proportional correction vector. We
	// allow the yaw reference source to affect all 3 components
	// of _omega_yaw_P as we need to be able to correctly hold a
	// heading when roll and pitch are non-zero
	_omega_yaw_P = error * _kp_yaw.get();

	// add yaw correction to integrator correction vector, but
	// only for the z gyro. We rely on the accelerometers for x
	// and y gyro drift correction. Using the compass or GPS for
	// x/y drift correction is too inaccurate, and can lead to
	// incorrect builups in the x/y drift. We rely on the
	// accelerometers to get the x/y components right
	_omega_I_sum.z += error.z * (_ki_yaw * yaw_deltat);

	// we keep the sum of yaw error for reporting via MAVLink.
	_error_yaw_sum += error_course;
	_error_yaw_count++;

check_sum_time:
	if (_omega_I_sum_time > 10) {
		// every 10 seconds we apply the accumulated
		// _omega_I_sum changes to _omega_I. We do this to
		// prevent short term feedback between the P and I
		// terms of the controller. The _omega_I vector is
		// supposed to refect long term gyro drift, but with
		// high noise it can start to build up due to short
		// term interactions. By summing it over 10 seconds we
		// prevent the short term interactions and can apply
		// the slope limit more accurately
		float drift_limit = _gyro_drift_limit * _omega_I_sum_time;
		_omega_I_sum.x = constrain(_omega_I_sum.x, -drift_limit, drift_limit);
		_omega_I_sum.y = constrain(_omega_I_sum.y, -drift_limit, drift_limit);
		_omega_I_sum.z = constrain(_omega_I_sum.z, -drift_limit, drift_limit);

		_omega_I += _omega_I_sum;

		// zero the sum
		_omega_I_sum.zero();
		_omega_I_sum_time = 0;
	}
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
