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
	// _omega_integ_corr is used for centripetal correction
	// (theoretically better than _omega)
	_omega_integ_corr = _gyro_vector + _omega_I;

	// Equation 16, adding proportional and integral correction terms
	_omega = _omega_integ_corr + _omega_P;

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



// yaw drift correction using the compass
void
AP_AHRS_DCM::drift_correction_compass(float deltat)
{
	if (_compass == NULL ||
	    _compass->last_update == _compass_last_update) {
		// slowly degrade the yaw error term so we cope
		// gracefully with the compass going offline
		_drift_error_earth.z *= 0.97;
		return;
	}

	Vector3f mag = Vector3f(_compass->mag_x, _compass->mag_y, _compass->mag_z);

	if (!_have_initial_yaw) {
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
		_field_strength = mag.length();
		_compass_last_update = _compass->last_update;
		return;
	}

	float yaw_deltat = 1.0e-6*(_compass->last_update - _compass_last_update);

	_compass_last_update = _compass->last_update;

	// keep a estimate of the magnetic field strength
	_field_strength = (_field_strength * 0.95) + (mag.length() * 0.05);

	// get the mag vector in the earth frame
	Vector3f rb = _dcm_matrix * mag;

	// normalise rb so that it can be directly combined with
	// rotations given by the accelerometers, which are in 1g units
	rb *= yaw_deltat / _field_strength;

	if (rb.is_inf()) {
		// not a valid vector
		return;
	}

	// get the earths magnetic field (only X and Y components needed)
	Vector3f mag_earth = Vector3f(cos(_compass->get_declination()),
				      sin(_compass->get_declination()), 0);

	// calculate the error term in earth frame
	Vector3f error = rb % mag_earth;

	// setup the z component of the total drift error in earth
	// frame. This is then used by the main drift correction code
	_drift_error_earth.z = constrain(error.z, -0.4, 0.4);

	_error_yaw_sum += fabs(_drift_error_earth.z);
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

	// if we don't have a working GPS then use the old style
	// of drift correction
	if (_gps == NULL || _gps->status() != GPS::GPS_OK) {
		//drift_correction_old(deltat);
		return;
	}

	// perform yaw drift correction if we have a new yaw reference
	// vector
	drift_correction_compass(deltat);

	// scale the accel vector so it is in 1g units. This brings it
	// into line with the mag vector, allowing the two to be combined
	_accel_vector *= (deltat / _gravity);

	// integrate the accel vector in the earth frame between GPS readings
	_ra_sum += _dcm_matrix * _accel_vector;

	// keep a sum of the deltat values, so we know how much time
	// we have integrated over
	_ra_deltat += deltat;

	// see if we have a new GPS reading
	if (_gps->last_fix_time == _ra_sum_start) {
		// we don't have a new GPS fix - nothing more to do
		return;
	}

	// get GPS velocity vector in earth frame
	Vector3f gps_velocity = Vector3f(_gps->velocity_north(), _gps->velocity_east(), 0);

	// see if this is our first time through - in which case we
	// just setup the start times and return
	if (_ra_sum_start == 0) {
		_ra_sum_start = _gps->last_fix_time;
		_gps_last_velocity = gps_velocity;
		return;
	}

	// get the corrected acceleration vector in earth frame. Units
	// are 1g
	Vector3f ge = Vector3f(0,0, -_ra_deltat) + ((gps_velocity - _gps_last_velocity)/_gravity);

	// calculate the error term in earth frame
	error = _ra_sum % ge;

	// extract the X and Y components for the total drift
	// error. The Z component comes from the yaw source
	// we constrain the error on each axis to 0.2
	// the Z component of this error comes from the yaw correction
	_drift_error_earth.x = constrain(error.x, -0.2, 0.2);
	_drift_error_earth.y = constrain(error.y, -0.2, 0.2);

	// convert the error term to body frame
	error = _dcm_matrix.mul_transpose(_drift_error_earth);

	// we now want to calculate _omega_P and _omega_I. The
	// _omega_P value is what drags us quickly to the
	// accelerometer reading.
	_omega_P = error * _kp;

	// the _omega_I is the long term accumulated gyro
	// error. This determines how much gyro drift we can
	// handle.
	Vector3f omega_I_delta = error * (_ki * _ra_deltat);

	// add in the limited omega correction into the long term
	// drift correction accumulator
	_omega_I_sum += omega_I_delta;
	_omega_I_sum_time += _ra_deltat;

	// if we have accumulated a gyro drift estimate for 15
	// seconds, then move it to the _omega_I term which is applied
	// on each update
	if (_omega_I_sum_time > 15) {
		// limit the slope of omega_I on each axis to
		// the maximum drift rate reported by the sensor driver
		float drift_limit = _gyro_drift_limit * _omega_I_sum_time;
		_omega_I_sum.x = constrain(_omega_I_sum.x, -drift_limit, drift_limit);
		_omega_I_sum.y = constrain(_omega_I_sum.y, -drift_limit, drift_limit);
		_omega_I_sum.z = constrain(_omega_I_sum.z, -drift_limit, drift_limit);
		_omega_I += _omega_I_sum;
		_omega_I_sum.zero();
		_omega_I_sum_time = 0;
	}


	// zero our accumulator ready for the next GPS step
	_ra_sum.zero();
	_ra_deltat = 0;
	_ra_sum_start = _gps->last_fix_time;

	// remember the GPS velocity for next time
	_gps_last_velocity = gps_velocity;

	// these sums support the reporting of the DCM state via MAVLink
	_error_rp_sum += Vector3f(_drift_error_earth.x, _drift_error_earth.y, 0).length();
	_error_rp_count++;
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
