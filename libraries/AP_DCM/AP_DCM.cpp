
#define RADX100 0.000174532925
#define DEGX100 5729.57795
/*
	APM_DCM_FW.cpp - DCM AHRS Library, fixed wing version, for Ardupilot Mega
		Code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com

	This library works with the ArduPilot Mega and "Oilpan"

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

        Methods:
				update_DCM()	: Updates the AHRS by integrating the rotation matrix over time using the IMU object data
				get_gyro()			: Returns gyro vector corrected for bias
				get_accel()		: Returns accelerometer vector
				get_dcm_matrix()	: Returns dcm matrix

*/
#include <AP_DCM.h>

#define OUTPUTMODE 1				// This is just used for debugging, remove later
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

//#define Kp_ROLLPITCH 0.05967 		// .0014 * 418/9.81 Pitch&Roll Drift Correction Proportional Gain
//#define Ki_ROLLPITCH 0.00001278		// 0.0000003 * 418/9.81 Pitch&Roll Drift Correction Integrator Gain
//#define Ki_ROLLPITCH 0.0			// 0.0000003 * 418/9.81 Pitch&Roll Drift Correction Integrator Gain

//#define Kp_YAW 0.8		 		// Yaw Drift Correction Porportional Gain
//#define Ki_YAW 0.00004 				// Yaw Drift CorrectionIntegrator Gain

// this is the speed in cm/s above which we first get a yaw lock with
// the GPS
#define GPS_SPEED_MIN 300

// this is the speed in cm/s at which we stop using drift correction
// from the GPS and wait for the ground speed to get above GPS_SPEED_MIN
#define GPS_SPEED_RESET 100

void
AP_DCM::set_compass(Compass *compass)
{
	_compass = compass;
}

/**************************************************/
void
AP_DCM::update_DCM_fast(void)
{
	float delta_t;

	_imu->update();
	_gyro_vector 	= _imu->get_gyro();			// Get current values for IMU sensors
	_accel_vector 	= _imu->get_accel();			// Get current values for IMU sensors

	delta_t = _imu->get_delta_time();

	matrix_update(delta_t); 	// Integrate the DCM matrix

	switch(_toggle++){
		case 0:
			normalize();				// Normalize the DCM matrix
		break;

		case 1:
			//drift_correction();			// Normalize the DCM matrix
			euler_rp();			// Calculate pitch, roll, yaw for stabilization and navigation
		break;

		case 2:
			drift_correction();			// Normalize the DCM matrix
		break;

		case 3:
			//drift_correction();			// Normalize the DCM matrix
			euler_rp();			// Calculate pitch, roll, yaw for stabilization and navigation
		break;

		case 4:
			euler_yaw();
		break;

		default:
			euler_rp();			// Calculate pitch, roll, yaw for stabilization and navigation
			_toggle = 0;
			//drift_correction();			// Normalize the DCM matrix
		break;
	}
}

/**************************************************/
void
AP_DCM::update_DCM(void)
{
	float delta_t;

	_imu->update();
	_gyro_vector 	= _imu->get_gyro();			// Get current values for IMU sensors
	_accel_vector 	= _imu->get_accel();			// Get current values for IMU sensors

	delta_t = _imu->get_delta_time();

	matrix_update(delta_t); 	// Integrate the DCM matrix
	normalize();			// Normalize the DCM matrix
	drift_correction();		// Perform drift correction
	euler_angles();			// Calculate pitch, roll, yaw for stabilization and navigation
}

/**************************************************/

    //For Debugging
/*
void
printm(const char *l, Matrix3f &m)
{ 	Serial.println(" "); Serial.println(l);
	Serial.print(m.a.x, 12); Serial.print(" "); Serial.print(m.a.y, 12); Serial.print(" "); Serial.println(m.a.z, 12);
	Serial.print(m.b.x, 12); Serial.print(" "); Serial.print(m.b.y, 12); Serial.print(" "); Serial.println(m.b.z, 12);
	Serial.print(m.c.x, 12); Serial.print(" "); Serial.print(m.c.y, 12); Serial.print(" "); Serial.println(m.c.z, 12);
	Serial.print(*(uint32_t *)&(m.a.x), HEX); Serial.print(" "); Serial.print(*(uint32_t *)&(m.a.y), HEX); Serial.print(" ");  Serial.println(*(uint32_t *)&(m.a.z), HEX);
	Serial.print(*(uint32_t *)&(m.b.x), HEX); Serial.print(" "); Serial.print(*(uint32_t *)&(m.b.y), HEX); Serial.print(" ");  Serial.println(*(uint32_t *)&(m.b.z), HEX);
	Serial.print(*(uint32_t *)&(m.c.x), HEX); Serial.print(" "); Serial.print(*(uint32_t *)&(m.c.y), HEX); Serial.print(" ");  Serial.println(*(uint32_t *)&(m.c.z), HEX);
}
*/

/**************************************************/
void
AP_DCM::matrix_update(float _G_Dt)
{
	Matrix3f	update_matrix;
	Matrix3f	temp_matrix;

	_omega_integ_corr 	= _gyro_vector 		+ _omega_I;		// Used for _centripetal correction (theoretically better than _omega)
	_omega 				= _omega_integ_corr + _omega_P;		// Equation 16, adding proportional and integral correction terms

	if(_centripetal &&
	   _gps != NULL &&
	   _gps->status() == GPS::GPS_OK) {
		// Remove _centripetal acceleration.
		accel_adjust();
	}

 #if OUTPUTMODE == 1
 	float tmp = _G_Dt * _omega.x;
	update_matrix.b.z = -tmp; 		// -delta Theta x
	update_matrix.c.y =  tmp; 		// delta Theta x

 	tmp = _G_Dt * _omega.y;
	update_matrix.c.x = -tmp; 		// -delta Theta y
	update_matrix.a.z =  tmp; 		// delta Theta y

 	tmp = _G_Dt * _omega.z;
	update_matrix.b.x =  tmp; 		// delta Theta z
	update_matrix.a.y = -tmp; 		// -delta Theta z

	update_matrix.a.x = 0;
	update_matrix.b.y = 0;
	update_matrix.c.z = 0;
 #else										// Uncorrected data (no drift correction)
	update_matrix.a.x = 0;
	update_matrix.a.y = -_G_Dt * _gyro_vector.z;
	update_matrix.a.z =  _G_Dt * _gyro_vector.y;
	update_matrix.b.x =  _G_Dt * _gyro_vector.z;
	update_matrix.b.y = 0;
	update_matrix.b.z = -_G_Dt * _gyro_vector.x;
	update_matrix.c.x = -_G_Dt * _gyro_vector.y;
	update_matrix.c.y =  _G_Dt * _gyro_vector.x;
	update_matrix.c.z = 0;
 #endif

	temp_matrix = _dcm_matrix * update_matrix;
	_dcm_matrix = _dcm_matrix + temp_matrix;		// Equation 17
}


/**************************************************/
void
AP_DCM::accel_adjust(void)
{
	Vector3f veloc, temp;

	veloc.x = _gps->ground_speed / 100;		// We are working with acceleration in m/s^2 units

	// We are working with a modified version of equation 26 as our IMU object reports acceleration in the positive axis direction as positive

	//_accel_vector -= _omega_integ_corr % _veloc;		// Equation 26  This line is giving the compiler a problem so we break it up below
	temp.x = 0;
	temp.y = _omega_integ_corr.z * veloc.x; 			// only computing the non-zero terms
	temp.z = -1.0f * _omega_integ_corr.y * veloc.x;	// After looking at the compiler issue lets remove _veloc and simlify

	_accel_vector -= temp;
}

/*
  reset the DCM matrix and omega. Used on ground start, and on
  extreme errors in the matrix
 */
void
AP_DCM::matrix_reset(bool recover_eulers)
{
	if (_compass != NULL) {
		_compass->null_offsets_disable();		
	}

	// reset the integration terms
	_omega_I.x = 0.0f;
	_omega_I.y = 0.0f;
	_omega_I.z = 0.0f;
	_omega_P = _omega_I;
	_omega_integ_corr = _omega_I;
	_omega = _omega_I;
	_error_roll_pitch = _omega_I;

	// if the caller wants us to try to recover to the current
	// attitude then calculate the dcm matrix from the current
	// roll/pitch/yaw values
	if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
		rotation_matrix_from_euler(_dcm_matrix, roll, pitch, yaw);
	} else {
		// otherwise make it flat
		rotation_matrix_from_euler(_dcm_matrix, 0, 0, 0);
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
AP_DCM::check_matrix(void)
{
	if (_dcm_matrix.is_nan()) {
		//Serial.printf("ERROR: DCM matrix NAN\n");
		SITL_debug("ERROR: DCM matrix NAN\n");
		renorm_blowup_count++;
		matrix_reset(true);
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
			matrix_reset(true);
		}
	}
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
AP_DCM::normalize(void)
{
	float error = 0;
	Vector3f	temporary[3];

	int problem = 0;

	error = _dcm_matrix.a * _dcm_matrix.b; 							// eq.18

	temporary[0] = _dcm_matrix.b;
	temporary[1] = _dcm_matrix.a;
	temporary[0] = _dcm_matrix.a - (temporary[0] * (0.5f * error));		// eq.19
	temporary[1] = _dcm_matrix.b - (temporary[1] * (0.5f * error));		// eq.19

	temporary[2] = temporary[0] % temporary[1];							// c= a x b // eq.20

	_dcm_matrix.a = renorm(temporary[0], problem);
	_dcm_matrix.b = renorm(temporary[1], problem);
	_dcm_matrix.c = renorm(temporary[2], problem);

	if (problem == 1) {		// Our solution is blowing up and we will force back to initial condition.	Hope we are not upside down!
		matrix_reset(true);
	}
}

/**************************************************/
Vector3f
AP_DCM::renorm(Vector3f const &a, int &problem)
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

	renorm_val = 1.0 / sqrt(a * a);

	if (!(renorm_val < 2.0 && renorm_val > 0.5)) {
		// this is larger than it should get - log it as a warning
		renorm_range_count++;
		if (!(renorm_val < 1.0e6 && renorm_val > 1.0e-6)) {
			// we are getting values which are way out of
			// range, we will reset the matrix and hope we
			// can recover our attitude using drift
			// correction before we hit the ground!
			problem = 1;
			//Serial.printf("ERROR: DCM renormalisation error. renorm_val=%f\n",
			//	   renorm_val);
			SITL_debug("ERROR: DCM renormalisation error. renorm_val=%f\n",
				   renorm_val);
			renorm_blowup_count++;
		}
	}

	return (a * renorm_val);
}

/**************************************************/
void
AP_DCM::drift_correction(void)
{
	//Compensation the Roll, Pitch and Yaw drift.
	//float mag_heading_x;
	//float mag_heading_y;
	float error_course = 0;
	float accel_magnitude;
	float accel_weight;
	float integrator_magnitude;
	Vector3f error_yaw;
	//static float scaled_omega_P[3];
	//static float scaled_omega_I[3];

	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	accel_magnitude = _accel_vector.length() / 9.80665f;

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	accel_weight = constrain(1 - _clamp * fabs(1 - accel_magnitude), 0, 1);	// upped to (<0.66G = 0.0, 1G = 1.0 , >1.33G = 0.0)

	//	We monitor the amount that the accelerometer based drift correction is deweighted for performance reporting
	_health = constrain(_health+(0.02 * (accel_weight - .5)), 0, 1);

	// adjust the ground of reference
	_error_roll_pitch =  _dcm_matrix.c % _accel_vector;			// Equation 27  *** sign changed from prev implementation???

	// error_roll_pitch are in Accel m/s^2 units
	// Limit max error_roll_pitch to limit max omega_P and omega_I
	_error_roll_pitch.x = constrain(_error_roll_pitch.x, -1.17f, 1.17f);
	_error_roll_pitch.y = constrain(_error_roll_pitch.y, -1.17f, 1.17f);
	_error_roll_pitch.z = constrain(_error_roll_pitch.z, -1.17f, 1.17f);

	_omega_P =  _error_roll_pitch * (_kp_roll_pitch * accel_weight);
	_omega_I += _error_roll_pitch * (_ki_roll_pitch * accel_weight);


	//*****YAW***************

	if (_compass && _compass->use_for_yaw()) {
		if (_have_initial_yaw) {
			// Equation 23, Calculating YAW error
			// We make the gyro YAW drift correction based
			// on compass magnetic heading
			error_course = (_dcm_matrix.a.x * _compass->heading_y) - (_dcm_matrix.b.x * _compass->heading_x);
		} else {
			// this is our first estimate of the yaw,
			// construct a DCM matrix based on the current
			// roll/pitch and the compass heading, but

			// first ensure the compass heading has been
			// calculated
			_compass->calculate(_dcm_matrix);

			// now construct a new DCM matrix
			_compass->null_offsets_disable();
			rotation_matrix_from_euler(_dcm_matrix, roll, pitch, _compass->heading);
			_compass->null_offsets_enable();
			_have_initial_yaw = true;
		}
	} else if (_gps && _gps->status() == GPS::GPS_OK) {

		// Use GPS Ground course to correct yaw gyro drift
		if (_gps->ground_speed >= GPS_SPEED_MIN) {
			if (_have_initial_yaw) {
				float course_over_ground_x = cos(ToRad(_gps->ground_course/100.0));
				float course_over_ground_y = sin(ToRad(_gps->ground_course/100.0));
				// Equation 23, Calculating YAW error
				error_course = (_dcm_matrix.a.x * course_over_ground_y) - (_dcm_matrix.b.x * course_over_ground_x);
			} else  {
				// when we first start moving, set the
				// DCM matrix to the current
				// roll/pitch values, but with yaw
				// from the GPS
				if (_compass) {
					_compass->null_offsets_disable();
				}
				rotation_matrix_from_euler(_dcm_matrix, roll, pitch, ToRad(_gps->ground_course));
				if (_compass) {
					_compass->null_offsets_enable();
				}
				_have_initial_yaw =  true;
				error_course = 0;
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

	error_yaw = _dcm_matrix.c * error_course;	// Equation 24, Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	_omega_P += error_yaw * _kp_yaw;			// Adding yaw correction to proportional correction vector.
	_omega_I += error_yaw * _ki_yaw;			// adding yaw correction to integrator correction vector.

	//	Here we will place a limit on the integrator so that the integrator cannot ever exceed ~30 degrees/second
	integrator_magnitude = _omega_I.length();
	if (integrator_magnitude > radians(30)) {
		_omega_I *= (radians(30) / integrator_magnitude);
	}
	//Serial.print("*");
}


/**************************************************/
void
AP_DCM::euler_angles(void)
{
	check_matrix();

	#if (OUTPUTMODE == 2)				 // Only accelerometer info (debugging purposes)
	roll 		= atan2(_accel_vector.y, -_accel_vector.z);		// atan2(acc_y, acc_z)
	pitch 		= safe_asin((_accel_vector.x) / (double)9.81); // asin(acc_x)
	yaw 			= 0;
	#else
	calculate_euler_angles(_dcm_matrix, &roll, &pitch, &yaw);
	#endif

	roll_sensor 	= degrees(roll)  * 100;
	pitch_sensor 	= degrees(pitch) * 100;
	yaw_sensor 	= degrees(yaw)   * 100;

	if (yaw_sensor < 0)
		yaw_sensor += 36000;
}

void
AP_DCM::euler_rp(void)
{
	check_matrix();
	calculate_euler_angles(_dcm_matrix, &roll, &pitch, NULL);
	roll_sensor 	= roll * DEGX100;	//degrees(roll)  * 100;
	pitch_sensor 	= pitch * DEGX100; //degrees(pitch) * 100;
}

void
AP_DCM::euler_yaw(void)
{
	calculate_euler_angles(_dcm_matrix, NULL, NULL, &yaw);
	yaw_sensor 		= yaw * DEGX100; //degrees(yaw)   * 100;

	if (yaw_sensor < 0)
		yaw_sensor += 36000;
}
