
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

#define SPEEDFILT 300				// centimeters/second
#define ADC_CONSTRAINT 900


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

	if(_centripetal){
		accel_adjust();				// Remove _centripetal acceleration.
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

	if (_gps) {
		veloc.x = _gps->ground_speed / 100;		// We are working with acceleration in m/s^2 units
	}

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
AP_DCM::matrix_reset(void)
{
	_dcm_matrix.a.x = 1.0f;
	_dcm_matrix.a.y = 0.0f;
	_dcm_matrix.a.z = 0.0f;
	_dcm_matrix.b.x = 0.0f;
	_dcm_matrix.b.y = 1.0f;
	_dcm_matrix.b.z = 0.0f;
	_dcm_matrix.c.x = 0.0f;
	_dcm_matrix.c.y = 0.0f;
	_dcm_matrix.c.z = 1.0f;
	_omega_I.x = 0.0f;
	_omega_I.y = 0.0f;
	_omega_I.z = 0.0f;
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
		matrix_reset();
	}
}

/**************************************************/
Vector3f
AP_DCM::renorm(Vector3f const &a, int &problem)
{
	float	renorm_val;

	renorm_val = a * a;

	if (renorm_val < 1.5625f && renorm_val > 0.64f) {			// Check if we are OK to use Taylor expansion
		renorm_val = 0.5 * (3 - renorm_val);					// eq.21
	} else if (renorm_val < 100.0f && renorm_val > 0.01f) {
		renorm_val = 1.0 / sqrt(renorm_val);
		renorm_sqrt_count++;
	} else {
		problem = 1;
		renorm_blowup_count++;
	}

	return(a * renorm_val);
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
	//static float scaled_omega_P[3];
	//static float scaled_omega_I[3];
	static bool in_motion = false;
	Matrix3f rot_mat;

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

	if (_compass) {
		// We make the gyro YAW drift correction based on compass magnetic heading
		error_course = (_dcm_matrix.a.x * _compass->heading_y) - (_dcm_matrix.b.x * _compass->heading_x);	// Equation 23, Calculating YAW error

	} else if (_gps) {

		// Use GPS Ground course to correct yaw gyro drift
		if (_gps->ground_speed >= SPEEDFILT) {

			_course_over_ground_x = cos(ToRad(_gps->ground_course/100.0));
			_course_over_ground_y = sin(ToRad(_gps->ground_course/100.0));
			if(in_motion) {
				error_course = (_dcm_matrix.a.x * _course_over_ground_y) - (_dcm_matrix.b.x * _course_over_ground_x);	// Equation 23, Calculating YAW error
			} else  {
				float cos_psi_err, sin_psi_err;
				// This is the case for when we first start moving and reset the DCM so that yaw matches the gps ground course
				// This is just to get a reasonable estimate faster
				yaw = atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);
				cos_psi_err = cos(ToRad(_gps->ground_course/100.0) - yaw);
				sin_psi_err = sin(ToRad(_gps->ground_course/100.0) - yaw);
				// Rxx = cos psi err, Rxy = - sin psi err, Rxz = 0
				// Ryx = sin psi err, Ryy = cos psi err,   Ryz = 0
				// Rzx = Rzy = 0, Rzz = 1
				rot_mat.a.x = cos_psi_err;
				rot_mat.a.y = -sin_psi_err;
				rot_mat.b.x = sin_psi_err;
				rot_mat.b.y = cos_psi_err;
				rot_mat.a.z = 0;
				rot_mat.b.z = 0;
				rot_mat.c.x = 0;
				rot_mat.c.y = 0;
				rot_mat.c.z = 1.0;

				_dcm_matrix = rot_mat * _dcm_matrix;
				in_motion =  true;
				error_course = 0;
			}

		} else {
			error_course = 0;
			in_motion = false;
		}
	}

	_error_yaw = _dcm_matrix.c * error_course;	// Equation 24, Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	_omega_P += _error_yaw * _kp_yaw;			// Adding yaw correction to proportional correction vector.
	_omega_I += _error_yaw * _ki_yaw;			// adding yaw correction to integrator correction vector.

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
	#if (OUTPUTMODE == 2)				 // Only accelerometer info (debugging purposes)
	roll 		= atan2(_accel_vector.y, -_accel_vector.z);		// atan2(acc_y, acc_z)
	pitch 		= asin((_accel_vector.x) / (double)9.81); // asin(acc_x)
	yaw 			= 0;
	#else
	pitch 		= -asin(_dcm_matrix.c.x);
	roll 		= atan2(_dcm_matrix.c.y, _dcm_matrix.c.z);
	yaw 		= atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);
	#endif

	roll_sensor 	= degrees(roll)  * 100;
	pitch_sensor 	= degrees(pitch) * 100;
	yaw_sensor 		= degrees(yaw)   * 100;

	if (yaw_sensor < 0)
		yaw_sensor += 36000;
}

void
AP_DCM::euler_rp(void)
{
	pitch 			= -asin(_dcm_matrix.c.x);
	roll 			= atan2(_dcm_matrix.c.y, _dcm_matrix.c.z);
	roll_sensor 	= roll * DEGX100;	//degrees(roll)  * 100;
	pitch_sensor 	= pitch * DEGX100; //degrees(pitch) * 100;
}

void
AP_DCM::euler_yaw(void)
{
	yaw 			= atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);
	yaw_sensor 		= yaw * DEGX100; //degrees(yaw)   * 100;

	if (yaw_sensor < 0)
		yaw_sensor += 36000;
}
