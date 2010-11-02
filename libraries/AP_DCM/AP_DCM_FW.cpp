/*
	APM_DCM_FW.cpp - DCM AHRS Library, fixed wing version, for Ardupilot Mega
		Code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com

	This library works with the ArduPilot Mega and "Oilpan"
	
	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

        Methods:
                quick_init()		: For air restart
                init() 				: For ground start.  Calibrates the IMU
				update_DCM(_G_Dt)	: Updates the AHRS by integrating the rotation matrix over time _G_Dt using the IMU object data
				get_roll_sensor()	: Returns roll in degrees * 100
				get_roll()			: Returns roll in radians
				get_pitch_sensor()	: Returns pitch in degrees * 100
				get_pitch()			: Returns pitch in radians
				get_yaw_sensor()	: Returns yaw in degrees * 100
				get_yaw()			: Returns yaw in radians

*/
#include <AP_DCM_FW.h>

#define OUTPUTMODE 1	// This is just used for debugging, remove later
#define TRUE 1
#define FALSE 0


#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define Kp_ROLLPITCH 0.05967 		// .0014 * 418/9.81 Pitch&Roll Drift Correction Proportional Gain
#define Ki_ROLLPITCH 0.00001278		// 0.0000003 * 418/9.81 Pitch&Roll Drift Correction Integrator Gain
#define Kp_YAW 0.8		 			// Yaw Drift Correction Porportional Gain	
#define Ki_YAW 0.00004 				// Yaw Drift CorrectionIntegrator Gain


#define SPEEDFILT 300			// centimeters/second
#define ADC_CONSTRAINT 900


// Constructors ////////////////////////////////////////////////////////////////

AP_DCM_FW::AP_DCM_FW(GPS *GPS) :
	_gps(GPS),
	_compass(0),
	_dcm_matrix(1, 0, 0,
				0, 1, 0,
				0, 0, 1),
	_course_over_ground_x(0),
	_course_over_ground_y(1)
{
	AP_IMU _imu();
}

AP_DCM_FW::AP_DCM_FW(GPS *GPS, APM_Compass_Class *withCompass) :
	_gps(GPS),
	_compass(withCompass),
	_dcm_matrix(1, 0, 0,
				0, 1, 0,
				0, 0, 1),
	_course_over_ground_x(0),
	_course_over_ground_y(1)
{
	AP_IMU _imu();
}

/**************************************************/
void
AP_DCM_FW::update_DCM(float _G_Dt)
{
	_gyro_vector = _imu.get_gyro();			// Get current values for IMU sensors
	_accel_vector = _imu.get_accel();			// Get current values for IMU sensors
	matrix_update(_G_Dt); 	// Integrate the DCM matrix
	normalize();			// Normalize the DCM matrix
	drift_correction();		// Perform drift correction
	euler_angles();			// Calculate pitch, roll, yaw for stabilization and navigation
}


/**************************************************/
void
AP_DCM_FW::quick_init(void)
{
	_imu.quick_init();
}
/**************************************************/
void
AP_DCM_FW::init(void)
{
	_imu.init();
}


/**************************************************/
long
AP_DCM_FW::get_roll_sensor(void)
{	return degrees(roll) * 100;}

/**************************************************/
long
AP_DCM_FW::get_pitch_sensor(void)
{	return degrees(pitch) * 100;}

/**************************************************/
long
AP_DCM_FW::get_yaw_sensor(void)
{	
	long yaw_sensor = degrees(yaw) * 100;
	if (yaw_sensor < 0)	yaw_sensor += 36000;
	return yaw_sensor;
}

/**************************************************/
float
AP_DCM_FW::get_roll(void)
{	return roll;}

/**************************************************/
float
AP_DCM_FW::get_pitch(void)
{	return pitch;}

/**************************************************/
float
AP_DCM_FW::get_yaw(void)
{	return yaw;}

/**************************************************/
Vector3f
AP_DCM_FW::get_gyros(void)
{	return _gyro_vector;}

/**************************************************/
Vector3f
AP_DCM_FW::get_accels(void)
{	return _accel_vector;}

/**************************************************/
void 
AP_DCM_FW::matrix_update(float _G_Dt)
{
	Matrix3f	_update_matrix;
	static int8_t timer;
	
	//Record when you saturate any of the gyros.
	if((abs(_gyro_vector.x) >= radians(300)) || 
	   (abs(_gyro_vector.y) >= radians(300)) || 
	   (abs(_gyro_vector.z) >= radians(300)))
		gyro_sat_count++;

	_omega_integ_corr = _gyro_vector + _omega_I;		// Used for centrep correction (theoretically better than _omega)
	_omega = _omega_integ_corr + _omega_P;				// Equation 16, adding proportional and integral correction terms
	
	_accel_adjust();				// Remove centrifugal acceleration.
	
	
 #if OUTPUTMODE == 1				 
	_update_matrix.a.x = 0;
	_update_matrix.a.y = -_G_Dt * _omega.z; 		// -delta Theta z
	_update_matrix.a.z =  _G_Dt * _omega.y; 		// delta Theta y
	_update_matrix.b.x =  _G_Dt * _omega.z; 		// delta Theta z
	_update_matrix.b.y = 0;
	_update_matrix.b.z = -_G_Dt * _omega.x; 		// -delta Theta x
	_update_matrix.c.x = -_G_Dt * _omega.y; 		// -delta Theta y
	_update_matrix.c.y =  _G_Dt * _omega.x; 		// delta Theta x
	_update_matrix.c.z = 0;
 #else										// Uncorrected data (no drift correction)				 
	_update_matrix.a.x = 0;
	_update_matrix.a.y = -_G_Dt * _gyro_vector.z; 		
	_update_matrix.a.z =  _G_Dt * _gyro_vector.y;
	_update_matrix.b.x =  _G_Dt * _gyro_vector.z; 		
	_update_matrix.b.y = 0;
	_update_matrix.b.z = -_G_Dt * _gyro_vector.x; 		
	_update_matrix.c.x = -_G_Dt * _gyro_vector.y; 		
	_update_matrix.c.y =  _G_Dt * _gyro_vector.x; 		
	_update_matrix.c.z = 0;
 #endif
	
Serial.println("update matrix before");
Serial.println(_update_matrix.a.x);
Serial.println(_update_matrix.a.y);
Serial.println(_update_matrix.a.z);
Serial.println(_update_matrix.b.x);
Serial.println(_update_matrix.b.y);
Serial.println(_update_matrix.b.z);
Serial.println(_update_matrix.c.x);
Serial.println(_update_matrix.c.y);
Serial.println(_update_matrix.c.z);
Serial.println("dcm matrix before");
Serial.println(_dcm_matrix.a.x);
Serial.println(_dcm_matrix.a.y);
Serial.println(_dcm_matrix.a.z);
Serial.println(_dcm_matrix.b.x);
Serial.println(_dcm_matrix.b.y);
Serial.println(_dcm_matrix.b.z);
Serial.println(_dcm_matrix.c.x);
Serial.println(_dcm_matrix.c.y);
Serial.println(_dcm_matrix.c.z);
	// update
	_update_matrix = _dcm_matrix * _update_matrix;		// Equation 17
	
Serial.println("update matrix middle");
Serial.println(_update_matrix.a.x,12);
Serial.println(_update_matrix.a.y,12);
Serial.println(_update_matrix.a.z,12);
Serial.println(_update_matrix.b.x,12);
Serial.println(_update_matrix.b.y,12);
Serial.println(_update_matrix.b.z,12);
Serial.println(_update_matrix.c.x,12);
Serial.println(_update_matrix.c.y,12);
Serial.println(_update_matrix.c.z,12);
Serial.println("dcm matrix middle");
Serial.println(_dcm_matrix.a.x,12);
Serial.println(_dcm_matrix.a.y,12);
Serial.println(_dcm_matrix.a.z,12);
Serial.println(_dcm_matrix.b.x,12);
Serial.println(_dcm_matrix.b.y,12);
Serial.println(_dcm_matrix.b.z,12);
Serial.println(_dcm_matrix.c.x,12);
Serial.println(_dcm_matrix.c.y,12);
Serial.println(_dcm_matrix.c.z,12);

	_dcm_matrix = _dcm_matrix + _update_matrix;		// Equation 17
		
Serial.println("update matrix after");
Serial.println(_update_matrix.a.x);
Serial.println(_update_matrix.a.y);
Serial.println(_update_matrix.a.z);
Serial.println(_update_matrix.b.x);
Serial.println(_update_matrix.b.y);
Serial.println(_update_matrix.b.z);
Serial.println(_update_matrix.c.x);
Serial.println(_update_matrix.c.y);
Serial.println(_update_matrix.c.z);
Serial.println("dcm matrix after");
Serial.println(_dcm_matrix.a.x);
Serial.println(_dcm_matrix.a.y);
Serial.println(_dcm_matrix.a.z);
Serial.println(_dcm_matrix.b.x);
Serial.println(_dcm_matrix.b.y);
Serial.println(_dcm_matrix.b.z);
Serial.println(_dcm_matrix.c.x);
Serial.println(_dcm_matrix.c.y);
Serial.println(_dcm_matrix.c.z);
}


/**************************************************/
void 
AP_DCM_FW::_accel_adjust(void)
{
	Vector3f _veloc, _temp;
	float _vel;
	
	_veloc.x = _gps->ground_speed / 100;		// We are working with acceleration in m/s^2 units
	
	// We are working with a modified version of equation 26 as our IMU object reports acceleration in the positive axis direction as positive
	//_accel_vector -= _omega_integ_corr % _veloc;		// Equation 26  This line is giving the compiler a problem so we break it up below
	_temp.y = _omega_integ_corr.z * _veloc.x; 			// only computing the non-zero terms
	_temp.z = -1.0f * _omega_integ_corr.y * _veloc.x;	// After looking at the compiler issue lets remove _veloc and simlify 

	_accel_vector -= _temp;

}


/**************************************************/
void 
AP_DCM_FW::normalize(void)
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

	_dcm_matrix.a = _renorm(temporary[0], problem);
	_dcm_matrix.b = _renorm(temporary[1], problem);
	_dcm_matrix.c = _renorm(temporary[2], problem);
	
	if (problem == 1) {		// Our solution is blowing up and we will force back to initial condition.	Hope we are not upside down!
		_dcm_matrix.a.x = 1.0f;
		_dcm_matrix.a.y = 0.0f;
		_dcm_matrix.a.z = 0.0f;
		_dcm_matrix.b.x = 0.0f;
		_dcm_matrix.b.y = 1.0f;
		_dcm_matrix.b.z = 0.0f;
		_dcm_matrix.c.x = 0.0f;
		_dcm_matrix.c.y = 0.0f;
		_dcm_matrix.c.z = 1.0f;
Serial.println("Solution blew up");	
/*
Serial.println(_dcm_matrix.a.x);
Serial.println(_dcm_matrix.a.y);
Serial.println(_dcm_matrix.a.z);
Serial.println(_dcm_matrix.b.x);
Serial.println(_dcm_matrix.b.y);
Serial.println(_dcm_matrix.b.z);
Serial.println(_dcm_matrix.c.x);
Serial.println(_dcm_matrix.c.y);
Serial.println(_dcm_matrix.c.z);
*/
	}
}

/**************************************************/
Vector3f
AP_DCM_FW::_renorm(Vector3f const &a, int &problem)
{
	float	renorm;

	renorm = a * a;
	
	if (renorm < 1.5625f && renorm > 0.64f) {			// Check if we are OK to use Taylor expansion
		renorm = 0.5 * (3 - renorm);					// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1.0 / sqrt(renorm);
		renorm_sqrt_count++;
	} else {
		problem = 1;
		renorm_blowup_count++;
	}

	return(a * renorm);
}

/**************************************************/
void 
AP_DCM_FW::drift_correction(void)
{
	//Compensation the Roll, Pitch and Yaw drift. 
	float mag_heading_x;
	float mag_heading_y;
	float error_course = 0;
	static float scaled_omega_P[3];
	static float scaled_omega_I[3];
	float accel_magnitude;
	float accel_weight;
	float integrator_magnitude;
	static bool in_motion = FALSE;
	Matrix3f _rot_mat;
	
	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	accel_magnitude = _accel_vector.length() / 9.80665f;

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	accel_weight = constrain(1 - 2 * abs(1 - accel_magnitude), 0, 1);	//	
	
	//	We monitor the amount that the accelerometer based drift correction is deweighted for performance reporting
	imu_health = imu_health + 0.02 * (accel_weight-.5);
	imu_health = constrain(imu_health, 0, 1);
	
	// adjust the ground of reference 
	_error_roll_pitch =  _dcm_matrix.c % _accel_vector;			// Equation 27  *** sign changed from prev implementation???

	// error_roll_pitch are in Accel m/s^2 units
	// Limit max error_roll_pitch to limit max omega_P and omega_I
	_error_roll_pitch.x = constrain(_error_roll_pitch.x, -1.17f, 1.17f);
	_error_roll_pitch.y = constrain(_error_roll_pitch.y, -1.17f, 1.17f);
	_error_roll_pitch.z = constrain(_error_roll_pitch.z, -1.17f, 1.17f);

	_omega_P = _error_roll_pitch * (Kp_ROLLPITCH * accel_weight);
	_omega_I += _error_roll_pitch * (Ki_ROLLPITCH * accel_weight);

	
	//*****YAW***************
	
	if (_compass) {
		// We make the gyro YAW drift correction based on compass magnetic heading
		error_course= (_dcm_matrix.a.x * _compass->Heading_Y) - (_dcm_matrix.b.x * _compass->Heading_X);	// Equation 23, Calculating YAW error	
	} else {
		// Use GPS Ground course to correct yaw gyro drift
	//	if (_gps->ground_speed >= SPEEDFILT) {
			//_course_over_ground_x = cos(ToRad(_gps->ground_course/100.0));
			//_course_over_ground_y = sin(ToRad(_gps->ground_course/100.0));
Serial.println(_dcm_matrix.a.x);
Serial.println(_dcm_matrix.a.y);
Serial.println(_dcm_matrix.a.z);
Serial.println(_dcm_matrix.b.x);
Serial.println(_dcm_matrix.b.y);
Serial.println(_dcm_matrix.b.z);
Serial.println(_dcm_matrix.c.x);
Serial.println(_dcm_matrix.c.y);
Serial.println(_dcm_matrix.c.z);
			_course_over_ground_x = 1;
			_course_over_ground_y = 0;
Serial.print("in motion = ");
Serial.print(in_motion);
Serial.print("\t");
			if(in_motion) {
				error_course = (_dcm_matrix.a.x * _course_over_ground_y) - (_dcm_matrix.b.x * _course_over_ground_x);	// Equation 23, Calculating YAW error
			} else  {
Serial.println("in yaw reset");
				float cos_psi_err, sin_psi_err;
				
				yaw = atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);
				cos_psi_err = cos(yaw - ToRad(_gps->ground_course/100.0));
				sin_psi_err = sin(yaw - ToRad(_gps->ground_course/100.0));
				// Rxx = cos psi err, Rxy = - sin psi err, Rxz = 0
				// Ryx = sin psi err, Ryy = cos psi err,   Ryz = 0
				// Rzx = Rzy = Rzz = 0
				_rot_mat.a.x = cos_psi_err;
				_rot_mat.a.y = - sin_psi_err;
				_rot_mat.b.x = sin_psi_err;
				_rot_mat.b.y = cos_psi_err;
				_dcm_matrix = _rot_mat * _dcm_matrix;
				in_motion =  TRUE;
				error_course = 0;
				
			}	
	/*	} else {
			error_course = 0;
			in_motion = FALSE;
		}	*/
	}	
	
	_error_yaw = _dcm_matrix.c * error_course;	// Equation 24, Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
	
	_omega_P += _error_yaw * Kp_YAW;			// Adding yaw correction to proportional correction vector.
	_omega_I += _error_yaw * Ki_YAW;			// adding yaw correction to integrator correction vector.	 

	//	Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
	integrator_magnitude = _omega_I.length();
	if (integrator_magnitude > radians(300)) {
		_omega_I *= (0.5f * radians(300) / integrator_magnitude);		// Why do we have this discontinuous?  EG, why the 0.5?
	}

}


/**************************************************/
void 
AP_DCM_FW::euler_angles(void)
{
	#if (OUTPUTMODE == 2)				 // Only accelerometer info (debugging purposes)
	roll 		= atan2(_accel_vector.y, _accel_vector.z);		// atan2(acc_y, acc_z)
	pitch 		= -asin((_accel_vector.x) / (double)9.81); // asin(acc_x)
	yaw 			= 0;
	#else
	pitch 		= -asin(_dcm_matrix.c.x);
	roll 		= atan2(_dcm_matrix.c.y, _dcm_matrix.c.z);
	yaw 		= atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);
	#endif

}

/**************************************************/


