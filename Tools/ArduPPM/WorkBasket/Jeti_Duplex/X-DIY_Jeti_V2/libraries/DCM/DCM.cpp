#include "DCM.h"

// XXX HACKS
APM_ADC	adc;

// XXX END HACKS


#define GRAVITY 418 //this equivalent to 1G in the raw data coming from the accelerometer 
#define ADC_CONSTRAINT 900

#define Kp_ROLLPITCH 0.0014	//0.015 // Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.0000003 // 0.00001	 Pitch&Roll Integrator Gain
#define Kp_YAW 1.2		 // 1.2	Yaw Porportional Gain	
#define Ki_YAW 0.00005 // 0.00005	Yaw Integrator Gain

// Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_DCM::_sensors[6]       = {1,2,0,4,5,6};	// For ArduPilot Mega Sensor Shield Hardware
const int     AP_DCM::_sensor_signs[]	= {1,-1,-1,-1,1,1,-1,-1,-1};	//{-1,1,-1,1,-1,1,-1,-1,-1}		!!!! These are probably not right

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
const float   AP_DCM::_gyro_temp_curve[3][3] = {
	{1665,0,0},
	{1665,0,0},
	{1665,0,0}
};	// values may migrate to a Config file



// Constructors ////////////////////////////////////////////////////////////////
AP_DCM::AP_DCM(APM_Compass *withCompass) :
	_compass(withCompass),
	_dcm_matrix(1, 0, 0,
		    0, 1, 0,
		    0, 0, 1),
	_G_Dt(0.02),
	_course_over_ground_x(0),
	_course_over_ground_y(1)
{
}

void
AP_DCM::update_DCM(void)
{
	read_adc_raw();			// Get current values for IMU sensors
	matrix_update(); 		// Integrate the DCM matrix
	normalize();			// Normalize the DCM matrix
	drift_correction();		// Perform drift correction
	euler_angles();			// Calculate pitch, roll, yaw for stabilization and navigation
}


// Read the 6 ADC channels needed for the IMU
// ------------------------------------------
void
AP_DCM::read_adc_raw(void)
{
	int tc_temp = adc.Ch(_gyro_temp_ch);
	for (int i = 0; i < 6; i++) {
		_adc_in[i] = adc.Ch(_sensors[i]);
		if (i < 3) {	// XXX magic numbers!
			_adc_in[i] -= _gyro_temp_comp(i, tc_temp);		// Subtract temp compensated typical gyro bias
		} else {
			_adc_in[i] -= 2025;							// Subtract typical accel bias
		}
	}
}

// Returns the temperature compensated raw gyro value
//---------------------------------------------------
float
AP_DCM::_gyro_temp_comp(int i, int temp) const
{
	// We use a 2nd order curve of the form Gtc = A + B * Graw + C * (Graw)**2
	//------------------------------------------------------------------------
	return _gyro_temp_curve[i][0] + _gyro_temp_curve[i][1] * temp + _gyro_temp_curve[i][2] * temp * temp;	
}

// Returns an analog value with the offset removed
// -----------------
float
AP_DCM::read_adc(int select)
{
	float temp;
	if (_sensor_signs[select] < 0)
		temp = (_adc_offset[select] - _adc_in[select]);
	else
		temp = (_adc_in[select] - _adc_offset[select]);
		
	if (abs(temp) > ADC_CONSTRAINT) 
		adc_constraints++; 			// We keep track of the number of times we constrain the ADC output for performance reporting

/*
//	For checking the pitch/roll drift correction gain time constants
switch (select) {
	case 3:
		return 0;
		break;
	case 4:
		return 0;
		break;
	case 5:
		return 400;
		break; 
}
*/
		

//End of drift correction gain test code	
		
	return constrain(temp, -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
}

/**************************************************/
void 
AP_DCM::normalize(void)
{
	float error = 0;
	DCM_Vector	temporary[3];
	
	uint8_t problem = 0;
	
	error = -_dcm_matrix(0).dot_product(_dcm_matrix(1)) * 0.5; // eq.19

	temporary[0] = _dcm_matrix(1) * error + _dcm_matrix(0);		// eq.19
	temporary[1] = _dcm_matrix(0) * error + _dcm_matrix(1);		// eq.19

	temporary[2] = temporary[0] ^ temporary[1];	// c= a x b // eq.20

	_dcm_matrix(0) = _renorm(temporary[0], problem);
	_dcm_matrix(1) = _renorm(temporary[1], problem);
	_dcm_matrix(2) = _renorm(temporary[2], problem);
	
	if (problem == 1) {		// Our solution is blowing up and we will force back to initial condition.	Hope we are not upside down!
		_dcm_matrix(0, 0)= 1.0f;
		_dcm_matrix(0, 1)= 0.0f;
		_dcm_matrix(0, 2)= 0.0f;
		_dcm_matrix(1, 0)= 0.0f;
		_dcm_matrix(1, 1)= 1.0f;
		_dcm_matrix(1, 2)= 0.0f;
		_dcm_matrix(2, 0)= 0.0f;
		_dcm_matrix(2, 1)= 0.0f;
		_dcm_matrix(2, 2)= 1.0f;
	}
}

DCM_Vector
AP_DCM::_renorm(DCM_Vector const &a, uint8_t &problem)
{
	float	renorm;

	renorm = a.dot_product(a);
	
	if (renorm < 1.5625f && renorm > 0.64f) {	// Check if we are OK with Taylor expansion
		renorm = 0.5 * (3 - renorm);		// eq.21
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
AP_DCM::drift_correction(void)
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
	
	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	accel_magnitude = _accel_vector.magnitude() / GRAVITY; // Scale to gravity.

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	accel_weight = constrain(1 - 2 * abs(1 - accel_magnitude), 0, 1);	//	
	
	//	We monitor the amount that the accelerometer based drift correction is deweighted for performanc reporting
	imu_health = imu_health + 0.02 * (accel_weight-.5);
	imu_health = constrain(imu_health, 0, 1);
	
	// adjust the ground of reference 
	_error_roll_pitch = _accel_vector ^ _dcm_matrix(2);

	// error_roll_pitch are in Accel ADC units
	// Limit max error_roll_pitch to limit max omega_P and omega_I
	_error_roll_pitch(0) = constrain(_error_roll_pitch(0), -50, 50);
	_error_roll_pitch(1) = constrain(_error_roll_pitch(1), -50, 50);
	_error_roll_pitch(2) = constrain(_error_roll_pitch(2), -50, 50);

	_omega_P = _error_roll_pitch * (Kp_ROLLPITCH * accel_weight);
	_omega_I += _error_roll_pitch * (Ki_ROLLPITCH * accel_weight);
	
	//*****YAW***************
	
	if (_compass) {
		// We make the gyro YAW drift correction based on compass magnetic heading
		error_course= (_dcm_matrix(0, 0) * _compass->Heading_Y) - (_dcm_matrix(1, 0) * _compass->Heading_X);	// Calculating YAW error	
	} else {
		// Use GPS Ground course to correct yaw gyro drift
		if (ground_speed >= SPEEDFILT) {
			// Optimization: We have precalculated course_over_ground_x and course_over_ground_y (Course over Ground X and Y) from GPS info
			error_course = (_dcm_matrix(0, 0) * _course_over_ground_y) - (_dcm_matrix(1, 0) * _course_over_ground_x);	// Calculating YAW error
		}
	}			
	_error_yaw = _dcm_matrix(2) * error_course;	// Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
	
	_omega_P += _error_yaw * Kp_YAW;		// Adding	Proportional.
	_omega_I += _error_yaw * Ki_YAW;		// adding integrator to the omega_I	 

	//	Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
	integrator_magnitude = sqrt(_omega_I.dot_product(_omega_I));
	if (integrator_magnitude > radians(300)) {
		_omega_I *= (0.5f * radians(300) / integrator_magnitude);
	}
	
}

/**************************************************/
void 
AP_DCM::_accel_adjust(void)
{
	_accel_vector(1) += accel_scale((ground_speed / 100) * _omega(2));	// Centrifugal force on Acc_y = GPS_speed * GyroZ
	_accel_vector(2) -= accel_scale((ground_speed / 100) * _omega(1));	// Centrifugal force on Acc_z = GPS_speed * GyroY 
}


/**************************************************/
void 
AP_DCM::matrix_update(void)
{
	DCM_Matrix	update_matrix;

	_gyro_vector(0) = gyro_scaled_X(read_adc(0)); // gyro x roll
	_gyro_vector(1) = gyro_scaled_Y(read_adc(1)); // gyro y pitch
	_gyro_vector(2) = gyro_scaled_Z(read_adc(2)); // gyro Z yaw
	
	//Record when you saturate any of the gyros.
	if((abs(_gyro_vector(0)) >= radians(300)) || 
	   (abs(_gyro_vector(1)) >= radians(300)) || 
	   (abs(_gyro_vector(2)) >= radians(300)))
		gyro_sat_count++;
		
/*	
Serial.print (__adc_in[0]);
Serial.print ("	 ");
Serial.print (_adc_offset[0]);
Serial.print ("	 ");
Serial.print (_gyro_vector(0));
Serial.print ("	 ");
Serial.print (__adc_in[1]);
Serial.print ("	 ");
Serial.print (_adc_offset[1]);
Serial.print ("	 ");
Serial.print (_gyro_vector(1));
Serial.print ("	 ");
Serial.print (__adc_in[2]);
Serial.print ("	 ");
Serial.print (_adc_offset[2]);
Serial.print ("	 ");
Serial.println (_gyro_vector(2));
*/

//	_accel_vector(0) = read_adc(3); // acc x
//	_accel_vector(1) = read_adc(4); // acc y
//	_accel_vector(2) = read_adc(5); // acc z 
	// Low pass filter on accelerometer data (to filter vibrations)
	_accel_vector(0) = _accel_vector(0) * 0.6 + (float)read_adc(3) * 0.4; // acc x
	_accel_vector(1) = _accel_vector(1) * 0.6 + (float)read_adc(4) * 0.4; // acc y
	_accel_vector(2) = _accel_vector(2) * 0.6 + (float)read_adc(5) * 0.4; // acc z

	_omega = _gyro_vector + _omega_I;		// adding proportional term
	_omega_vector = _omega + _omega_P;		// adding Integrator term

	_accel_adjust();				// Remove centrifugal acceleration.
	
 #if OUTPUTMODE == 1				 
	update_matrix(0, 0) = 0;
	update_matrix(0, 1) = -_G_Dt * 	_omega_vector(2); // -z
	update_matrix(0, 2) = _G_Dt * 	_omega_vector(1); // y
	update_matrix(1, 0) = _G_Dt * 	_omega_vector(2); // z
	update_matrix(1, 1) = 0;
	update_matrix(1, 2) = -_G_Dt * 	_omega_vector(0); // -x
	update_matrix(2, 0) = -_G_Dt * 	_omega_vector(1); // -y
	update_matrix(2, 1) = _G_Dt * 	_omega_vector(0); // x
	update_matrix(2, 2) = 0;
 #else										// Uncorrected data (no drift correction)
	update_matrix(0, 0) = 0;
	update_matrix(0, 1) = -_G_Dt * 	_gyro_vector(2); // -z
	update_matrix(0, 2) = _G_Dt * 	_gyro_vector(1); // y
	update_matrix(1, 0) = _G_Dt * 	_gyro_vector(2); // z
	update_matrix(1, 1) = 0;
	update_matrix(1, 2) = -_G_Dt * 	_gyro_vector(0);
	update_matrix(2, 0) = -_G_Dt * 	_gyro_vector(1);
	update_matrix(2, 1) = _G_Dt * 	_gyro_vector(0);
	update_matrix(2, 2) = 0;
 #endif

	// update
	_dcm_matrix += _dcm_matrix * update_matrix;
	
/*
Serial.print (_G_Dt * 1000);
Serial.print ("	 ");
Serial.print (dcm_matrix(0, 0));
Serial.print ("	 ");
Serial.print (dcm_matrix(0, 1));
Serial.print ("	 ");
Serial.print (dcm_matrix(0, 2));
Serial.print ("	 ");
Serial.print (dcm_matrix(1, 0));
Serial.print ("	 ");
Serial.print (dcm_matrix(1, 1));
Serial.print ("	 ");
Serial.print (dcm_matrix(1, 2));
Serial.print ("	 ");
Serial.print (dcm_matrix(2, 0));
Serial.print ("	 ");
Serial.print (dcm_matrix(2, 1));
Serial.print ("	 ");
Serial.println (dcm_matrix(2, 2));
*/
}

/**************************************************/
void 
AP_DCM::euler_angles(void)
{
	#if (OUTPUTMODE == 2)				 // Only accelerometer info (debugging purposes)
	roll 		= atan2(_accel_vector(1), _accel_vector(2));		// atan2(acc_y, acc_z)
	roll_sensor 	= degrees(roll) * 100;
	pitch 		= -asin((_accel_vector(0)) / (double)GRAVITY); // asin(acc_x)
	pitch_sensor 	= degrees(pitch) * 100;
	yaw 			= 0;
	#else
	pitch 		= -asin(_dcm_matrix(2, 0));
	pitch_sensor 	= degrees(pitch) * 100;
	roll 		= atan2(_dcm_matrix(2, 1), _dcm_matrix(2, 2));
	roll_sensor 	= degrees(roll) * 100;
	yaw 		= atan2(_dcm_matrix(1, 0), _dcm_matrix(0, 0));
	yaw_sensor 	= degrees(yaw) * 100;
	#endif
 
 /*
	Serial.print ("Roll ");
	Serial.print (roll_sensor / 100);
	Serial.print (", Pitch	");
	Serial.print (pitch_sensor / 100);
	Serial.print (", Yaw	");
	Serial.println (yaw_sensor / 100);
 */
}

/**************************************************/
//Computes the dot product of two vectors
float 
DCM_Vector::dot_product(DCM_Vector const &vector2) const
{
	float op = 0;
	
	for(int c = 0; c < 3; c++)
		op += _v[c] * vector2(c);
	
	return op; 
}

// cross-product
DCM_Vector
DCM_Vector::operator^(DCM_Vector const &a) const
{
	DCM_Vector	result;

	result(0) = (_v[1] * a(2)) - (_v[2] * a(1));
	result(1) = (_v[2] * a(0)) - (_v[0] * a(2));
	result(2) = (_v[0] * a(1)) - (_v[1] * a(0));

	return(result);
}

// scale
DCM_Vector
DCM_Vector::operator*(float scale) const
{
	DCM_Vector	result;

	result(0) = _v[0] * scale;
	result(1) = _v[1] * scale;
	result(2) = _v[2] * scale;

	return(result);
}

// scale
void
DCM_Vector::operator*=(float scale)
{
	_v[0] *= scale;
	_v[1] *= scale;
	_v[2] *= scale;
}

// add
DCM_Vector
DCM_Vector::operator+(DCM_Vector const &a) const
{
	DCM_Vector	result;

	result(0) = _v[0] + a(0);
	result(1) = _v[1] + a(1);
	result(2) = _v[2] + a(2);

	return(result);
}

// add
void
DCM_Vector::operator+=(DCM_Vector const &a)
{
	_v[0] += a(0);
	_v[1] += a(1);
	_v[2] += a(2);
}

// magnitude
float
DCM_Vector::magnitude(void) const
{
	return(sqrt((_v[0] * _v[0]) + 
		    (_v[1] * _v[1]) + 
		    (_v[2] * _v[2])));
}

// 3x3 matrix multiply
DCM_Matrix
DCM_Matrix::operator*(DCM_Matrix const &a) const
{
	DCM_Matrix	result;

	for (int x = 0; x < 3; x++) {
		for (int y = 0; y < 3; y++) {
			result(x, y) = 
				_m[x](0) * a(0, y) +
				_m[x](1) * a(1, y) +
				_m[x](2) * a(2, y);
		}
	}
	return(result);
}

// 3x3 matrix add
void
DCM_Matrix::operator+=(DCM_Matrix const &a)
{
	for (int x = 0; x < 3; x++)
		for (int y = 0; y < 3; y++)
			_m[x](y) += a(x,y);
}

