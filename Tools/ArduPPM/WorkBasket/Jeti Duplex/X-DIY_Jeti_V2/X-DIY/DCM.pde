
// Read the 6 ADC channels needed for the IMU
// -----------------
void Read_adc_raw(void)
{
	int tc_temp = APM_ADC.Ch(GYRO_TEMP_CH);
	for (int i = 0; i < 6; i++) {
		AN[i] = APM_ADC.Ch(sensors[i]);
		if (i < 3) {
			AN[i] -= gyro_temp_comp(i, tc_temp);		// Subtract temp compensated typical gyro bias
		} else {
			AN[i] -= 2025;							// Subtract typical accel bias
		}
	}
}

// Returns the temperature compensated raw gyro value
//---------------------------------------------------
float gyro_temp_comp(int i, int temp)
{
	// We use a 2nd order curve of the form Gtc = A + B * Graw + C * (Graw)**2
	//------------------------------------------------------------------------
	return GTC[i][0] + GTC[i][1] * temp + GTC[i][2] * temp * temp;	
}

// Returns an analog value with the offset removed
// -----------------
float read_adc(int select)
{
	float temp;
	if (SENSOR_SIGN[select] < 0)
		temp = (AN_OFFSET[select]-AN[select]);
	else
		temp = (AN[select]-AN_OFFSET[select]);
		
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
void Normalize(void)
{
	float error = 0;
	float temporary[3][3];
	float renorm = 0;
	boolean problem = FALSE;
	
	error= -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; // eq.19

	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); // eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); // eq.19
	
	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]); //eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]); //eq.19
	
	Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b // eq.20
	
	renorm = Vector_Dot_Product(&temporary[0][0], &temporary[0][0]); 
	if (renorm < 1.5625f && renorm > 0.64f) {										// Check if we are OK with Taylor expansion
		renorm = .5 * (3 - renorm);													// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1. / sqrt(renorm);													
		renorm_sqrt_count++;
	} else {
		problem = TRUE;
		renorm_blowup_count++;
	}
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
	
	renorm = Vector_Dot_Product(&temporary[1][0], &temporary[1][0]); 
	if (renorm < 1.5625f && renorm > 0.64f) {										// Check if we are OK with Taylor expansion
		renorm = .5 * (3 - renorm);													// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1. / sqrt(renorm);	
		renorm_sqrt_count++;
	} else {
		problem = TRUE;
		renorm_blowup_count++;
	}
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
	
	renorm = Vector_Dot_Product(&temporary[2][0], &temporary[2][0]); 
	if (renorm < 1.5625f && renorm > 0.64f) {										// Check if we are OK with Taylor expansion
		renorm = .5 * (3 - renorm);													// eq.21
	} else if (renorm < 100.0f && renorm > 0.01f) {
		renorm = 1. / sqrt(renorm);	
		renorm_sqrt_count++;
	} else {
		problem = TRUE;	
		renorm_blowup_count++;
	}
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
	
	if (problem) {								// Our solution is blowing up and we will force back to initial condition.	Hope we are not upside down!
		DCM_Matrix[0][0] = 1.0f;
		DCM_Matrix[0][1] = 0.0f;
		DCM_Matrix[0][2] = 0.0f;
		DCM_Matrix[1][0] = 0.0f;
		DCM_Matrix[1][1] = 1.0f;
		DCM_Matrix[1][2] = 0.0f;
		DCM_Matrix[2][0] = 0.0f;
		DCM_Matrix[2][1] = 0.0f;
		DCM_Matrix[2][2] = 1.0f;
		problem = FALSE;	
	}
}

/**************************************************/
void Drift_correction(void)
{
	//Compensation the Roll, Pitch and Yaw drift. 
	float mag_heading_x;
	float mag_heading_y;
	float errorCourse = 0;
	static float Scaled_Omega_P[3];
	static float Scaled_Omega_I[3];
	float Accel_magnitude;
	float Accel_weight;
	float Integrator_magnitude;
	
	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
	Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.

	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1);	//	
	
	//	We monitor the amount that the accelerometer based drift correction is deweighted for performanc reporting
	imu_health = imu_health + 0.02 * (Accel_weight-.5);
		imu_health = constrain(imu_health, 0, 1);
	
	Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); // adjust the ground of reference 
	// errorRollPitch are in Accel ADC units
	// Limit max errorRollPitch to limit max Omega_P and Omega_I
	errorRollPitch[0] = constrain(errorRollPitch[0], -50, 50);
	errorRollPitch[1] = constrain(errorRollPitch[1], -50, 50);
	errorRollPitch[2] = constrain(errorRollPitch[2], -50, 50);

	Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);
	
	Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);		 
	
	//*****YAW***************
	
	#if MAGNETOMETER == ENABLED 
		// We make the gyro YAW drift correction based on compass magnetic heading
		errorCourse= (DCM_Matrix[0][0] * APM_Compass.Heading_Y) - (DCM_Matrix[1][0] * APM_Compass.Heading_X);	// Calculating YAW error	
	#else	// Use GPS Ground course to correct yaw gyro drift=
		if(GPS.ground_speed >= SPEEDFILT){
			// Optimization: We have precalculated COGX and COGY (Course over Ground X and Y) from GPS info
			errorCourse = (DCM_Matrix[0][0] * COGY) - (DCM_Matrix[1][0] * COGX);	// Calculating YAW error
		} 
	#endif
	
	Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse); // Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW);
	Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding	Proportional.

	Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW);
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I	 
	

	//	Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
	Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I, Omega_I));
	if (Integrator_magnitude > ToRad(300)) {
		Vector_Scale(Omega_I, Omega_I, 0.5f * ToRad(300) / Integrator_magnitude);
	}
}

/**************************************************/
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale((GPS.ground_speed / 100) * Omega[2]);	// Centrifugal force on Acc_y = GPS_speed * GyroZ
 Accel_Vector[2] -= Accel_Scale((GPS.ground_speed / 100) * Omega[1]);	// Centrifugal force on Acc_z = GPS_speed * GyroY 
}


/**************************************************/
void Matrix_update(void)
{
	Gyro_Vector[0] = Gyro_Scaled_X(read_adc(0)); // gyro x roll
	Gyro_Vector[1] = Gyro_Scaled_Y(read_adc(1)); // gyro y pitch
	Gyro_Vector[2] = Gyro_Scaled_Z(read_adc(2)); // gyro Z yaw
	
	//Record when you saturate any of the gyros.
	if((abs(Gyro_Vector[0]) >= ToRad(300)) || (abs(Gyro_Vector[1]) >= ToRad(300)) || (abs(Gyro_Vector[2]) >= ToRad(300)))
			gyro_sat_count++;
		
/*	
Serial.print (AN[0]);
Serial.print ("	 ");
Serial.print (AN_OFFSET[0]);
Serial.print ("	 ");
Serial.print (Gyro_Vector[0]);
Serial.print ("	 ");
Serial.print (AN[1]);
Serial.print ("	 ");
Serial.print (AN_OFFSET[1]);
Serial.print ("	 ");
Serial.print (Gyro_Vector[1]);
Serial.print ("	 ");
Serial.print (AN[2]);
Serial.print ("	 ");
Serial.print (AN_OFFSET[2]);
Serial.print ("	 ");
Serial.println (Gyro_Vector[2]);
*/

//	Accel_Vector[0]=read_adc(3); // acc x
//	Accel_Vector[1]=read_adc(4); // acc y
//	Accel_Vector[2]=read_adc(5); // acc z 
	// Low pass filter on accelerometer data (to filter vibrations)
	Accel_Vector[0] = Accel_Vector[0] * 0.6 + (float)read_adc(3) * 0.4; // acc x
	Accel_Vector[1] = Accel_Vector[1] * 0.6 + (float)read_adc(4) * 0.4; // acc y
	Accel_Vector[2] = Accel_Vector[2] * 0.6 + (float)read_adc(5) * 0.4; // acc z

	
	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);	// adding proportional term
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); // adding Integrator term

	Accel_adjust();		// Remove centrifugal acceleration.
	
 #if OUTPUTMODE == 1				 
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; // -z
	Update_Matrix[0][2] = G_Dt * Omega_Vector[1]; // y
	Update_Matrix[1][0] = G_Dt * Omega_Vector[2]; // z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; // -x
	Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; // -y
	Update_Matrix[2][1] = G_Dt * Omega_Vector[0]; // x
	Update_Matrix[2][2] = 0;
 #else										// Uncorrected data (no drift correction)
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; // -z
	Update_Matrix[0][2] = G_Dt * Gyro_Vector[1]; // y
	Update_Matrix[1][0] = G_Dt * Gyro_Vector[2]; // z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
	Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
	Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
	Update_Matrix[2][2] = 0;
 #endif

	Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); // a * b = c

	for(int x = 0; x < 3; x++){ // Matrix Addition (update)
		for(int y = 0; y < 3; y++){
			DCM_Matrix[x][y] += Temporary_Matrix[x][y];
		} 
	}
	
/*
Serial.print (G_Dt * 1000);
Serial.print ("	 ");
Serial.print (DCM_Matrix[0][0]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[0][1]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[0][2]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[1][0]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[1][1]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[1][2]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[2][0]);
Serial.print ("	 ");
Serial.print (DCM_Matrix[2][1]);
Serial.print ("	 ");
Serial.println (DCM_Matrix[2][2]);
*/
}

/**************************************************/
void Euler_angles(void)
{
	#if (OUTPUTMODE == 2)				 // Only accelerometer info (debugging purposes)
		roll 			= atan2(Accel_Vector[1], Accel_Vector[2]);		// atan2(acc_y, acc_z)
		pitch 			= -asin((Accel_Vector[0]) / (double)GRAVITY); // asin(acc_x)
		yaw 			= 0;
		roll_sensor 	= ToDeg(roll) * 100;
		pitch_sensor 	= ToDeg(pitch) * 100;
	#else
		pitch 			= -asin(DCM_Matrix[2][0]);
		roll 			= atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
		yaw 			= atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
		pitch_sensor 	= ToDeg(pitch) * 100;
		roll_sensor 	= ToDeg(roll) * 100;
		yaw_sensor 		= ToDeg(yaw) * 100;
		if (yaw_sensor < 0)	yaw_sensor += 36000;
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
float Vector_Dot_Product(float vector1[3], float vector2[3])
{
	float op = 0;
	
	for(int c = 0; c < 3; c++)
	{
	op += vector1[c] * vector2[c];
	}
	
	return op; 
}

/**************************************************/
//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3])
{
	vectorOut[0]= (v1[1] * v2[2]) - (v1[2] * v2[1]);
	vectorOut[1]= (v1[2] * v2[0]) - (v1[0] * v2[2]);
	vectorOut[2]= (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

/**************************************************/
//Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2)
{
	for(int c = 0; c < 3; c++)
	{
	 vectorOut[c] = vectorIn[c] * scale2; 
	}
}

/**************************************************/
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
	for(int c = 0; c < 3; c++)
	{
		 vectorOut[c] = vectorIn1[c]+vectorIn2[c];
	}
}

/********* MATRIX FUNCTIONS *****************************************/
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
	float op[3];
	for(int x = 0; x < 3; x++){
		for(int y = 0; y < 3; y++){
			for(int w = 0; w < 3; w++){
				op[w] = a[x][w] * b[w][y];
			} 
			mat[x][y] = 0;
			mat[x][y] = op[0] + op[1] + op[2];
			float test = mat[x][y];
		}
	}
}
