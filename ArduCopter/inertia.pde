#if INERTIAL_NAV == ENABLED

// generates a new location and velocity in space based on inertia
// Calc 100 hz
void calc_inertia()
{
	// rotate accels based on DCM
	// --------------------------
	accels_rotated		= ahrs.get_dcm_matrix() * imu.get_accel();
	accels_rotated		+= accels_offset;						// skew accels to account for long term error using calibration
	accels_rotated.z 	+= 9.805;								// remove influence of gravity

	// rising 		= 2
	// neutral 		= 0
	// falling 		= -2


	// ACC Y POS = going EAST
	// ACC X POS = going North
	// ACC Z POS = going DOWN (lets flip this)

	// Integrate accels to get the velocity
	// ------------------------------------
	Vector3f temp = accels_rotated * (G_Dt * 100);
	temp.z = -temp.z;
	// Temp is changed to world frame and we can use it normaly

	// Integrate accels to get the velocity
	// ------------------------------------
	accels_velocity			+= temp;
}

void z_error_correction()
{
	speed_error.z 		= climb_rate - accels_velocity.z;
	accels_velocity.z	+= speed_error.z * 0.0350;							//speed_correction_z;
	accels_velocity.z   -= g.pid_throttle.get_integrator() * 0.0045; 		//g.alt_offset_correction; // OK
	accels_offset.z		-= g.pid_throttle.get_integrator() * 0.000003;		//g.alt_i_correction ; 	// .000002;
}

void xy_error_correction()
{
	// Calculate speed error
	// ---------------------
	speed_error.x 		= x_actual_speed - accels_velocity.x;
	speed_error.y 		= y_actual_speed - accels_velocity.y;

	// correct integrated velocity by speed_error
	// this number must be small or we will bring back sensor latency
	// -------------------------------------------
	accels_velocity.x	+= speed_error.x * 0.0175;								// g.speed_correction_x;
	accels_velocity.y	+= speed_error.y * 0.0175;

	// Error correct the accels to deal with calibration, drift and noise
	// ------------------------------------------------------------------
	accels_velocity.x	-= g.pid_loiter_rate_lon.get_integrator() * 0.007; 		// g.loiter_offset_correction; //.001;
	accels_velocity.y	-= g.pid_loiter_rate_lat.get_integrator() * 0.007; 		// g.loiter_offset_correction; //.001;

	// update our accel offsets
	// -------------------------
	accels_offset.x		-= g.pid_loiter_rate_lon.get_integrator() * 0.000003; 	// g.loiter_i_correction;
	accels_offset.y		-= g.pid_loiter_rate_lat.get_integrator() * 0.000003; 	// g.loiter_i_correction;


	// For developement only
	// ---------------------
	if(motors.armed())
		Log_Write_Raw();
}

static void calibrate_accels()
{
	// sets accels_velocity to 0,0,0
	zero_accels();

	accels_offset.x = 0;
	accels_offset.y = 0;
	accels_offset.z = 0;

	for (int i = 0; i < 200; i++){
		delay(10);
		read_AHRS();
	}

	for (int i = 0; i < 100; i++){
		delay(10);
		read_AHRS();
		calc_inertia();
		Serial.printf("call accels: %1.5f, %1.5f, %1.5f,\n", accels_rotated.x, accels_rotated.y, accels_rotated.z);
	}

	accels_velocity /= 100;
	accels_offset = accels_velocity;
	zero_accels();
	calc_inertia();

	Log_Write_Data(25, (float)accels_offset.x);
	Log_Write_Data(26, (float)accels_offset.y);
	Log_Write_Data(27, (float)accels_offset.z);
}

void zero_accels()
{
	accels_rotated.x 	= 0;
	accels_rotated.y 	= 0;
	accels_rotated.z 	= 0;

	accels_velocity.x 	= 0;
	accels_velocity.y 	= 0;
	accels_velocity.z 	= 0;
}


#endif