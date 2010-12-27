
// desired angle in
// motor commands out (in degrees)
void init_pids()
{
	max_stabilize_dampener 	= pid_stabilize_roll.kP() * 2500;
	stabilze_dampener 		= 5729.57795 * stabilize_rate_roll_pitch;

	max_yaw_dampener		= pid_yaw.kP() * 6000;				// .3 * 6000 = 1800
	stabilze_yaw_dampener 	= 5729.57795 * stabilize_rate_yaw; 	// .3
}

void output_stabilize()
{
	float roll_error, pitch_error;
	Vector3f omega = dcm.get_gyro();
	
	//pitch_sensor = roll_sensor = 0; // testing only
	
	// control +- 45° is mixed with the navigation request by the Autopilot
	// output is in degrees = target pitch and roll of copter
	rc_1.servo_out = rc_1.control_mix(nav_roll);
	rc_2.servo_out = rc_2.control_mix(nav_pitch);
		
	roll_error 		= rc_1.servo_out - roll_sensor;
	pitch_error 	= rc_2.servo_out - pitch_sensor;
	yaw_error		= nav_yaw - yaw_sensor;
	yaw_error 		= wrap_180(yaw_error);
	
	// limit the error we're feeding to the PID
	roll_error 		= constrain(roll_error,  -2500, 2500);
	pitch_error 	= constrain(pitch_error, -2500, 2500);
	yaw_error		= constrain(yaw_error,   -6000, 6000);

	//Serial.printf("s: %d \t mix  %d, err  %d", (int)roll_sensor, (int)rc_1.servo_out, (int)roll_error);

	// write out angles back to servo out - this will be converted to PWM by RC_Channel
	rc_1.servo_out 	= pid_stabilize_roll.get_pid(roll_error,  	deltaMiliSeconds, 1.0);
	rc_2.servo_out 	= pid_stabilize_pitch.get_pid(pitch_error, 	deltaMiliSeconds, 1.0);
	rc_4.servo_out 	= pid_yaw.get_pid(yaw_error, 				deltaMiliSeconds, 1.0); // .3 = 198pwm

	//Serial.printf("\tpid: %d", (int)rc_1.servo_out);

	// We adjust the output by the rate of rotation:
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading
	int roll_dampener 	= (omega.x * stabilze_dampener);// Omega is in radians
	int pitch_dampener 	= (omega.y * stabilze_dampener);
	int yaw_dampener 	= (omega.z * stabilze_yaw_dampener);
	
	// Limit dampening to be equal to propotional term for symmetry
	rc_1.servo_out	-= constrain(roll_dampener,  -max_stabilize_dampener, max_stabilize_dampener);	// +- 15°
	rc_2.servo_out	-= constrain(pitch_dampener, -max_stabilize_dampener, max_stabilize_dampener);	// +- 15°
	rc_4.servo_out	-= constrain(yaw_dampener,   -max_yaw_dampener, 	  max_yaw_dampener);
	

	//Serial.printf(" yaw out: %d, d: %d", (int)rc_4.angle_to_pwm(), yaw_dampener);

	//Serial.printf("\trd: %d", roll_dampener);
	//Serial.printf("\tlimit: %d, PWM: %d", rc_1.servo_out, rc_1.angle_to_pwm());	
}

//	err  -2500	pid: -1100	rd: 1117	limit: -1650, PWM: -152 
//s: -1247 	 mix  0, err  1247	pid: 548	rd: -153	limit: 395, PWM: 35 

void output_rate_control()
{
	Vector3f omega = dcm.get_gyro();

	rc_4.servo_out = rc_4.control_in;
	rc_1.servo_out = rc_2.control_in;
	rc_2.servo_out = rc_2.control_in;
	
	// Rate control through bias corrected gyro rates
	// omega is the raw gyro reading plus Omega_I, so it´s bias corrected
	rc_1.servo_out 	-= (omega.x * 5729.57795 * acro_rate_roll_pitch);
	rc_2.servo_out 	-= (omega.y * 5729.57795 * acro_rate_roll_pitch);
	rc_4.servo_out  -= (omega.z * 5729.57795 * acro_rate_yaw);

	//Serial.printf("\trated out %d, omega ", rc_1.servo_out);
	//Serial.print((Omega[0] * 5729.57795 * stabilize_rate_roll_pitch), 3);

	// Limit output
	rc_1.servo_out  = constrain(rc_1.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);
	rc_2.servo_out 	= constrain(rc_2.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);
	rc_4.servo_out  = constrain(rc_4.servo_out, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);	
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
	pid_nav.reset_I();
	pid_throttle.reset_I();
}



