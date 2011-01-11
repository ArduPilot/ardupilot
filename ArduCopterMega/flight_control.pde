
/*************************************************************
throttle control
****************************************************************/

// user input:
// -----------
void output_manual_throttle()
{
	rc_3.servo_out = rc_3.control_in;
	rc_3.servo_out = (float)rc_3.servo_out * angle_boost();
}

// Autopilot
// ---------
void output_auto_throttle()
{
	rc_3.servo_out 	= (float)nav_throttle * angle_boost();
	rc_3.servo_out = max(rc_3.servo_out, 1);
}

void calc_nav_throttle()
{
	long t_out;
	
	if(altitude_sensor == BARO) {
		t_out = pid_baro_throttle.get_pid(altitude_error, deltaMiliSeconds, 1.0);
	
		// limit output of throttle control
		t_out = throttle_cruise + constrain(t_out, -50, 100);
	} else {
		// SONAR
		t_out = pid_sonar_throttle.get_pid(altitude_error, deltaMiliSeconds, 1.0);
	
		// limit output of throttle control
		t_out = throttle_cruise + constrain(t_out, -60, 100);
	}
	
	nav_throttle = (float)(throttle_cruise + t_out) * angle_boost();
}

float angle_boost()
{
	//static byte flipper;
	//float temp = 1 / (cos(dcm.roll) * cos(dcm.pitch));
	float temp =  (1.0 - (cos(dcm.roll) * cos(dcm.pitch)));
	temp = 1.0 + (temp / 1.5);
	
	// limit the boost value
	if(temp > 1.3) 
		temp = 1.3;
	return temp;
}


/*************************************************************
yaw control
****************************************************************/

void output_manual_yaw()
{
	if(rc_3.control_in == 0){
		clear_yaw_control();
	} else {	
		// Yaw control
		if(rc_4.control_in == 0){
			//clear_yaw_control();
			output_yaw_with_hold(true); // hold yaw
		}else{
			output_yaw_with_hold(false); // rate control yaw
		}
	}
}

void auto_yaw()
{
	output_yaw_with_hold(true); // hold yaw
}

/*************************************************************
picth and roll control
****************************************************************/

/*// how hard to tilt towards the target
// -----------------------------------
void calc_nav_pid()
{
	// how hard to pitch to target
	
	nav_angle 	= pid_nav.get_pid(wp_distance * 100, dTnav, 1.0);
	nav_angle 	= constrain(nav_angle, -pitch_max, pitch_max);
}

// distribute the pitch angle based on our orientation
// ---------------------------------------------------
void calc_nav_pitch()
{
	// how hard to pitch to target

	long angle 	= wrap_360(nav_bearing - yaw_sensor);
	
	bool rev = false;
	float roll_out;
	
	if(angle > 18000){
		angle -= 18000;
		rev = true;
	}
	
	roll_out = abs(angle - 18000);
	roll_out = (9000.0 - roll_out) / 9000.0;
	roll_out = (rev) ? roll_out : -roll_out;

	nav_pitch = (float)nav_angle * roll_out;
}

// distribute the roll angle based on our orientation
// --------------------------------------------------
void calc_nav_roll()
{
	long angle 	= wrap_360(nav_bearing - yaw_sensor);

	bool rev = false;
	float roll_out;
	
	if(angle > 18000){
		angle -= 18000;
		rev = true;
	}
	
	roll_out = abs(angle - 9000);
	roll_out = (9000.0 - roll_out) / 9000.0;
	roll_out = (rev) ? -roll_out : roll_out;

	nav_roll = (float)nav_angle * roll_out;
}
*/










