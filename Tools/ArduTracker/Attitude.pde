// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************

void stabilize()
{
	float ch1_inf = 1.0;
	float ch2_inf = 1.0;
	float ch4_inf = 1.0;
#if AIRSPEED_SENSOR == ENABLED
	float speed_scaler = STANDARD_SPEED_SQUARED / (airspeed * airspeed);
	speed_scaler = constrain(speed_scaler, 0.11, 9.0);
#endif
#if AIRSPEED_SENSOR == DISABLED
	float speed_scaler;
	if (servo_out[CH_THROTTLE] > 0) 
		speed_scaler = 0.5 + (THROTTLE_CRUISE / servo_out[CH_THROTTLE] / 2.0);	// First order taylor expansion of square root
																				// Should maybe be to the 2/7 power, but we aren't goint to implement that...
	else
		speed_scaler = 1.67;
	speed_scaler = constrain(speed_scaler, 0.6, 1.67);		// This case is constrained tighter as we don't have real speed info
#endif
	
	
	if(crash_timer > 0){
		nav_roll = 0;
	}

	// For Testing Only
	// roll_sensor = (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 10;
	// Serial.print(" roll_sensor ");
	// Serial.print(roll_sensor,DEC);

	// Calculate dersired servo output for the roll 
	// ---------------------------------------------
	servo_out[CH_ROLL]	= pidServoRoll.get_pid((nav_roll - dcm.roll_sensor), deltaMiliSeconds, speed_scaler);
	servo_out[CH_PITCH] = pidServoPitch.get_pid((nav_pitch + fabs(dcm.roll_sensor * get(PARAM_KFF_PTCHCOMP)) - (dcm.pitch_sensor - get(PARAM_TRIM_PITCH))), deltaMiliSeconds, speed_scaler);
	//Serial.print(" servo_out[CH_ROLL] ");
	//Serial.print(servo_out[CH_ROLL],DEC);

	// Mix Stick input to allow users to override control surfaces
	// -----------------------------------------------------------
	if ((control_mode < FLY_BY_WIRE_A) || (ENABLE_STICK_MIXING == 1 && control_mode > FLY_BY_WIRE_B)) {
	
		ch1_inf = (float)radio_in[CH_ROLL] - (float)radio_trim(CH_ROLL);
		ch1_inf = fabs(ch1_inf);
		ch1_inf = min(ch1_inf, 400.0);
		ch1_inf = ((400.0 - ch1_inf) /400.0);
		
		ch2_inf = (float)radio_in[CH_PITCH] - radio_trim(CH_PITCH);
		ch2_inf = fabs(ch2_inf);									
		ch2_inf = min(ch2_inf, 400.0);							
		ch2_inf = ((400.0 - ch2_inf) /400.0);
		
		// scale the sensor input based on the stick input
		// -----------------------------------------------
		servo_out[CH_ROLL]		*= ch1_inf;
		servo_out[CH_PITCH]		*= ch2_inf;
	
		// Mix in stick inputs 
		// -------------------
		servo_out[CH_ROLL]		+=	reverse_roll * 	(radio_in[CH_ROLL]  - radio_trim(CH_ROLL)) * 9;
		servo_out[CH_PITCH]		+=	reverse_pitch * (radio_in[CH_PITCH] - radio_trim(CH_PITCH)) * 9;

		//Serial.print(" servo_out[CH_ROLL] ");
		//Serial.println(servo_out[CH_ROLL],DEC);
	}

	// stick mixing performed for rudder for all cases including FBW unless disabled for higher modes
	// important for steering on the ground during landing
	// -----------------------------------------------
	if (control_mode <= FLY_BY_WIRE_B || ENABLE_STICK_MIXING == 1) {
		ch4_inf = (float)radio_in[CH_RUDDER] - (float)radio_trim(CH_RUDDER);
		ch4_inf = fabs(ch4_inf);									
		ch4_inf = min(ch4_inf, 400.0);							
		ch4_inf = ((400.0 - ch4_inf) /400.0);
	}
	
	// Apply output to Rudder
	// ----------------------
	calc_nav_yaw(speed_scaler);
	servo_out[CH_RUDDER] *= ch4_inf;
	servo_out[CH_RUDDER] += reverse_rudder * (radio_in[CH_RUDDER] - radio_trim(CH_RUDDER)) * 9;
	
	// Call slew rate limiter if used
	// ------------------------------
	//#if(ROLL_SLEW_LIMIT != 0)
	//	servo_out[CH_ROLL] = roll_slew_limit(servo_out[CH_ROLL]);
	//#endif	
}

void crash_checker()
{
	if(dcm.pitch_sensor < -4500){
		crash_timer = 255;
	}
	if(crash_timer > 0)
		crash_timer--;
}


#if AIRSPEED_SENSOR == DISABLED
void calc_throttle()
{
	int throttle_target = get(PARAM_TRIM_THROTTLE) + throttle_nudge;
	
	// no airspeed sensor, we use nav pitch to determine the proper throttle output
	// AUTO, RTL, etc
	// ---------------------------------------------------------------------------
	if (nav_pitch >= 0) {
		servo_out[CH_THROTTLE] = throttle_target + (get(PARAM_THR_MAX) - throttle_target) * nav_pitch / get(PARAM_LIM_PITCH_MAX);
	} else {
		servo_out[CH_THROTTLE] = throttle_target - (throttle_target - get(PARAM_THR_MIN)) * nav_pitch / get(PARAM_LIM_PITCH_MIN);
	}
	
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], get(PARAM_THR_MIN), get(PARAM_THR_MAX));
}
#endif

#if AIRSPEED_SENSOR == ENABLED
void calc_throttle()
{
	// throttle control with airspeed compensation	 
	// -------------------------------------------
	energy_error = airspeed_energy_error + (float)altitude_error * 0.098f;
	
	// positive energy errors make the throttle go higher
	servo_out[CH_THROTTLE] = get(PARAM_TRIM_THROTTLE) + pidTeThrottle.get_pid(energy_error, dTnav);
	servo_out[CH_THROTTLE] = max(servo_out[CH_THROTTLE], 0);

	// are we going too slow? up the throttle to get some more groundspeed
	// move into PID loop in the future
	// lower the contstant value to limit the effect : 50 = default
	
	//JASON - We really should change this to act on airspeed in this case.  Let's visit about it....
	/*int gs_boost = 30 * (1.0 - ((float)gps.ground_speed / (float)airspeed_cruise));
	gs_boost = max(0,gs_boost);
	servo_out[CH_THROTTLE] += gs_boost;*/
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 
			get(PARAM_THR_MIN), get(PARAM_THR_MAX));
}
#endif



/*****************************************
 * Calculate desired roll/pitch/yaw angles (in medium freq loop)
 *****************************************/

//  Yaw is separated into a function for future implementation of heading hold on rolling take-off
// ----------------------------------------------------------------------------------------
void calc_nav_yaw(float speed_scaler)
{
#if HIL_MODE != HIL_MODE_ATTITUDE
	Vector3f temp = imu.get_accel();
	long error = -temp.y;

	// Control is a feedforward from the aileron control + a PID to coordinate the turn (drive y axis accel to zero)
	servo_out[CH_RUDDER] = get(PARAM_KFF_RDDRMIX) * servo_out[CH_ROLL] + pidServoRudder.get_pid(error, deltaMiliSeconds, speed_scaler);
#else
	servo_out[CH_RUDDER] = get(PARAM_KFF_RDDRMIX) * servo_out[CH_ROLL];
	// XXX probably need something here based on heading
#endif
}


void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
#if AIRSPEED_SENSOR == ENABLED
	nav_pitch = -pidNavPitchAirspeed.get_pid(airspeed_error, dTnav);
#else
	nav_pitch = pidNavPitchAltitude.get_pid(altitude_error, dTnav);
#endif
	nav_pitch = constrain(nav_pitch, get(PARAM_LIM_PITCH_MIN), get(PARAM_LIM_PITCH_MAX));
}


void calc_nav_roll()
{

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// This does not make provisions for wind speed in excess of airframe speed
	nav_gain_scaler = (float)gps.ground_speed / (STANDARD_SPEED * 100.0);
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);
	
	// negative error = left turn
	// positive error = right turn
	// Calculate the required roll of the plane
	// ----------------------------------------
	nav_roll = pidNavRoll.get_pid(bearing_error, dTnav, nav_gain_scaler);	//returns desired bank angle in degrees*100
	nav_roll = constrain(nav_roll,-get(PARAM_LIM_ROLL), get(PARAM_LIM_ROLL));
	
}


/*****************************************
 * Roll servo slew limit
 *****************************************/
/*
float roll_slew_limit(float servo)
{
	static float last;
	float temp = constrain(servo, last-ROLL_SLEW_LIMIT * deltaMiliSeconds/1000.f, last + ROLL_SLEW_LIMIT * deltaMiliSeconds/1000.f);
	last = servo;
	return temp;
}*/

/*****************************************
 * Throttle slew limit
 *****************************************/
/*float throttle_slew_limit(float throttle)
{
	static float last;
	float temp = constrain(throttle, last-THROTTLE_SLEW_LIMIT * deltaMiliSeconds/1000.f, last + THROTTLE_SLEW_LIMIT * deltaMiliSeconds/1000.f);
	last = throttle;
	return temp;
}
*/

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
	pidNavRoll.reset_I();
	pidNavPitchAirspeed.reset_I();
	pidNavPitchAltitude.reset_I();
	pidTeThrottle.reset_I();
	pidAltitudeThrottle.reset_I();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
void set_servos_4(void)
{
	if(control_mode == MANUAL){
		// do a direct pass through of radio values
		for(int y=0; y<4; y++) {
			radio_out[y] = radio_in[y];
		}
		
	} else {
		// Patch Antenna Control Code
		float phi, theta; //,servo_phi, servo_theta;
		float x1,x2,y1,y2,x,y,r,z;
		
		y1 = 110600*current_loc.lat/t7;
		x1 = (PI/180)*6378137*(cos(atan(0.99664719*tan(current_loc.lat/t7*PI/180))))*(current_loc.lng/t7);

		y2 = 110600*trackVehicle_loc.lat/t7;
		x2 = (PI/180)*6378137*(cos(atan(0.99664719*tan(current_loc.lat/t7*PI/180))))*(trackVehicle_loc.lng/t7);

		x = abs(x2 - x1);
		y = abs(y2 - y1);

		r = sqrt(x*x+y*y);
		z = trackVehicle_loc.alt/100.0 - current_loc.alt;

		phi   = (atan(z/r)*180/PI);
		theta = (atan(x/y)*180/PI);
		// Check to see which quadrant of the angle
		
		if (trackVehicle_loc.lat >= current_loc.lat && trackVehicle_loc.lng >= current_loc.lng)
		{
			theta = abs(theta);
		}
		else if (trackVehicle_loc.lat >= current_loc.lat && trackVehicle_loc.lng <= current_loc.lng)
		{
			theta = 360 - abs(theta);
		}
		else if (trackVehicle_loc.lat <= current_loc.lat && trackVehicle_loc.lng >= current_loc.lng)		
		{
			theta = 180 - abs(theta);
		}
		else if (trackVehicle_loc.lat <= current_loc.lat && trackVehicle_loc.lng <= current_loc.lng)
		{
			theta = 180 +  abs(theta);
		}
	
		// Calibration of the top servo
		//servo_phi   = (91*phi + 7650)/9;

		// Calibration of the bottom servo
		//servo_theta = (2*theta + 5940)/3;
		

		Serial.print("target lat: "); Serial.println(current_loc.lat);
		Serial.print("tracker lat: "); Serial.println(trackVehicle_loc.lat);
		Serial.print("phi: "); Serial.println(phi);
		Serial.print("theta: "); Serial.println(theta);
		
		// Outputing to the servos
		servo_out[CH_ROLL] = 10000*phi/90.0;
		servo_out[CH_PITCH] = 10000*theta/360.0;
		servo_out[CH_RUDDER] = 0;
		servo_out[CH_THROTTLE] = 0;

		radio_out[CH_ROLL] 		= radio_trim(CH_ROLL)   + (reverse_roll   * ((float)servo_out[CH_ROLL]   * 0.11111));
		radio_out[CH_PITCH] 	= radio_trim(CH_PITCH)  + (reverse_pitch  * ((float)servo_out[CH_PITCH]  * 0.11111));
		radio_out[CH_RUDDER] 	=  0;
		radio_out[CH_THROTTLE]  = 0;

		/*
		if (mix_mode == 0){
			radio_out[CH_ROLL] 		= radio_trim(CH_ROLL)   + (reverse_roll   * ((float)servo_out[CH_ROLL]   * 0.11111));
			radio_out[CH_PITCH] 	= radio_trim(CH_PITCH)  + (reverse_pitch  * ((float)servo_out[CH_PITCH]  * 0.11111));
			radio_out[CH_RUDDER] 	= radio_trim(CH_RUDDER) + (reverse_rudder * ((float)servo_out[CH_RUDDER] * 0.11111));
		}else{
			//Elevon mode
			float ch1;
			float ch2;
			ch1 = reverse_elevons * (servo_out[CH_PITCH] - servo_out[CH_ROLL]);	
			ch2 = servo_out[CH_PITCH] + servo_out[CH_ROLL];
			radio_out[CH_ROLL] =	elevon1_trim + (reverse_ch1_elevon * (ch1 * 0.11111));
			radio_out[CH_PITCH] =	elevon2_trim + (reverse_ch2_elevon * (ch2 * 0.11111));
		}
		
		#if THROTTLE_OUT == 0
			radio_out[CH_THROTTLE] = 0;
		#endif
		
	
		// convert 0 to 100% into PWM
		servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 0, 100);
		radio_out[CH_THROTTLE] = servo_out[CH_THROTTLE] * ((radio_max(CH_THROTTLE) - radio_min(CH_THROTTLE)) / 100);
		radio_out[CH_THROTTLE] += radio_min(CH_THROTTLE);

		//Radio_in: 1763	PWM output: 2000	Throttle: 78.7695999145	PWM Min: 1110	PWM Max: 1938

		#if THROTTLE_REVERSE == 1
			radio_out[CH_THROTTLE] = radio_max(CH_THROTTLE) + radio_min(CH_THROTTLE) - radio_out[CH_THROTTLE];
		#endif
		*/
	}

	// send values to the PWM timers for output
	// ----------------------------------------
	for(int y=0; y<4; y++) { 
		APM_RC.OutputCh(y, radio_out[y]); // send to Servos
	}
 }

void demo_servos(byte i) {
	
	while(i > 0){
		gcs.send_text(SEVERITY_LOW,"Demo Servos!");
		APM_RC.OutputCh(1, 1400);
		delay(400);
		APM_RC.OutputCh(1, 1600);
		delay(200);
		APM_RC.OutputCh(1, 1500);
		delay(400);
		i--;
	}
}

int readOutputCh(unsigned char ch)
{
 int pwm;
 switch(ch)
	{
		case 0:	pwm = OCR5B; break;	// ch0
		case 1:	pwm = OCR5C; break;	// ch1
		case 2:	pwm = OCR1B; break;	// ch2
		case 3:	pwm = OCR1C; break;	// ch3
		case 4:	pwm = OCR4C; break;	// ch4
		case 5:	pwm = OCR4B; break;	// ch5
		case 6:	pwm = OCR3C; break;	// ch6
		case 7:	pwm = OCR3B; break;	// ch7
		case 8:	pwm = OCR5A; break;	// ch8,	PL3
		case 9:	pwm = OCR1A; break;	// ch9,	PB5
		case 10: pwm = OCR3A; break;	// ch10, PE3
	}
	pwm >>= 1;	 // pwm / 2;
	return pwm;
}
