//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************

void stabilize()
{
	float ch1_inf = 1.0;
	float ch2_inf = 1.0;
	float ch4_inf = 1.0;
	
	if(crash_timer > 0){
		nav_roll = 0;
	}

	// For Testing Only
	// roll_sensor = (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 10;
	// Serial.print(" roll_sensor ");
	// Serial.print(roll_sensor,DEC);

	// Calculate dersired servo output for the roll 
	// ---------------------------------------------
	servo_out[CH_ROLL]	= PID((nav_roll - roll_sensor), deltaMiliSeconds, CASE_SERVO_ROLL, 1.0);	
	servo_out[CH_PITCH] = PID((nav_pitch + abs(roll_sensor * kff[CASE_PITCH_COMP]) - pitch_sensor), deltaMiliSeconds, CASE_SERVO_PITCH, 1.0);  
	//Serial.print(" servo_out[CH_ROLL] ");
	//Serial.print(servo_out[CH_ROLL],DEC);

	// Mix Stick input to allow users to override control surfaces
	// -----------------------------------------------------------
	if ((control_mode < FLY_BY_WIRE_A) || (ENABLE_STICK_MIXING == 1 && control_mode > FLY_BY_WIRE_B)) {
	
		ch1_inf = (float)radio_in[CH_ROLL] - (float)radio_trim[CH_ROLL];
		ch1_inf = abs(ch1_inf);
		ch1_inf = min(ch1_inf, 400.0);
		ch1_inf = ((400.0 - ch1_inf) /400.0);
		
		ch2_inf = (float)radio_in[CH_PITCH] - radio_trim[CH_PITCH];
		ch2_inf = abs(ch2_inf);									
		ch2_inf = min(ch2_inf, 400.0);							
		ch2_inf = ((400.0 - ch2_inf) /400.0);
		
		// scale the sensor input based on the stick input
		// -----------------------------------------------
		servo_out[CH_ROLL]		*= ch1_inf;
		servo_out[CH_PITCH]		*= ch2_inf;
	
		// Mix in stick inputs 
		// -------------------
		servo_out[CH_ROLL]		+=	reverse_roll * 	(radio_in[CH_ROLL]  - radio_trim[CH_ROLL]) * 9;
		servo_out[CH_PITCH]		+=	reverse_pitch * (radio_in[CH_PITCH] - radio_trim[CH_PITCH]) * 9;

		//Serial.print(" servo_out[CH_ROLL] ");
		//Serial.println(servo_out[CH_ROLL],DEC);
	}

	// stick mixing performed for rudder for all cases including FBW unless disabled for higher modes
	// important for steering on the ground during landing
	// -----------------------------------------------
	if (control_mode <= FLY_BY_WIRE_B || ENABLE_STICK_MIXING == 1) {
		ch4_inf = (float)radio_in[CH_RUDDER] - (float)radio_trim[CH_RUDDER];
		ch4_inf = abs(ch4_inf);									
		ch4_inf = min(ch4_inf, 400.0);							
		ch4_inf = ((400.0 - ch4_inf) /400.0);
	}
	
	// Apply output to Rudder
	// ----------------------
	calc_nav_yaw();
	servo_out[CH_RUDDER] *= ch4_inf;
	servo_out[CH_RUDDER] += reverse_rudder * (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 9;
	
	// Call slew rate limiter if used
	// ------------------------------
	//#if(ROLL_SLEW_LIMIT != 0)
	//	servo_out[CH_ROLL] = roll_slew_limit(servo_out[CH_ROLL]);
	//#endif	
}

void crash_checker()
{
	if(pitch_sensor < -4500){
		crash_timer = 255;
	}
	if(crash_timer > 0)
		crash_timer--;
}


#if AIRSPEED_SENSOR == 0
void calc_throttle()
{
	// no airspeed sensor, we use nav pitch to determin the proper throttle output
	// AUTO, RTL, etc
	// ---------------------------------------------------------------------------
	if (nav_pitch >= 0) {
		servo_out[CH_THROTTLE] = throttle_cruise + (THROTTLE_MAX - throttle_cruise) * nav_pitch / pitch_max;
	} else {
		servo_out[CH_THROTTLE] = throttle_cruise - (throttle_cruise - THROTTLE_MIN) * nav_pitch / pitch_min;
	}
	servo_out[CH_THROTTLE] = max(servo_out[CH_THROTTLE], 0);
	
	// are we going too slow? up the throttle to get some more groundspeed
	// move into PID loop in the future
	// lower the contstant value to limit the effect : 50 = default
	int gs_boost = 30 * (1.0 - ((float)GPS.ground_speed / (float)airspeed_cruise));
	gs_boost = max(0, gs_boost);
	servo_out[CH_THROTTLE] += gs_boost;
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], THROTTLE_MIN, THROTTLE_MAX);
}
#endif

#if AIRSPEED_SENSOR == 1
void calc_throttle()
{
	// throttle control with airspeed compensation	 
	// -------------------------------------------
	energy_error = airspeed_energy_error + (float)altitude_error * 0.098f;
	
	// positive energy errors make the throttle go higher
	servo_out[CH_THROTTLE] = throttle_cruise + PID(energy_error, dTnav, CASE_TE_THROTTLE, 1.0);
	servo_out[CH_THROTTLE] = max(servo_out[CH_THROTTLE], 0);

	// are we going too slow? up the throttle to get some more groundspeed
	// move into PID loop in the future
	// lower the contstant value to limit the effect : 50 = default
	
	//JASON - We really should change this to act on airspeed in this case.  Let's visit about it....
	/*int gs_boost = 30 * (1.0 - ((float)GPS.ground_speed / (float)airspeed_cruise));
	gs_boost = max(0,gs_boost);
	servo_out[CH_THROTTLE] += gs_boost;*/
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], THROTTLE_MIN, THROTTLE_MAX);
}
#endif



/*****************************************
 * Calculate desired roll angle (in medium freq loop)
 *****************************************/
//  Yaw is separated into a function for future implementation of heading hold on take-off
// ----------------------------------------------------------------------------------------
void calc_nav_yaw()
{
	// read_adc(4) is y axis accel;
	// Control is a feedforward from the aileron control + a PID to coordinate the turn (drive y axis accel to zero)
	servo_out[CH_RUDDER] = kff[CASE_RUDDER_MIX] * servo_out[CH_ROLL] + PID((long)read_adc(4), deltaMiliSeconds, CASE_SERVO_RUDDER, 1.0);

}


#if AIRSPEED_SENSOR == 1
void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	nav_pitch = -PID(airspeed_error, dTnav, CASE_NAV_PITCH_ASP, 1.0);
	nav_pitch = constrain(nav_pitch, pitch_min, pitch_max);
}
#endif

#if AIRSPEED_SENSOR == 0
void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	nav_pitch = PID(altitude_error, dTnav, CASE_NAV_PITCH_ALT, 1.0);
	nav_pitch = constrain(nav_pitch, pitch_min, pitch_max);
}
#endif


void calc_nav_roll()
{

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// This does not make provisions for wind speed in excess of airframe speed
	nav_gain_scaler = (float)GPS.ground_speed / (float)(AIRSPEED_CRUISE * 100);
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);

	// Jason ->  I will implement the servo gain scaler some time, but it is independant of this scaling.
	// Doug to implement the high speed servo gain scale
	// use head max to limit turns, make a var
	
	// negative error = left turn
	// positive error = right turn
	// Calculate the required roll of the plane
	// ----------------------------------------
	nav_roll = PID(bearing_error, dTnav, CASE_NAV_ROLL, nav_gain_scaler);	//returns desired bank angle in degrees*100
	nav_roll = constrain(nav_roll,-head_max, head_max);
	
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

/*****************************************
 * Proportional Integrator Derivative Control 
 *****************************************/

float PID(long PID_error, long dt, int PID_case, float scaler)
{
	// PID_case is used to keep track of which PID controller is being used - e.g.	PID_servo_out[CH_ROLL]
	float output, derivative;
 
	derivative 				= 1000.0f * (float)(PID_error - last_error[PID_case]) / (float)dt;
	last_error[PID_case] 	= PID_error;
	output = (kp[PID_case] * scaler * (float)PID_error);			// Compute proportional component
															//Positive error produces positive output

	integrator[PID_case] 	+= (float)PID_error * ki[PID_case] * scaler * (float)dt / 1000.0f; 
	integrator[PID_case] 	= constrain(integrator[PID_case], -1.0*(float)integrator_max[PID_case], (float)integrator_max[PID_case]);
	
	output += integrator[PID_case];							// Add the integral component
	output += kd[PID_case] * scaler * derivative;			// Add the derivative component
	return output;
}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
		integrator[CASE_NAV_ROLL] 		= 0;
		integrator[CASE_NAV_PITCH_ASP] 	= 0;
		integrator[CASE_NAV_PITCH_ALT] 	= 0;
		integrator[CASE_TE_THROTTLE] 	= 0;
		integrator[CASE_ALT_THROTTLE] 	= 0;

		last_error[CASE_NAV_ROLL] 		= 0;
		last_error[CASE_NAV_PITCH_ASP] 	= 0;
		last_error[CASE_NAV_PITCH_ALT] 	= 0;
		last_error[CASE_TE_THROTTLE] 	= 0;
		last_error[CASE_ALT_THROTTLE] 	= 0;
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
		if (mix_mode == 0){
			radio_out[CH_ROLL] 		= radio_trim[CH_ROLL]   + (reverse_roll   * ((float)servo_out[CH_ROLL]   * 0.11111));
			radio_out[CH_PITCH] 	= radio_trim[CH_PITCH]  + (reverse_pitch  * ((float)servo_out[CH_PITCH]  * 0.11111));
			radio_out[CH_RUDDER] 	= radio_trim[CH_RUDDER] + (reverse_rudder * ((float)servo_out[CH_RUDDER] * 0.11111));
		}else{
			/*Elevon mode*/
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
		radio_out[CH_THROTTLE] = servo_out[CH_THROTTLE] * ((radio_max[CH_THROTTLE] - radio_min[CH_THROTTLE]) / 100);
		radio_out[CH_THROTTLE] += radio_min[CH_THROTTLE];

		//Radio_in: 1763	PWM output: 2000	Throttle: 78.7695999145	PWM Min: 1110	PWM Max: 1938

		#if THROTTLE_REVERSE == 1
			radio_out[CH_THROTTLE] = radio_max[CH_THROTTLE] + radio_min[CH_THROTTLE] - radio_out[CH_THROTTLE];
		#endif
	}

	// send values to the PWM timers for output
	// ----------------------------------------
	for(int y=0; y<4; y++) { 
		//radio_out[y] = constrain(radio_out[y], 	radio_min[y], 	radio_max[y]);	
		APM_RC.OutputCh(y, radio_out[y]); // send to Servos
		//Serial.print(radio_out[y],DEC);
		//Serial.print(", ");
	}
		//Serial.println(" ");
 }

void demo_servos(byte i) {
	
	while(i > 0){
		send_message(SEVERITY_LOW,"Demo Servos!");
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
