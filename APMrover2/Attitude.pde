// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************

static void learning()
{
	// Calculate desired servo output for the turn  // Wheels Direction
	// ---------------------------------------------
        
      g.channel_roll.servo_out = nav_roll;
      g.channel_roll.servo_out = g.channel_roll.servo_out * g.turn_gain;
      g.channel_rudder.servo_out = g.channel_roll.servo_out;
}


static void crash_checker()
{
	if(ahrs.pitch_sensor < -4500){
		crash_timer = 255;
	}
	if(crash_timer > 0)
		crash_timer--;
}

static void calc_throttle()
{  int rov_speed;
    
   int throttle_target = g.throttle_cruise + throttle_nudge + 50;   
   
        target_airspeed = g.airspeed_cruise;    
     
   if(speed_boost)
     rov_speed = g.booster * target_airspeed;
   else 
     rov_speed = target_airspeed;   
        
        groundspeed_error = rov_speed - (float)ground_speed; 
        
        int throttle_req = (throttle_target + g.pidTeThrottle.get_pid(groundspeed_error, dTnav)) * 10;
        
        if(g.throttle_slewrate > 0)
        { if (throttle_req > throttle_last) 
	      throttle = throttle + g.throttle_slewrate;
          else if (throttle_req  < throttle_last) {
	      throttle = throttle - g.throttle_slewrate;   
          }
    	  throttle =  constrain(throttle, 500, throttle_req);
          throttle_last = throttle;
        } else {
          throttle = throttle_req;
        }
        g.channel_throttle.servo_out = constrain(((float)throttle / 10.0f), 0, g.throttle_max.get());
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

static void calc_nav_roll()
{

	// Adjust gain based on ground speed
	nav_gain_scaler = (float)ground_speed / (g.airspeed_cruise * 100.0);
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);

	// Calculate the required turn of the wheels rover
	// ----------------------------------------

        // negative error = left turn
	// positive error = right turn

	nav_roll = g.pidNavRoll.get_pid(bearing_error, dTnav, nav_gain_scaler);	//returns desired bank angle in degrees*100
	nav_roll = constrain(nav_roll, -g.roll_limit.get(), g.roll_limit.get());

}

/*****************************************
 * Roll servo slew limit
 *****************************************/
/*
float roll_slew_limit(float servo)
{
	static float last;
	float temp = constrain(servo, last-ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f, last + ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f);
	last = servo;
	return temp;
}*/

/*****************************************
 * Throttle slew limit
 *****************************************/
static void throttle_slew_limit()
{
  /*
	static int last = 1000;
	if(g.throttle_slewrate) {		// if slew limit rate is set to zero then do not slew limit

		float temp = g.throttle_slewrate * G_Dt * 10.f;		//  * 10 to scale % to pwm range of 1000 to 2000
		g.channel_throttle.radio_out = constrain(g.channel_throttle.radio_out, last - (int)temp, last + (int)temp);
		last = g.channel_throttle.radio_out;
	}
*/
}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_I(void)
{
	g.pidNavRoll.reset_I();
	g.pidTeThrottle.reset_I();
//	g.pidAltitudeThrottle.reset_I();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
	int flapSpeedSource = 0;

	// vectorize the rc channels
	RC_Channel_aux* rc_array[NUM_CHANNELS];
	rc_array[CH_1] = NULL;
	rc_array[CH_2] = NULL;
	rc_array[CH_3] = NULL;
	rc_array[CH_4] = NULL;
	rc_array[CH_5] = &g.rc_5;
	rc_array[CH_6] = &g.rc_6;
	rc_array[CH_7] = &g.rc_7;
	rc_array[CH_8] = &g.rc_8;

	if((control_mode == MANUAL) || (control_mode == LEARNING)){
		// do a direct pass through of radio values
		g.channel_roll.radio_out 		= g.channel_roll.radio_in;
		g.channel_pitch.radio_out 		= g.channel_pitch.radio_in;

		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
		g.channel_rudder.radio_out 	= g.channel_roll.radio_in;
	} else {       
                 
                g.channel_roll.calc_pwm();
		g.channel_pitch.calc_pwm();
		g.channel_rudder.calc_pwm();             

		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());

        }
                
        if (control_mode >= FLY_BY_WIRE_B) {
          // convert 0 to 100% into PWM
            g.channel_throttle.calc_pwm();
            /* only do throttle slew limiting in modes where throttle
               control is automatic */
            throttle_slew_limit();
        }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
	APM_RC.OutputCh(CH_1, g.channel_roll.radio_out); // send to Servos
	APM_RC.OutputCh(CH_2, g.channel_pitch.radio_out); // send to Servos
	APM_RC.OutputCh(CH_3, g.channel_throttle.radio_out); // send to Servos
	APM_RC.OutputCh(CH_4, g.channel_rudder.radio_out); // send to Servos
	// Route configurable aux. functions to their respective servos

	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);

#endif
}

static void demo_servos(byte i) {

	while(i > 0){
		gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
		APM_RC.OutputCh(1, 1400);
		mavlink_delay(400);
		APM_RC.OutputCh(1, 1600);
		mavlink_delay(200);
		APM_RC.OutputCh(1, 1500);
#endif
		mavlink_delay(400);
		i--;
	}
}
