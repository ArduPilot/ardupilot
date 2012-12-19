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


/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01 * fabs(g.channel_throttle.radio_max - g.channel_throttle.radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        g.channel_throttle.radio_out = constrain_int16(g.channel_throttle.radio_out, last_throttle - temp, last_throttle + temp);
    }
}

static void calc_throttle()
{  
   int rov_speed;
   int throttle_target = g.throttle_cruise + throttle_nudge;  
   
   rov_speed = g.airspeed_cruise;    
     
   if (speed_boost)
       rov_speed *= g.booster;
        
   groundspeed_error = rov_speed - (float)ground_speed; 
        
   throttle = throttle_target + g.pidTeThrottle.get_pid(groundspeed_error);
   g.channel_throttle.servo_out = constrain_int16(throttle, 0, g.throttle_max.get());
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

	nav_roll = g.pidNavRoll.get_pid(bearing_error, nav_gain_scaler);	//returns desired bank angle in degrees*100

    if(obstacle) {  // obstacle avoidance 
	    nav_roll += 9000;    // if obstacle in front turn 90° right	
        speed_boost = false;
    }
	nav_roll = constrain_int16(nav_roll, -g.roll_limit.get(), g.roll_limit.get());
}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_I(void)
{
	g.pidNavRoll.reset_I();
	g.pidTeThrottle.reset_I();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = g.channel_throttle.radio_out;

	if((control_mode == MANUAL) || (control_mode == LEARNING)){
		// do a direct pass through of radio values
		g.channel_roll.radio_out 		= g.channel_roll.radio_in;

        if(obstacle)    // obstacle in front, turn right in Stabilize mode
            g.channel_roll.radio_out -= 500;

		g.channel_pitch.radio_out 		= g.channel_pitch.radio_in;

		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
		g.channel_rudder.radio_out 	= g.channel_roll.radio_in;
	} else {       
        g.channel_roll.calc_pwm();
		g.channel_pitch.calc_pwm();
		g.channel_rudder.calc_pwm();             

		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
		g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
    }
                
    if (control_mode >= AUTO) {
        // convert 0 to 100% into PWM
        g.channel_throttle.calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    hal.rcout->write(CH_1, g.channel_roll.radio_out);     // send to Servos
    hal.rcout->write(CH_2, g.channel_pitch.radio_out);     // send to Servos
    hal.rcout->write(CH_3, g.channel_throttle.radio_out);     // send to Servos
    hal.rcout->write(CH_4, g.channel_rudder.radio_out);     // send to Servos
	// Route configurable aux. functions to their respective servos

	g.rc_5.output_ch(CH_5);
	g.rc_6.output_ch(CH_6);
	g.rc_7.output_ch(CH_7);
	g.rc_8.output_ch(CH_8);

#endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) {

    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
        hal.rcout->write(1, 1400);
        mavlink_delay(400);
        hal.rcout->write(1, 1600);
        mavlink_delay(200);
        hal.rcout->write(1, 1500);
#endif
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}
