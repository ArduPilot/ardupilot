// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************

static void stabilize()
{
	float ch1_inf = 1.0;
	float ch2_inf = 1.0;
	float ch4_inf = 1.0;
	float speed_scaler;

	if (g.airspeed_enabled == true){
		if(airspeed > 0)
			speed_scaler = (STANDARD_SPEED * 100) / airspeed;
		else
			speed_scaler = 2.0;
			speed_scaler = constrain(speed_scaler, 0.5, 2.0);
	} else {
		if (g.channel_throttle.servo_out > 0){
			speed_scaler = 0.5 + ((float)THROTTLE_CRUISE / g.channel_throttle.servo_out / 2.0);	// First order taylor expansion of square root
																				// Should maybe be to the 2/7 power, but we aren't goint to implement that...
		}else{
			speed_scaler = 1.67;
		}
		speed_scaler = constrain(speed_scaler, 0.6, 1.67);		// This case is constrained tighter as we don't have real speed info
	}

	if(crash_timer > 0){
		nav_roll = 0;
	}

    if (inverted_flight) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll += 18000;
        if (dcm.roll_sensor < 0) nav_roll -= 36000;
    }

	// For Testing Only
	// roll_sensor = (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 10;
	// Serial.printf_P(PSTR(" roll_sensor "));
	// Serial.print(roll_sensor,DEC);

	// Calculate dersired servo output for the roll
	// ---------------------------------------------
	g.channel_roll.servo_out = g.pidServoRoll.get_pid((nav_roll - dcm.roll_sensor), delta_ms_fast_loop, speed_scaler);
	long tempcalc = nav_pitch +
	        fabs(dcm.roll_sensor * g.kff_pitch_compensation) +
	        (g.channel_throttle.servo_out * g.kff_throttle_to_pitch) -
	        (dcm.pitch_sensor - g.pitch_trim);
    if (inverted_flight) {
        // when flying upside down the elevator control is inverted
        tempcalc = -tempcalc;
    }
	g.channel_pitch.servo_out = g.pidServoPitch.get_pid(tempcalc, delta_ms_fast_loop, speed_scaler);

	// Mix Stick input to allow users to override control surfaces
	// -----------------------------------------------------------
	if ((control_mode < FLY_BY_WIRE_A) ||
        (ENABLE_STICK_MIXING == 1 &&
         geofence_stickmixing() &&
         control_mode > FLY_BY_WIRE_B &&
         failsafe == FAILSAFE_NONE)) {

		// TODO: use RC_Channel control_mix function?
		ch1_inf = (float)g.channel_roll.radio_in - (float)g.channel_roll.radio_trim;
		ch1_inf = fabs(ch1_inf);
		ch1_inf = min(ch1_inf, 400.0);
		ch1_inf = ((400.0 - ch1_inf) /400.0);

		ch2_inf = (float)g.channel_pitch.radio_in - g.channel_pitch.radio_trim;
		ch2_inf = fabs(ch2_inf);
		ch2_inf = min(ch2_inf, 400.0);
		ch2_inf = ((400.0 - ch2_inf) /400.0);

		// scale the sensor input based on the stick input
		// -----------------------------------------------
		g.channel_roll.servo_out *= ch1_inf;
		g.channel_pitch.servo_out *= ch2_inf;

		// Mix in stick inputs
		// -------------------
		g.channel_roll.servo_out +=	g.channel_roll.pwm_to_angle();
		g.channel_pitch.servo_out +=	g.channel_pitch.pwm_to_angle();

		//Serial.printf_P(PSTR(" servo_out[CH_ROLL] "));
		//Serial.println(servo_out[CH_ROLL],DEC);
	}

	// stick mixing performed for rudder for all cases including FBW unless disabled for higher modes
	// important for steering on the ground during landing
	// -----------------------------------------------
	if (control_mode <= FLY_BY_WIRE_B ||
        (ENABLE_STICK_MIXING == 1 &&
         geofence_stickmixing() &&
         failsafe == FAILSAFE_NONE)) {
		ch4_inf = (float)g.channel_rudder.radio_in - (float)g.channel_rudder.radio_trim;
		ch4_inf = fabs(ch4_inf);
		ch4_inf = min(ch4_inf, 400.0);
		ch4_inf = ((400.0 - ch4_inf) /400.0);
	}

	// Apply output to Rudder
	// ----------------------
	calc_nav_yaw(speed_scaler);
	g.channel_rudder.servo_out *= ch4_inf;
	g.channel_rudder.servo_out += g.channel_rudder.pwm_to_angle();

	// Call slew rate limiter if used
	// ------------------------------
	//#if(ROLL_SLEW_LIMIT != 0)
	//	g.channel_roll.servo_out = roll_slew_limit(g.channel_roll.servo_out);
	//#endif
}

static void crash_checker()
{
	if(dcm.pitch_sensor < -4500){
		crash_timer = 255;
	}
	if(crash_timer > 0)
		crash_timer--;
}


static void calc_throttle()
{
  if (g.airspeed_enabled == false) {
	int throttle_target = g.throttle_cruise + throttle_nudge;

    // TODO: think up an elegant way to bump throttle when
    // groundspeed_undershoot > 0 in the no airspeed sensor case; PID
    // control?


		// no airspeed sensor, we use nav pitch to determine the proper throttle output
		// AUTO, RTL, etc
		// ---------------------------------------------------------------------------
		if (nav_pitch >= 0) {
			g.channel_throttle.servo_out = throttle_target + (g.throttle_max - throttle_target) * nav_pitch / g.pitch_limit_max;
		} else {
			g.channel_throttle.servo_out = throttle_target - (throttle_target - g.throttle_min) * nav_pitch / g.pitch_limit_min;
		}

		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
	} else {
		// throttle control with airspeed compensation
		// -------------------------------------------
		energy_error = airspeed_energy_error + (float)altitude_error * 0.098f;

		// positive energy errors make the throttle go higher
		g.channel_throttle.servo_out = g.throttle_cruise + g.pidTeThrottle.get_pid(energy_error, dTnav);
		g.channel_throttle.servo_out += (g.channel_pitch.servo_out * g.kff_pitch_to_throttle);

		g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out,
			g.throttle_min.get(), g.throttle_max.get());			// TODO - resolve why "saved" is used here versus "current"
	}

}

/*****************************************
 * Calculate desired roll/pitch/yaw angles (in medium freq loop)
 *****************************************/

//  Yaw is separated into a function for future implementation of heading hold on rolling take-off
// ----------------------------------------------------------------------------------------
static void calc_nav_yaw(float speed_scaler)
{
#if HIL_MODE != HIL_MODE_ATTITUDE
	Vector3f temp = imu.get_accel();
	long error = -temp.y;

	// Control is a feedforward from the aileron control + a PID to coordinate the turn (drive y axis accel to zero)
	g.channel_rudder.servo_out = g.kff_rudder_mix * g.channel_roll.servo_out + g.pidServoRudder.get_pid(error, delta_ms_fast_loop, speed_scaler);
#else
	g.channel_rudder.servo_out = g.kff_rudder_mix * g.channel_roll.servo_out;
	// XXX probably need something here based on heading
#endif
}


static void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	if (g.airspeed_enabled == true) {
		nav_pitch = -g.pidNavPitchAirspeed.get_pid(airspeed_error, dTnav);
	} else {
		nav_pitch = g.pidNavPitchAltitude.get_pid(altitude_error, dTnav);
    }
	nav_pitch = constrain(nav_pitch, g.pitch_limit_min.get(), g.pitch_limit_max.get());
}


#define YAW_DAMPENER 0

static void calc_nav_roll()
{

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// This does not make provisions for wind speed in excess of airframe speed
	nav_gain_scaler = (float)g_gps->ground_speed / (STANDARD_SPEED * 100.0);
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);

	// negative error = left turn
	// positive error = right turn
	// Calculate the required roll of the plane
	// ----------------------------------------
	nav_roll = g.pidNavRoll.get_pid(bearing_error, dTnav, nav_gain_scaler);	//returns desired bank angle in degrees*100
	nav_roll = constrain(nav_roll, -g.roll_limit.get(), g.roll_limit.get());

	Vector3f omega;
	omega = dcm.get_gyro();

	// rate limiter
	long rate		= degrees(omega.z) * 100; 										// 3rad = 17188 , 6rad = 34377
	rate			= constrain(rate, -6000, 6000);									// limit input
	int dampener 	= rate * YAW_DAMPENER;											// 34377 * .175 = 6000

	// add in yaw dampener
	nav_roll		-= dampener;
	nav_roll		= constrain(nav_roll, -g.roll_limit.get(), g.roll_limit.get());
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
	static int last = 1000;
	if(g.throttle_slewrate) {		// if slew limit rate is set to zero then do not slew limit

		float temp = g.throttle_slewrate * G_Dt * 10.f;		//  * 10 to scale % to pwm range of 1000 to 2000
Serial.print("radio ");	Serial.print(g.channel_throttle.radio_out); Serial.print("   temp "); Serial.print(temp); Serial.print("   last "); Serial.println(last);
		g.channel_throttle.radio_out = constrain(g.channel_throttle.radio_out, last - (int)temp, last + (int)temp);
		last = g.channel_throttle.radio_out;
	}
}


// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
static void reset_I(void)
{
	g.pidNavRoll.reset_I();
	g.pidNavPitchAirspeed.reset_I();
	g.pidNavPitchAltitude.reset_I();
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

	if(control_mode == MANUAL){
		// do a direct pass through of radio values
		if (g.mix_mode == 0){
			g.channel_roll.radio_out 		= g.channel_roll.radio_in;
			g.channel_pitch.radio_out 		= g.channel_pitch.radio_in;
		} else {
			g.channel_roll.radio_out 		= APM_RC.InputCh(CH_ROLL);
			g.channel_pitch.radio_out 		= APM_RC.InputCh(CH_PITCH);
		}
		g.channel_throttle.radio_out 	= g.channel_throttle.radio_in;
		g.channel_rudder.radio_out 		= g.channel_rudder.radio_in;
		// FIXME To me it does not make sense to control the aileron using radio_in in manual mode
		// Doug could you please take a look at this ?
		if (g_rc_function[RC_Channel_aux::k_aileron]) {
			if (g_rc_function[RC_Channel_aux::k_aileron] != rc_array[g.flight_mode_channel-1]) {
				g_rc_function[RC_Channel_aux::k_aileron]->radio_out	= g_rc_function[RC_Channel_aux::k_aileron]->radio_in;
			}
		}
		// only use radio_in if the channel is not used as flight_mode_channel
		if (g_rc_function[RC_Channel_aux::k_flap_auto]) {
			if (g_rc_function[RC_Channel_aux::k_flap_auto] != rc_array[g.flight_mode_channel-1]) {
				g_rc_function[RC_Channel_aux::k_flap_auto]->radio_out	= g_rc_function[RC_Channel_aux::k_flap_auto]->radio_in;
			} else {
				g_rc_function[RC_Channel_aux::k_flap_auto]->radio_out	= g_rc_function[RC_Channel_aux::k_flap_auto]->radio_trim;
			}
		}
	} else {
		if (g.mix_mode == 0) {
			g.channel_roll.calc_pwm();
			g.channel_pitch.calc_pwm();
			g.channel_rudder.calc_pwm();
			if (g_rc_function[RC_Channel_aux::k_aileron]) {
				g_rc_function[RC_Channel_aux::k_aileron]->servo_out = g.channel_roll.servo_out;
				g_rc_function[RC_Channel_aux::k_aileron]->calc_pwm();
			}

		}else{
			/*Elevon mode*/
			float ch1;
			float ch2;
			ch1 = BOOL_TO_SIGN(g.reverse_elevons) * (g.channel_pitch.servo_out - g.channel_roll.servo_out);
			ch2 = g.channel_pitch.servo_out + g.channel_roll.servo_out;
			g.channel_roll.radio_out =	elevon1_trim + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0/ SERVO_MAX));
			g.channel_pitch.radio_out =	elevon2_trim + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0/ SERVO_MAX));
		}

		#if THROTTLE_OUT == 0
			g.channel_throttle.servo_out = 0;
		#else
			// convert 0 to 100% into PWM
			g.channel_throttle.servo_out = constrain(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());

			// We want to supress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.
			/* Disable throttle if following conditions are met:
				1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
				AND
				2 - Our reported altitude is within 10 meters of the home altitude.
				3 - Our reported speed is under 5 meters per second.
				4 - We are not performing a takeoff in Auto mode
				OR
				5 - Home location is not set
			*/
			if (
					(control_mode == CIRCLE || control_mode >= FLY_BY_WIRE_B) &&
					(abs(home.alt - current_loc.alt) < 1000) &&
					((g.airspeed_enabled ? airspeed : g_gps->ground_speed) < 500 ) &&
					!(control_mode==AUTO && takeoff_complete == false)
				) {
				g.channel_throttle.servo_out = 0;
				g.channel_throttle.calc_pwm();
			}

		#endif

		g.channel_throttle.calc_pwm();

		/*  TO DO - fix this for RC_Channel library
		#if THROTTLE_REVERSE == 1
			radio_out[CH_THROTTLE] = radio_max(CH_THROTTLE) + radio_min(CH_THROTTLE) - radio_out[CH_THROTTLE];
		#endif
		*/

        if (control_mode >= FLY_BY_WIRE_B) {
            /* only do throttle slew limiting in modes where throttle
               control is automatic */
            throttle_slew_limit();
        }
	}

	// Auto flap deployment
	if (g_rc_function[RC_Channel_aux::k_flap_auto] != NULL) {
		if(control_mode < FLY_BY_WIRE_B) {
			// only use radio_in if the channel is not used as flight_mode_channel
			if (g_rc_function[RC_Channel_aux::k_flap_auto] != rc_array[g.flight_mode_channel-1]) {
				g_rc_function[RC_Channel_aux::k_flap_auto]->radio_out = g_rc_function[RC_Channel_aux::k_flap_auto]->radio_in;
			} else {
				g_rc_function[RC_Channel_aux::k_flap_auto]->radio_out = g_rc_function[RC_Channel_aux::k_flap_auto]->radio_trim;
			}
		} else if (control_mode >= FLY_BY_WIRE_B) {
            // FIXME: use target_airspeed in both FBW_B and g.airspeed_enabled cases - Doug?
			if (control_mode == FLY_BY_WIRE_B) {
				flapSpeedSource = ((float)target_airspeed)/100;
			} else if (g.airspeed_enabled == true) {
				flapSpeedSource = g.airspeed_cruise/100;
			} else {
				flapSpeedSource = g.throttle_cruise;
			}
			if ( flapSpeedSource > g.flap_1_speed) {
				g_rc_function[RC_Channel_aux::k_flap_auto]->servo_out = 0;
			} else if (flapSpeedSource > g.flap_2_speed) {
				g_rc_function[RC_Channel_aux::k_flap_auto]->servo_out = g.flap_1_percent;
			} else {
				g_rc_function[RC_Channel_aux::k_flap_auto]->servo_out = g.flap_2_percent;
			}
			g_rc_function[RC_Channel_aux::k_flap_auto]->calc_pwm();
		}
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
