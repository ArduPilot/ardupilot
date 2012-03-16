/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	HELI_FRAME

#define HELI_SERVO_AVERAGING_DIGITAL 0  // 250Hz
#define HELI_SERVO_AVERAGING_ANALOG  2  // 125Hz

static bool heli_swash_initialised = false;
static int heli_throttle_mid = 0;  // throttle mid point in pwm form (i.e. 0 ~ 1000)
static float heli_collective_scalar = 1;  // throttle scalar to convert pwm form (i.e. 0 ~ 1000) passed in to actual servo range (i.e 1250~1750 would be 500)

// heli_servo_averaging:
//   0 or 1 = no averaging, 250hz
//   2 = average two samples, 125hz
//   3 = averaging three samples = 83.3 hz
//   4 = averaging four samples = 62.5 hz
//   5 = averaging 5 samples = 50hz
//   digital = 0 / 250hz, analog = 2 / 83.3

// reset swash for maximum movements - used for set-up
static void heli_reset_swash()
{
	// free up servo ranges
	g.heli_servo_1.radio_min = 1000;
	g.heli_servo_1.radio_max = 2000;
	g.heli_servo_2.radio_min = 1000;
	g.heli_servo_2.radio_max = 2000;
	g.heli_servo_3.radio_min = 1000;
	g.heli_servo_3.radio_max = 2000;

	if (!g.heli_h1_swash_enabled){			//CCPM Swashplate, perform servo control mixing
		
		// roll factors
		heli_rollFactor[CH_1] = cos(radians(g.heli_servo1_pos + 90 - g.heli_phase_angle));
		heli_rollFactor[CH_2] = cos(radians(g.heli_servo2_pos + 90 - g.heli_phase_angle));
		heli_rollFactor[CH_3] = cos(radians(g.heli_servo3_pos + 90 - g.heli_phase_angle));
				
		// pitch factors
		heli_pitchFactor[CH_1] = cos(radians(g.heli_servo1_pos - g.heli_phase_angle));
		heli_pitchFactor[CH_2] = cos(radians(g.heli_servo2_pos - g.heli_phase_angle));
		heli_pitchFactor[CH_3] = cos(radians(g.heli_servo3_pos - g.heli_phase_angle));
		
		// collective factors
		heli_collectiveFactor[CH_1] = 1;
		heli_collectiveFactor[CH_2] = 1;
		heli_collectiveFactor[CH_3] = 1;
		
	}else{  								//H1 Swashplate, keep servo outputs seperated

		// roll factors
		heli_rollFactor[CH_1] = 1;
		heli_rollFactor[CH_2] = 0;
		heli_rollFactor[CH_3] = 0;
	
		// pitch factors
		heli_pitchFactor[CH_1] = 0;
		heli_pitchFactor[CH_2] = 1;
		heli_pitchFactor[CH_3] = 0;
		
		// collective factors
		heli_collectiveFactor[CH_1] = 0;
		heli_collectiveFactor[CH_2] = 0;
		heli_collectiveFactor[CH_3] = 1;
	}

	// set throttle scaling
	heli_collective_scalar = ((float)(g.rc_3.radio_max - g.rc_3.radio_min))/1000.0;

	// we must be in set-up mode so mark swash as uninitialised
	heli_swash_initialised = false;
}

// initialise the swash
static void heli_init_swash()
{
    int i;

	// swash servo initialisation
	g.heli_servo_1.set_range(0,1000);
	g.heli_servo_2.set_range(0,1000);
	g.heli_servo_3.set_range(0,1000);
	g.heli_servo_4.set_angle(4500);

	// ensure g.heli_coll values are reasonable
	if( g.heli_collective_min >= g.heli_collective_max ) {
	    g.heli_collective_min = 1000;
		g.heli_collective_max = 2000;
	}
	g.heli_collective_mid = constrain(g.heli_collective_mid, g.heli_collective_min, g.heli_collective_max);

	// calculate throttle mid point
	heli_throttle_mid = ((float)(g.heli_collective_mid-g.heli_collective_min))/((float)(g.heli_collective_max-g.heli_collective_min))*1000.0;

	// determine scalar throttle input
	heli_collective_scalar = ((float)(g.heli_collective_max-g.heli_collective_min))/1000.0;

	if (!g.heli_h1_swash_enabled){			//CCPM Swashplate, perform control mixing
		
		// roll factors
		heli_rollFactor[CH_1] = cos(radians(g.heli_servo1_pos + 90 - g.heli_phase_angle));
		heli_rollFactor[CH_2] = cos(radians(g.heli_servo2_pos + 90 - g.heli_phase_angle));
		heli_rollFactor[CH_3] = cos(radians(g.heli_servo3_pos + 90 - g.heli_phase_angle));
				
		// pitch factors
		heli_pitchFactor[CH_1] = cos(radians(g.heli_servo1_pos - g.heli_phase_angle));
		heli_pitchFactor[CH_2] = cos(radians(g.heli_servo2_pos - g.heli_phase_angle));
		heli_pitchFactor[CH_3] = cos(radians(g.heli_servo3_pos - g.heli_phase_angle));
		
		// collective factors
		heli_collectiveFactor[CH_1] = 1;
		heli_collectiveFactor[CH_2] = 1;
		heli_collectiveFactor[CH_3] = 1;
		
	}else{  								//H1 Swashplate, keep servo outputs seperated

		// roll factors
		heli_rollFactor[CH_1] = 1;
		heli_rollFactor[CH_2] = 0;
		heli_rollFactor[CH_3] = 0;
	
		// pitch factors
		heli_pitchFactor[CH_1] = 0;
		heli_pitchFactor[CH_2] = 1;
		heli_pitchFactor[CH_3] = 0;
		
		// collective factors
		heli_collectiveFactor[CH_1] = 0;
		heli_collectiveFactor[CH_2] = 0;
		heli_collectiveFactor[CH_3] = 1;
	}

	// servo min/max values
	g.heli_servo_1.radio_min = 1000;
	g.heli_servo_1.radio_max = 2000;
	g.heli_servo_2.radio_min = 1000;
	g.heli_servo_2.radio_max = 2000;
	g.heli_servo_3.radio_min = 1000;
	g.heli_servo_3.radio_max = 2000;

	// reset the servo averaging
	for( i=0; i<=3; i++ )
	    heli_servo_out[i] = 0;

    // double check heli_servo_averaging is reasonable
	if( g.heli_servo_averaging < 0 || g.heli_servo_averaging > 5 ) {
	    g.heli_servo_averaging = 0;
		g.heli_servo_averaging.save();
	}

	// mark swash as initialised
	heli_swash_initialised = true;
}

static void heli_move_servos_to_mid()
{
    // call multiple times to force through the servo averaging
    for( int i=0; i<5; i++ ) {
		heli_move_swash(0,0,500,0);
		delay(20);
	}
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 0 ~ 1000
//                       yaw:   -4500 ~ 4500
//
static void heli_move_swash(int roll_out, int pitch_out, int coll_out, int yaw_out)
{
	int yaw_offset = 0;
	int coll_out_scaled;

	if( g.heli_servo_manual == 1 ) {  // are we in manual servo mode? (i.e. swash set-up mode)?
		// check if we need to freeup the swash
		if( heli_swash_initialised ) {
			heli_reset_swash();
		}
		coll_out_scaled = coll_out * heli_collective_scalar + g.rc_3.radio_min - 1000;
	}else{  // regular flight mode

		// check if we need to reinitialise the swash
		if( !heli_swash_initialised ) {
			heli_init_swash();
		}

	    // ensure values are acceptable:
		roll_out = constrain(roll_out, (int)-g.heli_roll_max, (int)g.heli_roll_max);
		pitch_out = constrain(pitch_out, (int)-g.heli_pitch_max, (int)g.heli_pitch_max);
		coll_out = constrain(coll_out, 0, 1000);
		coll_out_scaled = coll_out * heli_collective_scalar + g.heli_collective_min - 1000;
		
		// rescale roll_out and pitch-out into the min and max ranges to provide linear motion 
		// across the input range instead of stopping when the input hits the constrain value
		// these calculations are based on an assumption of the user specified roll_max and pitch_max 
		// coming into this equation at 4500 or less, and based on the original assumption of the  
		// total g.heli_servo_x.servo_out range being -4500 to 4500.
		roll_out = (-g.heli_roll_max + (float)( 2 * g.heli_roll_max * (roll_out + 4500.0)/9000.0));
		pitch_out = (-g.heli_pitch_max + (float)(2 * g.heli_pitch_max * (pitch_out + 4500.0)/9000.0));
		

		// rudder feed forward based on collective
		#if HIL_MODE == HIL_MODE_DISABLED  // don't do rudder feed forward in simulator
		if( !g.heli_ext_gyro_enabled ) {
			yaw_offset = g.heli_collective_yaw_effect * abs(coll_out_scaled - heli_throttle_mid);
		}
		#endif
	}

	// swashplate servos
	g.heli_servo_1.servo_out = (heli_rollFactor[CH_1] * roll_out + heli_pitchFactor[CH_1] * pitch_out)/10 + heli_collectiveFactor[CH_1] * coll_out_scaled + (g.heli_servo_1.radio_trim-1500) + g.heli_h1_swash_enabled * 500;
	g.heli_servo_2.servo_out = (heli_rollFactor[CH_2] * roll_out + heli_pitchFactor[CH_2] * pitch_out)/10 + heli_collectiveFactor[CH_2] * coll_out_scaled + (g.heli_servo_2.radio_trim-1500) + g.heli_h1_swash_enabled * 500;
	g.heli_servo_3.servo_out = (heli_rollFactor[CH_3] * roll_out + heli_pitchFactor[CH_3] * pitch_out)/10 + heli_collectiveFactor[CH_3] * coll_out_scaled + (g.heli_servo_3.radio_trim-1500);
	g.heli_servo_4.servo_out = yaw_out + yaw_offset;

	// use servo_out to calculate pwm_out and radio_out
	g.heli_servo_1.calc_pwm();
	g.heli_servo_2.calc_pwm();
	g.heli_servo_3.calc_pwm();
	g.heli_servo_4.calc_pwm();

	// add the servo values to the averaging
	heli_servo_out[0] += g.heli_servo_1.radio_out;
	heli_servo_out[1] += g.heli_servo_2.radio_out;
	heli_servo_out[2] += g.heli_servo_3.radio_out;
	heli_servo_out[3] += g.heli_servo_4.radio_out;
	heli_servo_out_count++;

	// is it time to move the servos?
	if( heli_servo_out_count >= g.heli_servo_averaging ) {

	    // average the values if necessary
	    if( g.heli_servo_averaging >= 2 ) {
		    heli_servo_out[0] /= g.heli_servo_averaging;
			heli_servo_out[1] /= g.heli_servo_averaging;
			heli_servo_out[2] /= g.heli_servo_averaging;
			heli_servo_out[3] /= g.heli_servo_averaging;
		}

		// actually move the servos
		APM_RC.OutputCh(CH_1, heli_servo_out[0]);
		APM_RC.OutputCh(CH_2, heli_servo_out[1]);
		APM_RC.OutputCh(CH_3, heli_servo_out[2]);
		APM_RC.OutputCh(CH_4, heli_servo_out[3]);

		// output gyro value
		if( g.heli_ext_gyro_enabled ) {
			APM_RC.OutputCh(CH_7, g.heli_ext_gyro_gain);
		}

		#if INSTANT_PWM == 1
		// InstantPWM
		APM_RC.Force_Out0_Out1();
		APM_RC.Force_Out2_Out3();
		#endif

		// reset the averaging
		heli_servo_out_count = 0;
		heli_servo_out[0] = 0;
		heli_servo_out[1] = 0;
		heli_servo_out[2] = 0;
		heli_servo_out[3] = 0;
	}
}

static void init_motors_out()
{
	#if INSTANT_PWM == 0
    APM_RC.SetFastOutputChannels( _BV(CH_1) | _BV(CH_2) | _BV(CH_3) | _BV(CH_4), g.rc_speed );
	#endif
}

static void motors_output_enable()
{
  APM_RC.enable_out(CH_1);
  APM_RC.enable_out(CH_2);
  APM_RC.enable_out(CH_3);
  APM_RC.enable_out(CH_4);
  APM_RC.enable_out(CH_5);
  APM_RC.enable_out(CH_6);
  APM_RC.enable_out(CH_7);
  APM_RC.enable_out(CH_8);
}

// these are not really motors, they're servos but we don't rename the function because it fits with the rest of the code better
static void output_motors_armed()
{
    // if manual override (i.e. when setting up swash), pass pilot commands straight through to swash
    if( g.heli_servo_manual == 1 ) {
		g.rc_1.servo_out = g.rc_1.control_in;
		g.rc_2.servo_out = g.rc_2.control_in;
		g.rc_3.servo_out = g.rc_3.control_in;
		g.rc_4.servo_out = g.rc_4.control_in;
	}

    //static int counter = 0;
	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	heli_move_swash( g.rc_1.servo_out, g.rc_2.servo_out, g.rc_3.servo_out, g.rc_4.servo_out );
}

// for helis - armed or disarmed we allow servos to move
static void output_motors_disarmed()
{
	if(g.rc_3.control_in > 0){
		// we have pushed up the throttle, remove safety
		motor_auto_armed = true;
	}

	output_motors_armed();
}

static void output_motor_test()
{
}

// heli_angle_boost - adds a boost depending on roll/pitch values
// equivalent of quad's angle_boost function
// throttle value should be 0 ~ 1000
static int16_t heli_get_angle_boost(int throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
	angle_boost_factor = 1.0 - constrain(angle_boost_factor, .5, 1.0);
	int throttle_above_mid = max(throttle - heli_throttle_mid,0);
	return throttle + throttle_above_mid*angle_boost_factor;

}

#endif // HELI_FRAME
