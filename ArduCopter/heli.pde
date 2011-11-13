/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	HELI_FRAME

#define HELI_SERVO_AVERAGING_DIGITAL 0  // 250Hz
#define HELI_SERVO_AVERAGING_ANALOG  2  // 125Hz

static float heli_throttle_scaler = 0;

// heli_servo_averaging:
//   0 or 1 = no averaging, 250hz
//   2 = average two samples, 125hz
//   3 = averaging three samples = 83.3 hz
//   4 = averaging four samples = 62.5 hz
//   5 = averaging 5 samples = 50hz
//   digital = 0 / 250hz, analog = 2 / 83.3

static void heli_init_swash()
{
    int i;
	int tilt_max[CH_3+1];
	int total_tilt_max = 0;

	// swash servo initialisation
	g.heli_servo_1.set_range(0,1000);
	g.heli_servo_2.set_range(0,1000);
	g.heli_servo_3.set_range(0,1000);
	g.heli_servo_4.set_angle(4500);

	// pitch factors
	heli_pitchFactor[CH_1] = cos(radians(g.heli_servo1_pos - g.heli_phase_angle));
	heli_pitchFactor[CH_2] = cos(radians(g.heli_servo2_pos - g.heli_phase_angle));
	heli_pitchFactor[CH_3] = cos(radians(g.heli_servo3_pos - g.heli_phase_angle));

	// roll factors
    heli_rollFactor[CH_1] = cos(radians(g.heli_servo1_pos + 90 - g.heli_phase_angle));
	heli_rollFactor[CH_2] = cos(radians(g.heli_servo2_pos + 90 - g.heli_phase_angle));
	heli_rollFactor[CH_3] = cos(radians(g.heli_servo3_pos + 90 - g.heli_phase_angle));

	// collective min / max
	total_tilt_max = 0;
	for( i=CH_1; i<=CH_3; i++ ) {
	    tilt_max[i] = max(abs(heli_rollFactor[i]*g.heli_roll_max), abs(heli_pitchFactor[i]*g.heli_pitch_max))/100;
		total_tilt_max = max(total_tilt_max,tilt_max[i]);
	}

	// servo min/max values - or should I use set_range?
	g.heli_servo_1.radio_min = g.heli_coll_min - tilt_max[CH_1];
	g.heli_servo_1.radio_max = g.heli_coll_max + tilt_max[CH_1];
	g.heli_servo_2.radio_min = g.heli_coll_min - tilt_max[CH_2];
	g.heli_servo_2.radio_max = g.heli_coll_max + tilt_max[CH_2];
	g.heli_servo_3.radio_min = g.heli_coll_min - tilt_max[CH_3];
	g.heli_servo_3.radio_max = g.heli_coll_max + tilt_max[CH_3];

	// scaler for changing channel 3 radio input into collective range
	heli_throttle_scaler = ((float)(g.heli_coll_max - g.heli_coll_min))/1000;
	
	// reset the servo averaging
	for( i=0; i<=3; i++ )
	    heli_servo_out[i] = 0;

    // double check heli_servo_averaging is reasonable
	if( g.heli_servo_averaging < 0 || g.heli_servo_averaging < 0 > 5 ) {
	    g.heli_servo_averaging = 0;
		g.heli_servo_averaging.save();
	}
}

static void heli_move_servos_to_mid()
{
	heli_move_swash(0,0,1500,0);
}

//
// heli_move_swash - moves swash plate to attitude of parameters passed in
//                 - expected ranges:
//                       roll : -4500 ~ 4500
//                       pitch: -4500 ~ 4500
//                       collective: 1000 ~ 2000
//                       yaw:   -4500 ~ 4500
//
static void heli_move_swash(int roll_out, int pitch_out, int coll_out, int yaw_out)
{	
    int yaw_offset = 0;
	
    // regular flight mode checks
	if( g.heli_servo_manual != 1) {
	    // ensure values are acceptable:
		roll_out = constrain(roll_out, (int)-g.heli_roll_max, (int)g.heli_roll_max);
		pitch_out = constrain(pitch_out, (int)-g.heli_pitch_max, (int)g.heli_pitch_max);
		coll_out = constrain(coll_out, (int)g.heli_coll_min, (int)g.heli_coll_max);
		
		// rudder feed forward based on collective
		if( !g.heli_ext_gyro_enabled ) {
			yaw_offset = g.heli_coll_yaw_effect * (coll_out - g.heli_coll_mid);
		}
	}

	// swashplate servos
	g.heli_servo_1.servo_out = (heli_rollFactor[CH_1] * roll_out + heli_pitchFactor[CH_1] * pitch_out)/10 + coll_out - 1000 + (g.heli_servo_1.radio_trim-1500);
	g.heli_servo_2.servo_out = (heli_rollFactor[CH_2] * roll_out + heli_pitchFactor[CH_2] * pitch_out)/10 + coll_out - 1000 + (g.heli_servo_2.radio_trim-1500);
	g.heli_servo_3.servo_out = (heli_rollFactor[CH_3] * roll_out + heli_pitchFactor[CH_3] * pitch_out)/10 + coll_out - 1000 + (g.heli_servo_3.radio_trim-1500);
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
	ICR5 = 5000;	// 400 hz output 	CH 1, 2, 9
	ICR1 = 5000;	// 400 hz output	CH 3, 4, 10
	ICR3 = 40000;	// 50 hz output		CH 7, 8, 11
	#endif
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

	heli_move_swash( g.rc_1.servo_out, g.rc_2.servo_out, g.rc_3.radio_out, g.rc_4.servo_out );
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

// heli_get_scaled_throttle - user's throttle scaled to collective range
// input is expected to be in the range of 0~1000 (ie. pwm)
// also does equivalent of angle_boost
static int heli_get_scaled_throttle(int throttle)
{
    float scaled_throttle = (g.heli_coll_min - 1000) + throttle * heli_throttle_scaler;
	return scaled_throttle;
}

// heli_angle_boost - takes servo_out position
// adds a boost depending on roll/pitch values
// equivalent of quad's angle_boost function
// pwm_out value should be 0 ~ 1000
static int heli_get_angle_boost(int pwm_out)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
	angle_boost_factor = 1.0 - constrain(angle_boost_factor, .5, 1.0);
    int throttle_above_center = max(1000 + pwm_out - g.heli_coll_mid,0);
	return pwm_out + throttle_above_center*angle_boost_factor;
}

#endif // HELI_FRAME
