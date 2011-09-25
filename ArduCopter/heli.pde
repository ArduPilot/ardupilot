/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if FRAME_CONFIG ==	HELI_FRAME

#define HELI_SERVO_AVERAGING_DIGITAL 0  // 250Hz
#define HELI_SERVO_AVERAGING_ANALOG  2  // 125Hz

static int heli_manual_override = false;

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
	heli_pitchFactor[CH_1] = cos(radians(g.heli_servo1_pos));
	heli_pitchFactor[CH_2] = cos(radians(g.heli_servo2_pos));
	heli_pitchFactor[CH_3] = cos(radians(g.heli_servo3_pos));
	
	// roll factors
    heli_rollFactor[CH_1] = cos(radians(g.heli_servo1_pos + 90));
	heli_rollFactor[CH_2] = cos(radians(g.heli_servo2_pos + 90));
	heli_rollFactor[CH_3] = cos(radians(g.heli_servo3_pos + 90));
	
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
    // ensure values are acceptable:
	roll_out = constrain(roll_out, (int)-g.heli_roll_max, (int)g.heli_roll_max);
	pitch_out = constrain(pitch_out, (int)-g.heli_pitch_max, (int)g.heli_pitch_max);
	coll_out = constrain(coll_out, (int)g.heli_coll_min, (int)g.heli_coll_max);

	// swashplate servos
	g.heli_servo_1.servo_out = (heli_rollFactor[CH_1] * roll_out + heli_pitchFactor[CH_1] * pitch_out)/10 + coll_out + (g.heli_servo_1.radio_trim-1500);
	if( g.heli_servo_1.get_reverse() )
	    g.heli_servo_1.servo_out = 3000 - g.heli_servo_1.servo_out;
		
	g.heli_servo_2.servo_out = (heli_rollFactor[CH_2] * roll_out + heli_pitchFactor[CH_2] * pitch_out)/10 + coll_out + (g.heli_servo_2.radio_trim-1500);
	if( g.heli_servo_2.get_reverse() )
		g.heli_servo_2.servo_out = 3000 - g.heli_servo_2.servo_out;
		
	g.heli_servo_3.servo_out = (heli_rollFactor[CH_3] * roll_out + heli_pitchFactor[CH_3] * pitch_out)/10 + coll_out + (g.heli_servo_3.radio_trim-1500);
	if( g.heli_servo_3.get_reverse() )
	    g.heli_servo_3.servo_out = 3000 - g.heli_servo_3.servo_out;
		
	if( g.heli_servo_4.get_reverse() )
		g.heli_servo_4.servo_out = -yaw_out;  // should probably just use rc_4 directly like we do for a tricopter
	else
		g.heli_servo_4.servo_out = yaw_out;
	
	// use servo_out to calculate pwm_out and radio_out
	g.heli_servo_1.calc_pwm();
	g.heli_servo_2.calc_pwm();
	g.heli_servo_3.calc_pwm();
	g.heli_servo_4.calc_pwm();	
	
	// add the servo values to the averaging
	heli_servo_out[0] += g.heli_servo_1.servo_out;
	heli_servo_out[1] += g.heli_servo_2.servo_out;
	heli_servo_out[2] += g.heli_servo_3.servo_out;
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

		// InstantPWM - force message to the servos
		APM_RC.Force_Out0_Out1();
		APM_RC.Force_Out2_Out3();
		
		// reset the averaging
		heli_servo_out_count = 0;
		heli_servo_out[0] = 0;
		heli_servo_out[1] = 0;
		heli_servo_out[2] = 0;
		heli_servo_out[3] = 0;
	}
}

// these are not really motors, they're servos but we don't rename the function because it fits with the rest of the code better
static void output_motors_armed()
{
    //static int counter = 0;
	g.rc_1.calc_pwm();
	g.rc_2.calc_pwm();
	g.rc_3.calc_pwm();
	g.rc_4.calc_pwm();

	if( heli_manual_override ) {
	    // straight pass through from radio inputs to swash plate
	    heli_move_swash( g.rc_1.control_in, g.rc_2.control_in, g.rc_3.radio_in, g.rc_4.control_in );
	}else{
	    // source inputs from attitude controller
		heli_move_swash( g.rc_1.servo_out, g.rc_2.servo_out, g.rc_3.radio_out, g.rc_4.servo_out );
	}
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

#endif // HELI_FRAME
