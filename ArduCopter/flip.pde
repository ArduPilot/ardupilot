// 2010 Jose Julio
// 2011 Adapted and updated for AC2 by Jason Short
//
// Automatic Acrobatic Procedure (AAP) v1 : Roll flip
// State machine aproach:
//    Some states are fixed commands (for a fixed time)
//    Some states are fixed commands (until some IMU condition)
//    Some states include controls inside
uint8_t flip_timer;
uint8_t	flip_state;

#define AAP_THR_INC 170
#define AAP_THR_DEC 90
#define AAP_ROLL_OUT 2000

void init_flip()
{
	if(do_flip == false){
		do_flip = true;
		flip_timer = 0;
		flip_state = 0;
	}
}

void roll_flip()
{
	// Yaw
	g.rc_4.servo_out = get_stabilize_yaw(nav_yaw);

	// Pitch
	g.rc_2.servo_out = get_stabilize_pitch(0);

	// Roll State machine
	switch (flip_state){
		case 0: // Step 1 : Initialize
			flip_timer = 0;
			flip_state++;
			break;
		case 1: // Step 2 : Increase throttle to start maneuver
			if (flip_timer < 90){ 	// .5 seconds
				g.rc_1.servo_out = get_stabilize_roll(0);
				g.rc_3.servo_out = g.throttle_cruise + AAP_THR_INC;
				flip_timer++;
			}else{
				flip_state++;
				flip_timer = 0;
			}
			break;

		case 2: // Step 3 : ROLL (until we reach a certain angle [45deg])
			if (ahrs.roll_sensor < 4500){
				// Roll control
				g.rc_1.servo_out = AAP_ROLL_OUT;
				g.rc_3.servo_out = g.throttle_cruise;
			}else{
				flip_state++;
			}
			break;

		case 3: // Step 4 : CONTINUE ROLL (until we reach a certain angle [-45deg])
			if((ahrs.roll_sensor >= 4500) || (ahrs.roll_sensor < -9000)){// we are in second half of roll
				//g.rc_1.servo_out = 0;
				g.rc_1.servo_out = get_rate_roll(40000);
				g.rc_3.servo_out = g.throttle_cruise - AAP_THR_DEC;
			}else{
				flip_state++;
			}
			break;

		case 4: // Step 5 : Increase throttle to stop the descend
			if (flip_timer < 120){ // .5 seconds
				g.rc_1.servo_out = get_stabilize_roll(0);
				g.rc_3.servo_out = g.throttle_cruise + g.throttle_cruise / 2;
				flip_timer++;
			}else{
				flip_state++;
				flip_timer = 0;
			}
			break;

		case 5: // exit mode
			flip_timer = 0;
			flip_state = 0;
			do_flip = false;
			break;
	}
}
