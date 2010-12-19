void init_radio()
{
	rc_1.set_angle(4500);
	rc_1.dead_zone = 50;
	rc_2.set_angle(4500);
	rc_2.dead_zone = 50;
	rc_3.set_range(0,1000);
	rc_3.dead_zone = 20;
	rc_3.radio_max += 300; // hack for better throttle control
	rc_4.set_angle(6000); 
	rc_4.dead_zone = 500;
	rc_5.set_range(0,1000);
	rc_5.set_filter(false);
	rc_6.set_range(50,200);
	rc_7.set_range(0,1000);
	rc_8.set_range(0,1000);
	
	#if ARM_AT_STARTUP == 1
		motor_armed = 1;	
	#endif

	APM_RC.OutputCh(CH_1, 	rc_3.radio_min);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_3, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_4, 	rc_3.radio_min);

	APM_RC.Init();		// APM Radio initialization

	APM_RC.OutputCh(CH_1, 	rc_3.radio_min);					// Initialization of servo outputs
	APM_RC.OutputCh(CH_2, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_3, 	rc_3.radio_min);
	APM_RC.OutputCh(CH_4, 	rc_3.radio_min);
}

void read_radio()
{	
	rc_1.set_pwm(APM_RC.InputCh(CH_1));
	rc_2.set_pwm(APM_RC.InputCh(CH_2));
	rc_3.set_pwm(APM_RC.InputCh(CH_3));
	rc_4.set_pwm(APM_RC.InputCh(CH_4));
	rc_5.set_pwm(APM_RC.InputCh(CH_5));
	rc_6.set_pwm(APM_RC.InputCh(CH_6));
	rc_7.set_pwm(APM_RC.InputCh(CH_7));
	rc_8.set_pwm(APM_RC.InputCh(CH_8));
	//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"), rc_1.control_in, rc_2.control_in, rc_3.control_in, rc_4.control_in);
}

void trim_radio()
{
	read_radio();
	rc_1.trim();	// roll
	rc_2.trim();	// pitch
	rc_4.trim();	// yaw
}


#define ARM_DELAY 10
#define DISARM_DELAY 10

void arm_motors()
{
	static byte arming_counter;

	// Arm motor output : Throttle down and full yaw right for more than 2 seconds
	if (rc_3.control_in == 0){		
		if (rc_4.control_in > 2700) {
			if (arming_counter > ARM_DELAY) {
				motor_armed = true;
			} else{
				arming_counter++;
			}
		}else if (rc_4.control_in < -2700) {
			if (arming_counter > DISARM_DELAY){
				motor_armed = false;
			}else{
				arming_counter++;
			}			
		}else{
			arming_counter = 0;
		}
	}
}
