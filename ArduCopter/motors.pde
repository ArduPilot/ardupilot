/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// 10 = 1 second
#define ARM_DELAY 30
#define DISARM_DELAY 20
#define LEVEL_DELAY 100


// called at 10hz
static void arm_motors()
{
	static int arming_counter;

	// don't allow arming/disarming in anything but manual
	if ((g.rc_3.control_in > 0) || (control_mode >= ALT_HOLD) || (arming_counter > LEVEL_DELAY)){
		arming_counter = 0;
		return;
	}

	// full right
	if (g.rc_4.control_in > 4000) {
		if (arming_counter == LEVEL_DELAY){
			//Serial.printf("\nAL\n");
			// begin auto leveling
			auto_level_counter = 250;
			arming_counter = 0;

		}else if (arming_counter == ARM_DELAY){
			if(motor_armed == false){
				// arm the motors and configure for flight
				init_arm_motors();
			}
			// keep going up
			arming_counter++;
		} else{
			arming_counter++;
		}

	// full left
	}else if (g.rc_4.control_in < -4000) {
		if (arming_counter == LEVEL_DELAY){
			//Serial.printf("\nLEV\n");

			// begin manual leveling
			imu.init_accel(mavlink_delay, flash_leds);
			arming_counter = 0;

		}else if (arming_counter == DISARM_DELAY){
			if(motor_armed == true){
				// arm the motors and configure for flight
				init_disarm_motors();
			}
			// keep going up
			arming_counter++;
		}else{
			arming_counter++;
		}

	// Yaw is centered
	}else{
		arming_counter = 0;
	}
}


static void init_arm_motors()
{
	//Serial.printf("\nARM\n");
    #if HIL_MODE != HIL_MODE_DISABLED || defined(DESKTOP_BUILD)
	gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
	#endif

	motor_armed 	= true;

	#if PIEZO_ARMING == 1
	piezo_beep();
	piezo_beep();
	#endif

	// Remember Orientation
	// --------------------
	init_simple_bearing();

	init_z_damper();

	// Reset home position
	// -------------------
	if(home_is_set)
		init_home();

	if(did_ground_start == false){
		did_ground_start = true;
		startup_ground();
	}

	#if HIL_MODE != HIL_MODE_ATTITUDE
		// read Baro pressure at ground -
		// this resets Baro for more accuracy
		//-----------------------------------
		init_barometer();
	#endif

	// temp hack
	motor_light = true;
	digitalWrite(A_LED_PIN, LED_ON);
}


static void init_disarm_motors()
{
	//Serial.printf("\nDISARM\n");
    #if HIL_MODE != HIL_MODE_DISABLED || defined(DESKTOP_BUILD)
	gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
	#endif

	motor_armed 	= false;
	compass.save_offsets();

	#if PIEZO_ARMING == 1
	piezo_beep();
	#endif
}

/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
static void
set_servos_4()
{
	if (motor_armed == true && motor_auto_armed == true) {
		// creates the radio_out and pwm_out values
		output_motors_armed();
	} else{
		output_motors_disarmed();
	}
}
