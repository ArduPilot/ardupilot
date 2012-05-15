/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// 10 = 1 second
#define ARM_DELAY 20
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
			if(motors.armed() == false){
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
			if(motors.armed()){
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
	// Flag used to track if we have armed the motors the first time.
	// This is used to decide if we should run the ground_start routine
	// which calibrates the IMU
	static bool did_ground_start = false;

	//Serial.printf("\nARM\n");
    #if HIL_MODE != HIL_MODE_DISABLED || defined(DESKTOP_BUILD)
	gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
	#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    Serial.set_blocking_writes(false);
    if (gcs3.initialised) {
        Serial3.set_blocking_writes(false);
    }
	motors.armed(true);

	if ( bitRead(g.copter_leds_mode, 3) ){	
		piezo_beep();
		delay(50);
		piezo_beep();
	}

	// Remember Orientation
	// --------------------
	init_simple_bearing();

	init_z_damper();

	// Reset home position
	// -------------------
	if(home_is_set)
		init_home();

	// all I terms are invalid
	// -----------------------
    reset_I_all();

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

	motors.armed(false);
	compass.save_offsets();

	g.throttle_cruise.save();

	// we are not in the air
	takeoff_complete = false;

	if ( bitRead(g.copter_leds_mode, 3) ){	
		piezo_beep();
	}
}

/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
static void
set_servos_4()
{
	motors.output();
}

