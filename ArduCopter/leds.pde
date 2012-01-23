static void update_lights()
{
	switch(led_mode){
		case NORMAL_LEDS:
			update_motor_light();
			update_GPS_light();
			break;

		case AUTO_TRIM_LEDS:
			dancing_light();
			break;
	}
}

static void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	switch (g_gps->status()){

		case(2):
			if(home_is_set) {                      // JLN update
			    digitalWrite(C_LED_PIN, LED_ON);  //Turn LED C on when gps has valid fix AND home is set.
			} else {
            	digitalWrite(C_LED_PIN, LED_OFF);
            }
			break;

		case(1):
			if (g_gps->valid_read == true){
				GPS_light = !GPS_light; // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
				if (GPS_light){
					digitalWrite(C_LED_PIN, LED_OFF);
				}else{
					digitalWrite(C_LED_PIN, LED_ON);
				}
				g_gps->valid_read = false;
			}
			break;

		default:
			digitalWrite(C_LED_PIN, LED_OFF);
			break;
	}
}

static void update_motor_light(void)
{
	if(motor_armed == false){
		motor_light = !motor_light;

		// blink
		if(motor_light){
			digitalWrite(A_LED_PIN, LED_ON);
		}else{
			digitalWrite(A_LED_PIN, LED_OFF);
		}
	}else{
		if(!motor_light){
			motor_light = true;
			digitalWrite(A_LED_PIN, LED_ON);
		}
	}
}

static void dancing_light()
{
	static byte step;

	if (step++ == 3)
		step = 0;

	switch(step)
	{
		case 0:
			digitalWrite(C_LED_PIN, LED_OFF);
			digitalWrite(A_LED_PIN, LED_ON);
			break;

		case 1:
			digitalWrite(A_LED_PIN, LED_OFF);
			digitalWrite(B_LED_PIN, LED_ON);
			break;

		case 2:
			digitalWrite(B_LED_PIN, LED_OFF);
			digitalWrite(C_LED_PIN, LED_ON);
			break;
	}
}
static void clear_leds()
{
	digitalWrite(A_LED_PIN, LED_OFF);
	digitalWrite(B_LED_PIN, LED_OFF);
	digitalWrite(C_LED_PIN, LED_OFF);
	motor_light = false;
	led_mode = NORMAL_LEDS;
}

#if MOTOR_LEDS == 1
static void update_motor_leds(void)
{
	if (motor_armed == true){
		if (low_batt == true){
	    	// blink rear
			static bool blink = false;

			if (blink){
				digitalWrite(RE_LED, LED_ON);
				digitalWrite(FR_LED, LED_ON);
				digitalWrite(RI_LED, LED_OFF);
				digitalWrite(LE_LED, LED_OFF);
			}else{
				digitalWrite(RE_LED, LED_OFF);
				digitalWrite(FR_LED, LED_OFF);
				digitalWrite(RI_LED, LED_ON);
				digitalWrite(LE_LED, LED_ON);
			}
			blink = !blink;
		}else{
			digitalWrite(RE_LED, LED_ON);
			digitalWrite(FR_LED, LED_ON);
			digitalWrite(RI_LED, LED_ON);
			digitalWrite(LE_LED, LED_ON);
		}
	}else {
		digitalWrite(RE_LED, LED_OFF);
		digitalWrite(FR_LED, LED_OFF);
		digitalWrite(RI_LED, LED_OFF);
		digitalWrite(LE_LED, LED_OFF);
	}
}
#endif


