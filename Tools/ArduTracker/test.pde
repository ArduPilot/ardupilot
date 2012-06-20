// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_adc(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_imu(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_gyro(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_airspeed(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_pressure(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_xbee(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_eedump(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_rawgps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_modeswitch(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_dipswitches(uint8_t argc, 		const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
	{"radio",		test_radio},
	{"battery",		test_battery},
	{"relay",		test_relay},
	{"waypoints",	test_wp},
	{"xbee",		test_xbee},
	{"eedump",		test_eedump},	
	{"modeswitch",	test_modeswitch},	
	{"dipswitches",	test_dipswitches},	

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
	{"adc", 		test_adc},
	{"gps",			test_gps},
	{"rawgps",		test_rawgps},	
	{"imu",			test_imu},
	{"gyro",		test_gyro},
	{"airspeed",	test_airspeed},
	{"airpressure",	test_pressure},
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_SENSORS
	{"adc", 		test_adc},
	{"gps",			test_gps},
	{"imu",			test_imu},
	{"gyro",		test_gyro},
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_ATTITUDE
#endif

};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Test Mode\n\n"));
	test_menu.run();
}

void print_hit_enter()
{
	Serial.printf_P(PSTR("Hit Enter to exit.\n\n"));
}

static int8_t
test_eedump(uint8_t argc, const Menu::arg *argv)
{
	int		i, j;

	// hexdump the EEPROM
	for (i = 0; i < EEPROM_MAX_ADDR; i += 16) {
		Serial.printf_P(PSTR("%04x:"), i);
		for (j = 0; j < 16; j++)
			Serial.printf_P(PSTR(" %02x"), eeprom_read_byte((const uint8_t *)(i + j)));
		Serial.println();
	}
	return(0);
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	#if THROTTLE_REVERSE == 1
		Serial.println("Throttle is reversed in config: ");
		delay(1000);
	#endif

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();                                                     
		update_servo_switches();

		servo_out[CH_ROLL]  	= reverse_roll   * int(radio_in[CH_ROLL]   - radio_trim(CH_ROLL))   * 9;
		servo_out[CH_PITCH] 	= reverse_pitch  * int(radio_in[CH_PITCH]  - radio_trim(CH_PITCH))  * 9;
		servo_out[CH_RUDDER] 	= reverse_rudder * int(radio_in[CH_RUDDER] - radio_trim(CH_RUDDER)) * 9;

		// write out the servo PWM values
		// ------------------------------
		set_servos_4();
		
		
		for(int y = 4; y < 8; y++) { 
			radio_out[y] = constrain(radio_in[y], 	radio_min(y), 	radio_max(y));	
			APM_RC.OutputCh(y, radio_out[y]); // send to Servos
		}
		Serial.printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\t"), radio_in[CH_1], radio_in[CH_2], radio_in[CH_3], radio_in[CH_4], radio_in[CH_5], radio_in[CH_6], radio_in[CH_7], radio_in[CH_8]);
		Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (servo_out[CH_ROLL]/100), (servo_out[CH_PITCH]/100), servo_out[CH_THROTTLE], (servo_out[CH_RUDDER]/100));

		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
#if BATTERY_EVENT == 1
	for (int i = 0; i < 20; i++){
		delay(20);
		read_battery();
	}
	Serial.printf_P(PSTR("Volts: 1:"));
	Serial.print(battery_voltage1, 4);
	Serial.print(" 2:");
	Serial.print(battery_voltage2, 4);
	Serial.print(" 3:");
	Serial.print(battery_voltage3, 4);
	Serial.print(" 4:");
	Serial.println(battery_voltage4, 4);
#else
	Serial.printf_P(PSTR("Not enabled\n"));
	
#endif
	return (0);
}


static int8_t
test_relay(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	
	while(1){
		Serial.println("Relay on");
		relay_on();
		delay(3000);
		if(Serial.available() > 0){
			return (0);
		}
	
		Serial.println("Relay off");
		relay_off();
		delay(3000);
		if(Serial.available() > 0){
			return (0);
		}
	}
}

void
test_wp_print(struct Location *cmd, byte index)
{
	Serial.print("command #: ");
	Serial.print(index, DEC);
	Serial.print(" id: ");
	Serial.print(cmd->id, DEC);
	Serial.print(" p1: ");
	Serial.print(cmd->p1, DEC);
	Serial.print(" p2: ");
	Serial.print(cmd->alt, DEC);
	Serial.print(" p3: ");
	Serial.print(cmd->lat, DEC);
	Serial.print(" p4: ");
	Serial.println(cmd->lng, DEC);
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);
	ground_alt = 0;
	//read_EEPROM_waypoint_info();
	
	// save the alitude above home option
	if(get(PARAM_CONFIG) & HOLD_ALT_ABOVE_HOME){
		Serial.printf_P(PSTR("Hold altitude of %ldm\n"), get(PARAM_ALT_HOLD_HOME)/100);
	}else{
		Serial.printf_P(PSTR("Hold current altitude\n"));
	}
	
	Serial.printf_P(PSTR("%d waypoints\n"), get(PARAM_WP_TOTAL));
	Serial.printf_P(PSTR("Hit radius: %d\n"), get(PARAM_WP_RADIUS));
	Serial.printf_P(PSTR("Loiter radius: %d\n\n"), get(PARAM_LOITER_RADIUS));
	
	for(byte i = 0; i <= get(PARAM_WP_TOTAL); i++){
		struct Location temp = get_wp_with_index(i);
		test_wp_print(&temp, i);
	}
	
	return (0);
}

static int8_t
test_xbee(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));
	while(1){
		delay(250);		 
		// Timeout set high enough for X-CTU RSSI Calc over XBee @ 115200
		Serial3.printf_P(PSTR("0123456789:;<=>?@ABCDEFGHIJKLMNO\n"));
		//Serial.print("X");
		// Default 32bit data from X-CTU Range Test
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	Serial.print("Control CH ");
	Serial.println(FLIGHT_MODE_CHANNEL, DEC);

	while(1){
		delay(20);
		byte switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			Serial.printf_P(PSTR("Position %d\n"),  switchPosition);
			oldSwitchPosition = switchPosition;
		}
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_dipswitches(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(100);
		update_servo_switches();
		if (mix_mode == 0) {
			Serial.print("Mix:standard ");
			Serial.print("\t reverse_roll: ");
			Serial.print(reverse_roll, DEC);
			Serial.print("\t reverse_pitch: ");
			Serial.print(reverse_pitch, DEC);
			Serial.print("\t reverse_rudder: ");
			Serial.println(reverse_rudder, DEC);
		} else {
			Serial.print("Mix:elevons ");
			Serial.print("\t reverse_elevons: ");
			Serial.print(reverse_elevons, DEC);
			Serial.print("\t reverse_ch1_elevon: ");
			Serial.print(reverse_ch1_elevon, DEC);
			Serial.print("\t reverse_ch2_elevon: ");
			Serial.println(reverse_ch2_elevon, DEC);
		}
		if(Serial.available() > 0){
			return (0);
		}
	}
}

//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

#if HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	adc.Init();
	delay(1000);
	Serial.printf_P(PSTR("ADC\n"));
	delay(1000);
	
	while(1){
		for (int i=0;i<9;i++) Serial.printf_P(PSTR("%d\t"),adc.Ch(i));
		Serial.println();
		delay(100);
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(333);

		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		update_GPS_light();
		
		gps.update();
		
		if (gps.new_data){
			Serial.print("Lat:");
			Serial.print((float)gps.latitude/10000000, 10);
			Serial.print(" Lon:");
			Serial.print((float)gps.longitude/10000000, 10);
			Serial.printf_P(PSTR("alt %ldm, #sats: %d\n"), gps.altitude/100, gps.num_sats);
		}else{
			Serial.print(".");
		}
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_imu(uint8_t argc, const Menu::arg *argv)
{
	//Serial.printf_P(PSTR("Calibrating."));

	imu.init_gyro();

	print_hit_enter();
	delay(1000);
	
	while(1){
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			deltaMiliSeconds 	= millis() - fast_loopTimer;
			G_Dt 				= (float)deltaMiliSeconds / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// IMU
			// ---
			dcm.update_DCM(G_Dt);

			#if MAGNETOMETER == 1
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					medium_loopCounter = 0;
				}
			#endif
			
			// We are using the IMU
			// ---------------------
			Serial.printf_P(PSTR("r: %d\tp: %d\t y: %d\n"), ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
	
		}
		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_gyro(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	adc.Init();
	delay(1000);
	Serial.printf_P(PSTR("Gyro | Accel\n"));
	delay(1000);
	
	while(1){
		Vector3f gyros 	= imu.get_gyro();
		Vector3f accels = imu.get_accel();
		Serial.printf_P(PSTR("%d\t%d\t%d\t|\t%d\t%d\t%d\n"), (int)gyros.x, (int)gyros.y, (int)gyros.z, (int)accels.x, (int)accels.y, (int)accels.z);
		delay(100);

		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
#if MAGNETOMETER == DISABLED
	Serial.printf_P(PSTR("Compass disabled\n"));
	return (0);
#else	
	print_hit_enter();
	
	while(1){
		float heading;

		delay(250);	
		compass.read();
		heading = compass.calculate_heading(0,0);
		Serial.printf_P(PSTR("Heading: ("));
		Serial.print(ToDeg(heading));
		Serial.printf_P(PSTR(")  XYZ: ("));
		Serial.print(compass.mag_x);
		Serial.print(comma);
		Serial.print(compass.mag_y);
		Serial.print(comma);
		Serial.print(compass.mag_z);
		Serial.println(")");
	}
#endif
}

#endif // HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

#if HIL_MODE == HIL_MODE_DISABLED

static int8_t
test_airspeed(uint8_t argc, const Menu::arg *argv)
{
#if AIRSPEED_SENSOR == DISABLED
	Serial.printf_P(PSTR("airspeed disabled\n"));
	return (0);
#else	
	print_hit_enter();
	zero_airspeed();
	
	while(1){
		delay(20);
		read_airspeed();
		Serial.printf_P(PSTR("%dm/s\n"),airspeed/100);
		
		if(Serial.available() > 0){
			return (0);
		}
	}
#endif
}

static int8_t
test_pressure(uint8_t argc, const Menu::arg *argv)
{
	uint32_t sum = 0; 
	Serial.printf_P(PSTR("Uncalibrated Abs Airpressure\n"));
	Serial.printf_P(PSTR("Note - the altitude displayed is relative to the start of this test\n"));
	print_hit_enter();
	Serial.printf_P(PSTR("\nCalibrating....\n"));
	for (int i=1;i<301;i++) {
		read_airpressure();
		if(i>200)sum += abs_press_filt;
		delay(10);
	}
	abs_press_gnd = (double)sum/100.0;

	ground_alt = 0;
	while(1){
		delay(100);
		read_airpressure();
		//Serial.printf_P(PSTR("Alt: %dm, Raw: %d\n"), press_alt / 100, abs_press_filt);   // Someone needs to fix the formatting here for long integers
Serial.print("Altitude: ");
Serial.print(press_alt/100.0,2);
Serial.print("    Raw pressure value: ");
Serial.println(abs_press_filt);
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_rawgps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	
	while(1){
		if(Serial1.available() > 0)	// Ok, let me see, the buffer is empty?
		{
			delay(60);	// wait for it all
			while(Serial1.available() > 0){
				byte incoming = Serial1.read();
				//Serial.print(incoming,DEC);
				Serial.print(incoming, BYTE); // will output Hex values
				//Serial.print(",");
			}
			Serial.println(" ");
		}
		if(Serial.available() > 0){
			return (0);
		}
	}	
}
#endif // HIL_MODE == HIL_MODE_DISABLED
