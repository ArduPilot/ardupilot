// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
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
static int8_t	test_jeti(uint8_t argc, 		const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
/*static int8_t	help_test(uint8_t argc, 			const Menu::arg *argv)
{
	Serial.printf_P(PSTR("\n"
						 "Commands:\n"
						 "  radio\n"
						 "  servos\n"
						 "  gps\n"
						 "  imu\n"
						 "  battery\n"
						 "\n"));
}*/

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
	{"radio",		test_radio},
	{"gps",			test_gps},
	{"imu",			test_imu},
	{"gyro",		test_gyro},
	{"battery",		test_battery},
	{"relay",		test_relay},
	{"waypoints",	test_wp},
	{"airspeed",	test_airspeed},
	{"airpressure",	test_pressure},
	{"compass",		test_mag},
	{"xbee",		test_xbee},
	{"eedump",		test_eedump},	
	{"jeti",		test_jeti},
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Test Mode\n\n"));
	test_menu.run();
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

		servo_out[CH_ROLL]  	= reverse_roll   * (radio_in[CH_ROLL]   - radio_trim[CH_ROLL])   * 9;
		servo_out[CH_PITCH] 	= reverse_pitch  * (radio_in[CH_PITCH]  - radio_trim[CH_PITCH])  * 9;
		servo_out[CH_RUDDER] 	= reverse_rudder * (radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 9;

		// write out the servo PWM values
		// ------------------------------
		set_servos_4();
		
		
		for(int y = 4; y < 8; y++) { 
			radio_out[y] = constrain(radio_in[y], 	radio_min[y], 	radio_max[y]);	
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
test_gps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(333);

		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		update_GPS_light();
		
		GPS.update();
		
		if (GPS.new_data){
			Serial.print("Lat:");
			Serial.print((float)GPS.latitude/10000000, 10);
			Serial.print(" Lon:");
			Serial.print((float)GPS.longitude/10000000, 10);
			Serial.printf_P(PSTR("alt %dm, #sats: %d\n"), GPS.altitude/100, GPS.num_sats);
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
	Serial.printf_P(PSTR("Calibrating."));
	startup_IMU_ground();
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);
		read_AHRS();
			
		// We are using the IMU
		// ---------------------
		Serial.printf_P(PSTR("r: %d\tp: %d\t y: %d\n"), ((int)roll_sensor/100), ((int)pitch_sensor/100), ((uint16_t)yaw_sensor/100));

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
	//DDRL |= B00000100;
	
	while(1){
		Serial.println(int DDRL);
		Serial.println(int PORTL);
		Serial.println("Relay A");
		relay_A();
		delay(500);
		if(Serial.available() > 0){
			return (0);
		}
	
		Serial.println("Relay B");
		relay_B();
		delay(500);
		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_gyro(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Gyro | Accel\n"));
	delay(200);

	while(1){
		for (int i = 0; i < 6; i++) {
			AN[i] = APM_ADC.Ch(sensors[i]);
		}
		Serial.printf_P(PSTR("%d\t%d\t%d\t|\t%d\t%d\t%d\n"), (int)AN[0], (int)AN[1], (int)AN[2], (int)AN[3], (int)AN[4], (int)AN[5]);
		delay(100);

		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);
	ground_alt = 0;
	read_EEPROM_waypoint_info();
	
	uint8_t options 	= eeprom_read_byte((const uint8_t *) EE_CONFIG);
	int32_t hold 		= eeprom_read_dword((const uint32_t *) EE_ALT_HOLD_HOME);

	// save the alitude above home option
	if(options & HOLD_ALT_ABOVE_HOME){
		Serial.printf_P(PSTR("Hold altitude of %dm\n"), hold/100);
	}else{
		Serial.printf_P(PSTR("Hold current altitude\n"));
	}
	
	Serial.printf_P(PSTR("%d waypoints\n"), wp_total);
	Serial.printf_P(PSTR("Hit radius: %d\n"), wp_radius);
	Serial.printf_P(PSTR("Loiter radius: %d\n\n"), loiter_radius);
	
	for(byte i = 0; i <= wp_total; i++){
		struct Location temp = get_wp_with_index(i);
		print_waypoint(&temp, i);
	}
	
	return (0);
}


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
test_xbee(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));
	while(1){
		delay(250);		 
		// Timeout set high enough for X-CTU RSSI Calc over XBee @ 115200
		//Serial3.printf_P(PSTR("0123456789:;<=>?@ABCDEFGHIJKLMNO\n"));
		//Serial.print("X");
		// Default 32bit data from X-CTU Range Test
		if(Serial.available() > 0){
			return (0);
		}
	}
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
test_mag(uint8_t argc, const Menu::arg *argv)
{
#if MAGNETOMETER == DISABLED
	Serial.printf_P(PSTR("Compass disabled\n"));
	return (0);
#else	
	print_hit_enter();
	
	while(1){
		delay(250);	
		APM_Compass.Read();
		APM_Compass.Calculate(0,0);
		Serial.printf_P(PSTR("Heading: ("));
		Serial.print(ToDeg(APM_Compass.Heading));
		Serial.printf_P(PSTR(")  XYZ: ("));
		Serial.print(APM_Compass.Mag_X);
		Serial.print(comma);
		Serial.print(APM_Compass.Mag_Y);
		Serial.print(comma);
		Serial.print(APM_Compass.Mag_Z);
		Serial.println(")");
		if(Serial.available() > 0){
			return (0);
		}
	}
#endif
}

static int8_t
test_jeti(uint8_t argc, const Menu::arg *argv)
{
	uint8_t switchPosition, m;
	print_hit_enter();

	JB.begin();
	//JB.print("     X-DIY");
	//JB.print("   Test mode");
	//JB.checkvalue(20000);
	Serial.print("Jeti Box test: press any buttons\n\n");
	while(1){
		delay(200);
		//Serial.println(JB.checkvalue(0));
		read_radio();		
		switchPosition = readSwitch();
		m = flight_modes[switchPosition];
		JB.clear(1);
		JB.print("Mode: ");
		JB.print(flight_mode_strings[m]);
		JB.clear(2);
		switch (JB.readbuttons())
		{
		case JB_key_right:
			JB.print("rechts");
			Serial.print("rechts\n");
			break;

		case JB_key_left:
			JB.print("links");
			Serial.print("links\n");
			break;

		case JB_key_up:
			JB.print("hoch");
			Serial.print("hoch\n");
			break;

		case JB_key_down:
			JB.print("runter");
			Serial.print("runter\n");
			break;

		case JB_key_left | JB_key_right:
			JB.print("links-rechts");
			Serial.print("links-rechts\n");
			break;
		}

		if(Serial.available() > 0){
			return (0);
		}
	}
}

void print_hit_enter()
{
	Serial.printf_P(PSTR("Hit Enter to exit.\n\n"));
}
