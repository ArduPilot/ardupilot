// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio_pwm(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_flaps(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_stabilize(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_imu(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_gyro(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_omega(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_pressure(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_nav_out(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_xbee(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_eedump(uint8_t argc, 		const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of printf that reads from flash memory
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
	{"pwm",			test_radio_pwm},
	{"radio",		test_radio},
	{"flaps",		test_flaps},
	{"stabilize",	test_stabilize},
	{"gps",			test_gps},
	{"imu",			test_imu},
	{"gyro",		test_gyro},
	{"omega",		test_omega},
	{"battery",		test_battery},
	{"relay",		test_relay},
	{"waypoints",	test_wp},
	{"airpressure",	test_pressure},
	{"nav",			test_nav_out},
	{"compass",		test_mag},
	{"xbee",		test_xbee},
	{"eedump",		test_eedump},	
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
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	
	while(1){
		delay(20);

		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		Serial.printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"), rc_1.radio_in, rc_2.radio_in, rc_3.radio_in, rc_4.radio_in, rc_5.radio_in, rc_6.radio_in, rc_7.radio_in, rc_8.radio_in);

		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	while(1){
		delay(20);
		read_radio();
		output_manual_throttle();
		
		rc_1.calc_pwm();
		rc_2.calc_pwm();
		rc_3.calc_pwm();
		rc_4.calc_pwm();
		
		//Serial.printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"), (rc_1.control_in), (rc_2.control_in), (rc_3.control_in), (rc_4.control_in), rc_5.control_in, rc_6.control_in, rc_7.control_in);
		//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (rc_1.servo_out / 100), (rc_2.servo_out / 100), rc_3.servo_out, (rc_4.servo_out / 100));
		Serial.printf_P(PSTR(	"min: %d"
								"\t in: %d"
								"\t pwm_in: %d"
								"\t sout: %d"
								"\t pwm_out %d\n"),
								rc_3.radio_min, 
								rc_3.control_in,
								rc_3.radio_in,
								rc_3.servo_out,
								rc_3.pwm_out
								);

		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_stabilize(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	//imu.init_gyro();
	
	// read the radio to set trims
	// ---------------------------
	trim_radio();
	control_mode = STABILIZE;
	Serial.printf_P(PSTR("pid_stabilize_roll.kP: "));
	Serial.println(pid_stabilize_roll.kP(),3);
	Serial.printf_P(PSTR("max_stabilize_dampener:%d\n\n "), max_stabilize_dampener);
	/*
	Serial.printf_P(PSTR("pid_yaw.kP: "));
	Serial.println(pid_yaw.kP(),3);
	Serial.printf_P(PSTR("max_yaw_dampener:%d\n\n "), max_yaw_dampener);
	Serial.printf_P(PSTR("stabilize_rate_yaw "));
	Serial.print(stabilize_rate_yaw, 3);
	Serial.printf_P(PSTR("stabilze_yaw_dampener "));
	Serial.print(stabilze_yaw_dampener, 3);
	Serial.printf_P(PSTR("\n\n "));
	*/
	
	motor_armed = true;

	while(1){
		// 50 hz
		if (millis() - fast_loopTimer > 49) {
			deltaMiliSeconds 	= millis() - fast_loopTimer;
			fast_loopTimer		= millis();
			G_Dt 				= (float)deltaMiliSeconds / 1000.f;

			if(compass_enabled){
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					compass.calculate(roll, pitch);		// Calculate heading
					medium_loopCounter = 0;
				}
			}
			// for trim features
			read_trim_switch();

			// Filters radio input - adjust filters in the radio.pde file
			// ----------------------------------------------------------
			read_radio();
	
			// IMU
			// ---
			read_AHRS();
	
			// custom code/exceptions for flight modes
			// ---------------------------------------
			update_current_flight_mode();

			//Serial.println(" ");
			
			// write out the servo PWM values
			// ------------------------------
			set_servos_4();
			//Serial.printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)deltaMiliSeconds, ((int)roll_sensor/100), ((int)pitch_sensor/100), ((uint16_t)yaw_sensor/100));
			//Serial.printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)deltaMiliSeconds, ((int)roll_sensor/100), ((int)pitch_sensor/100), ((uint16_t)yaw_sensor/100));
			
			if(Serial.available() > 0){
				return (0);
			}
			
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
			read_AHRS();

			Vector3f accels 	= imu.get_accel();
			Vector3f gyros 		= imu.get_gyro();

			if(compass_enabled){
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					compass.calculate(roll, pitch);		// Calculate heading
					medium_loopCounter = 0;
				}
			}
			
			// We are using the IMU
			// ---------------------
			Serial.printf_P(PSTR("A: %d,%d,%d\tG: %d,%d,%d\t"), (int)(accels.x*100), (int)(accels.y*100), (int)(accels.z*100),(int)(gyros.x*100), (int)(gyros.y*100), (int)(gyros.z*100));

			Serial.printf_P(PSTR("r: %d\tp: %d\t y: %d\n"), ((int)roll_sensor/100), ((int)pitch_sensor/100), ((uint16_t)yaw_sensor/100));
		}
		
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
		delay(100);
		update_GPS();
		if(home.lng != 0)
			break;
	}
	
	while(1){
		delay(20);
		calc_distance_error();
		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		//update_GPS_light();
		
		GPS.update();
		
		if (GPS.new_data){
			Serial.print("Lat:");
			Serial.print((float)GPS.latitude/10000000, 10);
			Serial.print(" Lon:");
			Serial.print((float)GPS.longitude/10000000, 10);
			Serial.printf_P(PSTR(" alt %dm, spd: %d dist:%d, #sats: %d\n"), (int)GPS.altitude/100, (int)GPS.ground_speed, (int)wp_distance, (int)GPS.num_sats);
		}else{
			//Serial.print(".");
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

/*
static int8_t
test_dcm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Gyro | Accel\n"));
	delay(1000);
	
	while(1){
		Vector3f accels = dcm.get_accel();
		Serial.print("accels.z:");
		Serial.print(accels.z);
		Serial.print("omega.z:");
		Serial.print(omega.z);
		delay(100);

		if(Serial.available() > 0){
			return (0);
		}
	}
}
*/
static int8_t
test_omega(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Omega"));
	delay(1000);

	while(1){
		Vector3f omega = dcm.get_gyro();
		Serial.printf_P(PSTR("R: %d\tP: %d\tY: %d\n"), (int)(ToDeg(omega.x)), (int)(ToDeg(omega.y)), (int)(ToDeg(omega.z)));
		delay(100);

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
		Serial.println("Relay A");
		relay_A();
		delay(3000);
		if(Serial.available() > 0){
			return (0);
		}
	
		Serial.println("Relay B");
		relay_B();
		delay(3000);
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_flaps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	
	while(1){
		delay(300);
		read_radio();
		float temp  = (float)rc_6.control_in / 1000;
	
		Serial.print("flaps: ");
		Serial.println(temp, 3);
		
		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);
	read_EEPROM_waypoint_info();
	

	// save the alitude above home option
	if(alt_to_hold == -1){
		Serial.printf_P(PSTR("Hold current altitude\n"));
	}else{
		Serial.printf_P(PSTR("Hold altitude of %dm\n"), alt_to_hold/100);
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
test_pressure(uint8_t argc, const Menu::arg *argv)
{
	uint32_t sum;
	
	Serial.printf_P(PSTR("Uncalibrated Abs Airpressure\n"));
	Serial.printf_P(PSTR("Altitude is relative to the start of this test\n"));
	print_hit_enter();
	
	Serial.printf_P(PSTR("\nCalibrating....\n"));
	/*
	for (int i = 1; i < 301; i++) {
		read_barometer();
		if(i > 200)
			sum += abs_pressure;
		delay(10);
	}
	abs_pressure_ground = (float)sum / 100.0;
	*/
	
	home.alt = 0;
	wp_distance = 0;
	init_pressure_ground();
	
	while(1){
		if (millis()-fast_loopTimer > 9) {
			deltaMiliSeconds 	= millis() - fast_loopTimer;
			G_Dt 				= (float)deltaMiliSeconds / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();
			
			
			calc_altitude_error();
			calc_nav_throttle();
		}
		
		if (millis()-medium_loopTimer > 100) {
			medium_loopTimer	= millis();

			read_radio();			// read the radio first
			next_WP.alt = home.alt + rc_6.control_in; // 0 - 2000 (20 meters)		
			read_trim_switch();
			read_barometer();

			//Serial.printf_P(PSTR("Alt: %dm, Raw: %d\n"), pressure_altitude / 100, abs_pressure);   // Someone needs to fix the formatting here for long integers
			/*
			Serial.print("Altitude: ");
			Serial.print((int)current_loc.alt,DEC);
			Serial.print("\tnext_alt: ");
			Serial.print((int)next_WP.alt,DEC);
			Serial.print("\talt_err: ");
			Serial.print((int)altitude_error,DEC);
			Serial.print("\ttNom: ");
			Serial.print(throttle_cruise,DEC);
			Serial.print("\ttOut: ");
			Serial.println(rc_3.servo_out,DEC);
			*/
			//Serial.print("    Raw pressure value: ");
			//Serial.println(abs_pressure);
		}
		
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_nav_out(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Nav test\n"));
	print_hit_enter();
	
	wp_distance = 100;
	dTnav 		= 50;
	
	while(1){
		delay(50);
		bearing_error += 100;
		bearing_error = wrap_360(bearing_error);
		calc_nav_pid();
		calc_nav_pitch();
		calc_nav_roll();
		
		Serial.printf("error %ld,\troll %ld,\tpitch %ld\n", bearing_error, nav_roll, nav_pitch);
		
		if(Serial.available() > 0){
			return (0);
		}
	}
}

static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
	if(compass_enabled == false){
		Serial.printf_P(PSTR("Compass disabled\n"));
		return (0);
	}else{
		print_hit_enter();
		while(1){
			delay(250);	
			compass.read();
			compass.calculate(0,0);
			Serial.printf_P(PSTR("Heading: ("));
			Serial.print(ToDeg(compass.heading));
			Serial.printf_P(PSTR(")  XYZ: ("));
			Serial.print(compass.mag_x);
			Serial.print(comma);
			Serial.print(compass.mag_y);
			Serial.print(comma);
			Serial.print(compass.mag_z);
			Serial.println(")");
			if(Serial.available() > 0){
				return (0);
			}
		}
	}
}


void print_hit_enter()
{
	Serial.printf_P(PSTR("Hit Enter to exit.\n\n"));
}
