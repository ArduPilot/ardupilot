// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio_pwm(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_failsafe(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_stabilize(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_fbw(uint8_t argc,			const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_adc(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_imu(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_dcm(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_omega(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_current(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_pressure(uint8_t argc, 	const Menu::arg *argv);
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
	{"failsafe",	test_failsafe},
	{"stabilize",	test_stabilize},
	{"fbw",			test_fbw},
	{"gps",			test_gps},
	{"adc", 		test_adc},
	{"imu",			test_imu},
	{"dcm",			test_dcm},
	{"omega",		test_omega},
	{"battery",		test_battery},
	{"current",		test_current},
	{"relay",		test_relay},
	{"waypoints",	test_wp},
	{"airpressure",	test_pressure},
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
		
		Serial.printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"), (rc_1.control_in), (rc_2.control_in), (rc_3.control_in), (rc_4.control_in), rc_5.control_in, rc_6.control_in, rc_7.control_in);
		//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (rc_1.servo_out / 100), (rc_2.servo_out / 100), rc_3.servo_out, (rc_4.servo_out / 100));
		
		/*Serial.printf_P(PSTR(	"min: %d"
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
		*/
		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_failsafe(uint8_t argc, const Menu::arg *argv)
{
	byte fail_test;
	print_hit_enter();
	for(int i = 0; i < 50; i++){
		delay(20);
		read_radio();
	}
	
	// read the radio to set trims
	// ---------------------------
	trim_radio();
	
	oldSwitchPosition = readSwitch();
	
	Serial.printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
	
	while(1){
		delay(20);

		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
			
		if(rc_3.control_in > 0){
			Serial.printf_P(PSTR("THROTTLE ERROR %d \n"), rc_3.control_in);
			fail_test++;
		}
		
		if(oldSwitchPosition != readSwitch()){
			Serial.printf_P(PSTR("MODE CHANGE: "));
			Serial.println(flight_mode_strings[readSwitch()]);
			fail_test++;
		}
		if(throttle_failsafe_enabled && rc_3.get_failsafe()){
			Serial.printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), rc_3.radio_in);
			Serial.println(flight_mode_strings[readSwitch()]);
			fail_test++;
		}
		
		if(fail_test > 0){
			return (0);
		}
		if(Serial.available() > 0){
			Serial.printf_P(PSTR("LOS caused no change in ACM.\n"));
			return (0);
		}
	}
}

static int8_t
test_stabilize(uint8_t argc, const Menu::arg *argv)
{
	static byte ts_num;
	
	
	print_hit_enter();
	delay(1000);
	
	// setup the radio
	// ---------------
	init_rc_in();
	
	control_mode = STABILIZE;
	Serial.printf_P(PSTR("pid_stabilize_roll.kP: %4.4f\n"), pid_stabilize_roll.kP());
	Serial.printf_P(PSTR("max_stabilize_dampener:%d\n\n "), max_stabilize_dampener);

	trim_radio();

	motor_auto_safe 	= false;
	motor_armed 		= true;
	
	while(1){
		// 50 hz
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			fast_loopTimer		= millis();
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;

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
			
			// allow us to zero out sensors with control switches
			if(rc_5.control_in < 600){
				dcm.roll_sensor = dcm.pitch_sensor = 0;
			}
			
			// custom code/exceptions for flight modes
			// ---------------------------------------
			update_current_flight_mode();
			
			// write out the servo PWM values
			// ------------------------------
			set_servos_4();
			
			ts_num++;
			if (ts_num > 10){
				ts_num = 0;
				Serial.printf_P(PSTR("r: %d, p:%d, rc1:%d, Int%4.4f, "),
					(int)(dcm.roll_sensor/100),
					(int)(dcm.pitch_sensor/100),
					rc_1.pwm_out,
					pid_stabilize_roll.get_integrator());

				print_motor_out();
			}
			// R: 1417,  L: 1453  F: 1453  B: 1417
			
			//Serial.printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
			//Serial.printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
			
			if(Serial.available() > 0){
				return (0);
			}
			
		}
	}
}

static int8_t
test_fbw(uint8_t argc, const Menu::arg *argv)
{
	static byte ts_num;
	
	print_hit_enter();
	delay(1000);
	
	// setup the radio
	// ---------------
	init_rc_in();
	
	control_mode = FBW;
	//Serial.printf_P(PSTR("pid_stabilize_roll.kP: %4.4f\n"), pid_stabilize_roll.kP());
	//Serial.printf_P(PSTR("max_stabilize_dampener:%d\n\n "), max_stabilize_dampener);

	motor_armed = true;
	trim_radio();
	
	nav_yaw = 8000;
	scaleLongDown = 1;
	
	while(1){
		// 50 hz
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			fast_loopTimer		= millis();
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;


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
			
			// allow us to zero out sensors with control switches
			if(rc_5.control_in < 600){
				dcm.roll_sensor = dcm.pitch_sensor = 0;
			}
			
			// custom code/exceptions for flight modes
			// ---------------------------------------
			//update_current_flight_mode();
			
			// write out the servo PWM values
			// ------------------------------
			//set_servos_4();
			
			ts_num++;
			if (ts_num > 10){
				dTnav 				= 200;

				//next_WP.lat = random(-3000, 3000);
				//next_WP.lng = random(-3000, 3000);
				next_WP.lat = 3000;
				next_WP.lng = 3000;
				
				GPS.longitude = 0;
				GPS.latitude = 0;
				calc_nav();
				
				ts_num = 0;
				Serial.printf_P(PSTR(" ys:%ld, ny:%ld, ye:%ld, n_lat %ld, n_lon %ld -- n_pit %ld, n_rll %ld\n"),
					dcm.yaw_sensor,
					nav_yaw,
					yaw_error,
					nav_lat,
					nav_lon,
					nav_pitch,
					nav_roll);
			}
			//r: 0, p:0 -- ny:8000, ys:2172, ye:0, n_lat 0, n_lon 0 -- n_pit 0, n_rll 0

			// R: 1417,  L: 1453  F: 1453  B: 1417
			
			//Serial.printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
			//Serial.printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
			
			if(Serial.available() > 0){
				return (0);
			}
			
		}
	}
}
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	adc.Init();
	delay(1000);
	Serial.printf_P(PSTR("ADC\n"));
	delay(1000);
	
	while(1){
		for(int i = 0; i < 9; i++){
			Serial.printf_P(PSTR("i:%d\t"),adc.Ch(i));
		}
		Serial.println();
		delay(20);
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
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
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
			Serial.printf_P(PSTR("A: %4.4f, %4.4f, %4.4f\t"
								 "G: %4.4f, %4.4f, %4.4f\t"),
								accels.x, accels.y, accels.z,
								gyros.x,  gyros.y,  gyros.z);

			Serial.printf_P(PSTR("r: %ld\tp: %ld\t y: %ld\n"),
								dcm.roll_sensor,
								dcm.pitch_sensor,
								dcm.yaw_sensor);
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
		
		if(Serial.available() > 0){
			return (0);
		}
		
		if(home.lng != 0){
			break;
		}
	}
	
	while(1){
		delay(100);
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
test_dcm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Gyro | Accel\n"));
	Vector3f 	_cam_vector;
	Vector3f 	_out_vector;
	
	G_Dt = .02;

	while(1){
		for(byte i = 0; i <= 50; i++){
			delay(20);
			// IMU
			// ---
			read_AHRS();
		}
		
		Matrix3f temp = dcm.get_dcm_matrix();
		Matrix3f temp_t = dcm.get_dcm_transposed();
		
		Serial.printf_P(PSTR("dcm\n"
							 "%4.4f \t %4.4f \t %4.4f \n"
							 "%4.4f \t %4.4f \t %4.4f \n"
							 "%4.4f \t %4.4f \t %4.4f \n\n"),
							temp.a.x, temp.a.y, temp.a.z,
							temp.b.x, temp.b.y, temp.b.z,
							temp.c.x, temp.c.y, temp.c.z);

		int _pitch 		= degrees(-asin(temp.c.x));
		int _roll 		= degrees(atan2(temp.c.y, temp.c.z));
		int _yaw 		= degrees(atan2(temp.b.x, temp.a.x));
		Serial.printf_P(PSTR(	"angles\n"
								"%d \t %d \t %d\n\n"),
								_pitch, 
								_roll,
								_yaw);
		
		//_out_vector = _cam_vector * temp;
		//Serial.printf_P(PSTR(	"cam\n"
		//						"%d \t %d \t %d\n\n"),
		//						(int)temp.a.x * 100, (int)temp.a.y * 100, (int)temp.a.x * 100);

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
	static byte ts_num;
	float old_yaw;

	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Omega"));
	delay(1000);
	
	G_Dt = .02;

	while(1){
		delay(20);
		// IMU
		// ---
		read_AHRS();
		float my_oz = (dcm.yaw - old_yaw) * 50;
		
		old_yaw = dcm.yaw;
		
		ts_num++;
		if (ts_num > 2){
			ts_num = 0;
			//Serial.printf_P(PSTR("R: %4.4f\tP: %4.4f\tY: %4.4f\tY: %4.4f\n"), omega.x, omega.y, omega.z, my_oz);
			Serial.printf_P(PSTR(" Yaw: %ld\tY: %4.4f\tY: %4.4f\n"), dcm.yaw_sensor, omega.z, my_oz);
		}

		if(Serial.available() > 0){
			return (0);
		}
	}
	return (0);
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
test_current(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delta_ms_medium_loop = 100;
	
	while(1){
		delay(100);
		read_radio();
		read_current();
		Serial.printf_P(PSTR("V: %4.4f, A: %4.4f, mAh: %4.4f\n"), current_voltage, current_amps, current_total);

		//if(rc_3.control_in > 0){
			APM_RC.OutputCh(CH_1, rc_3.radio_in);
			APM_RC.OutputCh(CH_2, rc_3.radio_in);
			APM_RC.OutputCh(CH_3, rc_3.radio_in);
			APM_RC.OutputCh(CH_4, rc_3.radio_in);
		//}
		
		if(Serial.available() > 0){
			return (0);
		}
	}
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
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
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



void print_motor_out(){
	Serial.printf("out: R: %d,  L: %d  F: %d  B: %d\n", 
				(motor_out[RIGHT] 	- rc_3.radio_min),
				(motor_out[LEFT] 	- rc_3.radio_min),
				(motor_out[FRONT] 	- rc_3.radio_min),
				(motor_out[BACK] 	- rc_3.radio_min));
}