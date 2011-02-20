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
						 "  g_gps\n"
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

		Serial.printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"), g.rc_1.radio_in, g.rc_2.radio_in, g.rc_3.radio_in, g.rc_4.radio_in, g.rc_5.radio_in, g.rc_6.radio_in, g.rc_7.radio_in, g.rc_8.radio_in);

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

		g.rc_1.calc_pwm();
		g.rc_2.calc_pwm();
		g.rc_3.calc_pwm();
		g.rc_4.calc_pwm();

		Serial.printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"), (g.rc_1.control_in), (g.rc_2.control_in), (g.rc_3.control_in), (g.rc_4.control_in), g.rc_5.control_in, g.rc_6.control_in, g.rc_7.control_in);
		//Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (g.rc_1.servo_out / 100), (g.rc_2.servo_out / 100), g.rc_3.servo_out, (g.rc_4.servo_out / 100));

		/*Serial.printf_P(PSTR(	"min: %d"
								"\t in: %d"
								"\t pwm_in: %d"
								"\t sout: %d"
								"\t pwm_out %d\n"),
								g.rc_3.radio_min,
								g.rc_3.control_in,
								g.rc_3.radio_in,
								g.rc_3.servo_out,
								g.rc_3.pwm_out
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
	while(g.rc_3.control_in > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(g.rc_3.control_in > 0){
			Serial.printf_P(PSTR("THROTTLE CHANGED %d \n"), g.rc_3.control_in);
			fail_test++;
		}

		if(oldSwitchPosition != readSwitch()){
			Serial.printf_P(PSTR("CONTROL MODE CHANGED: "));
			Serial.println(flight_mode_strings[readSwitch()]);
			fail_test++;
		}

		if(g.throttle_fs_enabled && g.rc_3.get_failsafe()){
			Serial.printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), g.rc_3.radio_in);
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
	Serial.printf_P(PSTR("g.pid_stabilize_roll.kP: %4.4f\n"), g.pid_stabilize_roll.kP());
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

			if(g.compass_enabled){
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					compass.calculate(dcm.roll, dcm.pitch);		// Calculate heading
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
			if(g.rc_5.control_in < 600){
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
				Serial.printf_P(PSTR("r: %d, p:%d, rc1:%d, "),
					(int)(dcm.roll_sensor/100),
					(int)(dcm.pitch_sensor/100),
					g.rc_1.pwm_out);

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
	//Serial.printf_P(PSTR("g.pid_stabilize_roll.kP: %4.4f\n"), g.pid_stabilize_roll.kP());
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


			if(g.compass_enabled){
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					compass.calculate(dcm.roll, dcm.pitch);		// Calculate heading
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
			if(g.rc_5.control_in < 600){
				dcm.roll_sensor = dcm.pitch_sensor = 0;
			}

			// custom code/exceptions for flight modes
			// ---------------------------------------
			update_current_flight_mode();

			// write out the servo PWM values
			// ------------------------------
			set_servos_4();

			ts_num++;
			if (ts_num == 5){
				// 10 hz
				ts_num 				= 0;
				g_gps->longitude 		= 0;
				g_gps->latitude 		= 0;
				calc_nav();

				Serial.printf_P(PSTR("ys:%ld, WP.lat:%ld, WP.lng:%ld, n_lat:%ld, n_lon:%ld, nroll:%ld, npitch:%ld, pmax:%ld, \t- "),
					dcm.yaw_sensor,
					next_WP.lat,
					next_WP.lng,
					nav_lat,
					nav_lon,
					nav_roll,
					nav_pitch,
					(long)g.pitch_max);

				print_motor_out();
			}

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

	//float cos_roll, sin_roll, cos_pitch, sin_pitch, cos_yaw, sin_yaw;


	while(1){
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();
			/*
			Matrix3f temp 	= dcm.get_dcm_matrix();

			sin_pitch 		= -temp.c.x;
			cos_pitch 		= sqrt(1 - (temp.c.x * temp.c.x));

			cos_roll 		= temp.c.z / cos_pitch;
			sin_roll 		= temp.c.y / cos_pitch;

			yawvector.x 	= temp.a.x; // sin
			yawvector.y 	= temp.b.x;	// cos
			yawvector.normalize();
			cos_yaw 		= yawvector.y;	// 0 x = north
			sin_yaw 		= yawvector.x;	// 1 y
			*/

			// IMU
			// ---
			read_AHRS();

			Vector3f accels 	= imu.get_accel();
			Vector3f gyros 		= imu.get_gyro();

			update_trig();

			if(g.compass_enabled){
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					compass.calculate(dcm.roll, dcm.pitch);		// Calculate heading
					medium_loopCounter = 0;
				}
			}

			// We are using the IMU
			// ---------------------
			Serial.printf_P(PSTR("A: %4.4f, %4.4f, %4.4f\t"
								 "G: %4.4f, %4.4f, %4.4f\t"),
								accels.x, accels.y, accels.z,
								gyros.x,  gyros.y,  gyros.z);

			Serial.printf_P(PSTR("r: %ld\tp: %ld\t y: %ld\t"),
								dcm.roll_sensor,
								dcm.pitch_sensor,
								dcm.yaw_sensor);

			Serial.printf_P(PSTR("cp: %1.2f, sp: %1.2f, cr: %1.2f, sr: %1.2f, cy: %1.2f, sy: %1.2f,\n"),
								cos_pitch_x,
								sin_pitch_y,
								cos_roll_x,
								sin_roll_y,
								cos_yaw_x,	// x
								sin_yaw_y);	// y
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

	/*while(1){
		delay(100);

		update_GPS();

		if(Serial.available() > 0){
			return (0);
		}

		if(home.lng != 0){
			break;
		}
	}*/

	while(1){
		delay(100);
		update_GPS();

		calc_distance_error();

		//if (g_gps->new_data){
			Serial.printf_P(PSTR("Lat: %3.8f, Lon: %3.8f, alt %dm, spd: %d dist:%d, #sats: %d\n"),
						((float)g_gps->latitude /  10000000),
						((float)g_gps->longitude / 10000000),
						(int)g_gps->altitude / 100,
						(int)g_gps->ground_speed,
						(int)wp_distance,
						(int)g_gps->num_sats);
		//}else{
			//Serial.print(".");
		//}
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

		//if(g.rc_3.control_in > 0){
			APM_RC.OutputCh(CH_1, g.rc_3.radio_in);
			APM_RC.OutputCh(CH_2, g.rc_3.radio_in);
			APM_RC.OutputCh(CH_3, g.rc_3.radio_in);
			APM_RC.OutputCh(CH_4, g.rc_3.radio_in);
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

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);
	read_EEPROM_waypoint_info();


	// save the alitude above home option
	if(g.RTL_altitude == -1){
		Serial.printf_P(PSTR("Hold current altitude\n"));
	}else{
		Serial.printf_P(PSTR("Hold altitude of %dm\n"), (int)g.RTL_altitude);
	}

	Serial.printf_P(PSTR("%d waypoints\n"), (int)g.waypoint_total);
	Serial.printf_P(PSTR("Hit radius: %d\n"), (int)g.waypoint_radius);
	Serial.printf_P(PSTR("Loiter radius: %d\n\n"), (int)g.loiter_radius);

	for(byte i = 0; i <= g.waypoint_total; i++){
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
	ground_pressure = (float)sum / 100.0;
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
			next_WP.alt = home.alt + g.rc_6.control_in; // 0 - 2000 (20 meters)
			read_trim_switch();
			read_barometer();

			Serial.printf_P(PSTR("AP: %ld,\tAlt: %ld, \tnext_alt: %ld \terror: %ld, \tcruise: %d, \t out:%d\n"),
						abs_pressure,
						current_loc.alt,
						next_WP.alt,
						altitude_error,
						(int)g.throttle_cruise,
						g.rc_3.servo_out);

			/*
			Serial.print("Altitude: ");
			Serial.print((int)current_loc.alt,DEC);
			Serial.print("\tnext_alt: ");
			Serial.print((int)next_WP.alt,DEC);
			Serial.print("\talt_err: ");
			Serial.print((int)altitude_error,DEC);
			Serial.print("\ttNom: ");
			Serial.print(g.,DEC);
			Serial.print("\ttOut: ");
			Serial.println(g.rc_3.servo_out,DEC);
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
	if(g.compass_enabled == false){
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

void fake_out_gps()
{
	static float rads;
	g_gps->new_data 	= true;
	g_gps->fix	 	= true;

	int length = g.rc_6.control_in;
	rads += .05;

	if (rads > 6.28){
		rads = 0;
	}

	g_gps->latitude	= 377696000;	// Y
	g_gps->longitude	= -1224319000;	// X
	g_gps->altitude	= 9000;			// meters * 100

	//next_WP.lng	 	= home.lng - length * sin(rads);   // X
	//next_WP.lat 	= home.lat + length * cos(rads);   // Y
}



void print_motor_out(){
	Serial.printf("out: R: %d,  L: %d  F: %d  B: %d\n",
				(motor_out[RIGHT] 	- g.rc_3.radio_min),
				(motor_out[LEFT] 	- g.rc_3.radio_min),
				(motor_out[FRONT] 	- g.rc_3.radio_min),
				(motor_out[BACK] 	- g.rc_3.radio_min));
}
