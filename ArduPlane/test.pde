// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio_pwm(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_passthru(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_failsafe(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
#if CONFIG_ADC == ENABLED
static int8_t	test_adc(uint8_t argc, 			const Menu::arg *argv);
#endif
static int8_t	test_imu(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_current(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_airspeed(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_pressure(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_xbee(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_eedump(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_rawgps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_modeswitch(uint8_t argc, 		const Menu::arg *argv);
#if CONFIG_APM_HARDWARE != APM_HARDWARE_APM2
static int8_t	test_dipswitches(uint8_t argc, 		const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
	{"pwm",			test_radio_pwm},
	{"radio",		test_radio},
	{"passthru",	test_passthru},
	{"failsafe",	test_failsafe},
	{"battery",		test_battery},
	{"relay",		test_relay},
	{"waypoints",	test_wp},
	{"xbee",		test_xbee},
	{"eedump",		test_eedump},
	{"modeswitch",	test_modeswitch},
#if CONFIG_APM_HARDWARE != APM_HARDWARE_APM2
	{"dipswitches",	test_dipswitches},
#endif

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
#if CONFIG_ADC == ENABLED
	{"adc", 		test_adc},
#endif
	{"gps",			test_gps},
	{"rawgps",		test_rawgps},
	{"imu",			test_imu},
	{"airspeed",	test_airspeed},
	{"airpressure",	test_pressure},
	{"compass",		test_mag},
	{"current",		test_current},
#elif HIL_MODE == HIL_MODE_SENSORS
	{"adc", 		test_adc},
	{"gps",			test_gps},
	{"imu",			test_imu},
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_ATTITUDE
#endif

};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Test Mode\n\n"));
	test_menu.run();
    return 0;
}

static void print_hit_enter()
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
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		Serial.printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							g.channel_roll.radio_in,
							g.channel_pitch.radio_in,
							g.channel_throttle.radio_in,
							g.channel_rudder.radio_in,
							g.rc_5.radio_in,
							g.rc_6.radio_in,
							g.rc_7.radio_in,
							g.rc_8.radio_in);

		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_passthru(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (APM_RC.GetState() == 1){
            Serial.print("CH:");
            for(int i = 0; i < 8; i++){
                Serial.print(APM_RC.InputCh(i));	// Print channel values
                Serial.print(",");
                APM_RC.OutputCh(i, APM_RC.InputCh(i)); // Copy input to Servos
            }
            Serial.println();
        }
        if (Serial.available() > 0){
            return (0);
        }
    }
    return 0;
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	#if THROTTLE_REVERSE == 1
		Serial.printf_P(PSTR("Throttle is reversed in config: \n"));
		delay(1000);
	#endif

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	while(1){
		delay(20);
		read_radio();
		update_servo_switches();

		g.channel_roll.calc_pwm();
		g.channel_pitch.calc_pwm();
		g.channel_throttle.calc_pwm();
		g.channel_rudder.calc_pwm();

		// write out the servo PWM values
		// ------------------------------
		set_servos();

		Serial.printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							g.channel_roll.control_in,
							g.channel_pitch.control_in,
							g.channel_throttle.control_in,
							g.channel_rudder.control_in,
							g.rc_5.control_in,
							g.rc_6.control_in,
							g.rc_7.control_in,
							g.rc_8.control_in);

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
	while(g.channel_throttle.control_in > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(g.channel_throttle.control_in > 0){
			Serial.printf_P(PSTR("THROTTLE CHANGED %d \n"), g.channel_throttle.control_in);
			fail_test++;
		}

		if(oldSwitchPosition != readSwitch()){
			Serial.printf_P(PSTR("CONTROL MODE CHANGED: "));
			Serial.println(flight_mode_strings[readSwitch()]);
			fail_test++;
		}

		if(g.throttle_fs_enabled && g.channel_throttle.get_failsafe()){
			Serial.printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), g.channel_throttle.radio_in);
			Serial.println(flight_mode_strings[readSwitch()]);
			fail_test++;
		}

		if(fail_test > 0){
			return (0);
		}
		if(Serial.available() > 0){
			Serial.printf_P(PSTR("LOS caused no change in APM.\n"));
			return (0);
		}
	}
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
if (g.battery_monitoring >=1 && g.battery_monitoring < 4) {
	for (int i = 0; i < 80; i++){	//  Need to get many samples for filter to stabilize
		delay(20);
		read_battery();
	}
	Serial.printf_P(PSTR("Volts: 1:%2.2f, 2:%2.2f, 3:%2.2f, 4:%2.2f\n"),
			battery_voltage1,
			battery_voltage2,
			battery_voltage3,
			battery_voltage4);
} else {
	Serial.printf_P(PSTR("Not enabled\n"));
}
	return (0);
}

static int8_t
test_current(uint8_t argc, const Menu::arg *argv)
{
if (g.battery_monitoring == 4) {
	print_hit_enter();
	delta_ms_medium_loop = 100;

	while(1){
		delay(100);
		read_radio();
		read_battery();
		Serial.printf_P(PSTR("V: %4.4f, A: %4.4f, mAh: %4.4f\n"),
						battery_voltage,
						current_amps,
						current_total);

		// write out the servo PWM values
		// ------------------------------
		set_servos();

		if(Serial.available() > 0){
			return (0);
		}
	}
} else {
	Serial.printf_P(PSTR("Not enabled\n"));
	return (0);
}

}

static int8_t
test_relay(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		Serial.printf_P(PSTR("Relay on\n"));
		relay.on();
		delay(3000);
		if(Serial.available() > 0){
			return (0);
		}

		Serial.printf_P(PSTR("Relay off\n"));
		relay.off();
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

	// save the alitude above home option
	if(g.RTL_altitude < 0){
		Serial.printf_P(PSTR("Hold current altitude\n"));
	}else{
		Serial.printf_P(PSTR("Hold altitude of %dm\n"), (int)g.RTL_altitude/100);
	}

	Serial.printf_P(PSTR("%d waypoints\n"), (int)g.command_total);
	Serial.printf_P(PSTR("Hit radius: %d\n"), (int)g.waypoint_radius);
	Serial.printf_P(PSTR("Loiter radius: %d\n\n"), (int)g.loiter_radius);

	for(byte i = 0; i <= g.command_total; i++){
		struct Location temp = get_cmd_with_index(i);
		test_wp_print(&temp, i);
	}

	return (0);
}

static void
test_wp_print(struct Location *cmd, byte wp_index)
{
	Serial.printf_P(PSTR("command #: %d id:%d options:%d p1:%d p2:%ld p3:%ld p4:%ld \n"),
		(int)wp_index,
		(int)cmd->id,
		(int)cmd->options,
		(int)cmd->p1,
		cmd->alt,
		cmd->lat,
		cmd->lng);
}

static int8_t
test_xbee(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);
	Serial.printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));

	while(1){

	    if (Serial3.available())
   			Serial3.write(Serial3.read());

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

	Serial.printf_P(PSTR("Control CH "));

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

#if CONFIG_APM_HARDWARE != APM_HARDWARE_APM2
static int8_t
test_dipswitches(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

    if (!g.switch_enable) {
        Serial.println_P(PSTR("dip switches disabled, using EEPROM"));
    }

	while(1){
		delay(100);
		update_servo_switches();

		if (g.mix_mode == 0) {
			Serial.printf_P(PSTR("Mix:standard \trev roll:%d, rev pitch:%d, rev rudder:%d\n"),
				(int)g.channel_roll.get_reverse(),
				(int)g.channel_pitch.get_reverse(),
				(int)g.channel_rudder.get_reverse());
		} else {
			Serial.printf_P(PSTR("Mix:elevons \trev elev:%d, rev ch1:%d, rev ch2:%d\n"),
				(int)g.reverse_elevons,
				(int)g.reverse_ch1_elevon,
				(int)g.reverse_ch2_elevon);
		}
		if(Serial.available() > 0){
			return (0);
		}
	}
}
#endif // CONFIG_APM_HARDWARE != APM_HARDWARE_APM2


//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

#if HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

#if CONFIG_ADC == ENABLED
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	adc.Init(&timer_scheduler);
	delay(1000);
	Serial.printf_P(PSTR("ADC\n"));
	delay(1000);

	while(1){
		for (int i=0;i<9;i++) Serial.printf_P(PSTR("%.1f\t"),adc.Ch(i));
		Serial.println();
		delay(100);
		if(Serial.available() > 0){
			return (0);
		}
	}
}
#endif // CONFIG_ADC

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

		g_gps->update();

		if (g_gps->new_data){
			Serial.printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
					g_gps->latitude,
					g_gps->longitude,
					g_gps->altitude/100,
					g_gps->num_sats);
		}else{
			Serial.printf_P(PSTR("."));
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
	imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
    dcm.matrix_reset();

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
			dcm.update_DCM();

			if(g.compass_enabled) {
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();		 				// Read magnetometer
					compass.calculate(dcm.get_dcm_matrix());		// Calculate heading
					medium_loopCounter = 0;
				}
			}

			// We are using the IMU
			// ---------------------
            Vector3f gyros 	= imu.get_gyro();
            Vector3f accels = imu.get_accel();
			Serial.printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
                            (int)dcm.roll_sensor / 100,
                            (int)dcm.pitch_sensor / 100,
                            (uint16_t)dcm.yaw_sensor / 100,
                            gyros.x, gyros.y, gyros.z,
                            accels.x, accels.y, accels.z);
		}
		if(Serial.available() > 0){
			return (0);
		}
	}
}


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
	if (!g.compass_enabled) {
        Serial.printf_P(PSTR("Compass: "));
		print_enabled(false);
		return (0);
    }

    compass.set_orientation(MAG_ORIENTATION);
    if (!compass.init()) {
        Serial.println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }
    dcm.set_compass(&compass);
    report_compass();

    // we need the DCM initialised for this test
	imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
    dcm.matrix_reset();

	int counter = 0;
		//Serial.printf_P(PSTR("MAG_ORIENTATION: %d\n"), MAG_ORIENTATION);

    print_hit_enter();

    while(1) {
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// IMU
			// ---
			dcm.update_DCM();

            medium_loopCounter++;
            if(medium_loopCounter == 5){
                compass.read();		 				// Read magnetometer
                compass.calculate(dcm.get_dcm_matrix());		// Calculate heading
                compass.null_offsets(dcm.get_dcm_matrix());
                medium_loopCounter = 0;
            }

			counter++;
			if (counter>20) {
                Vector3f maggy = compass.get_offsets();
                Serial.printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                (wrap_360(ToDeg(compass.heading) * 100)) /100,
                                compass.mag_x,
                                compass.mag_y,
                                compass.mag_z,
                                maggy.x,
                                maggy.y,
                                maggy.z);
                counter=0;
            }
		}
        if (Serial.available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.    
    Serial.println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

#endif // HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

#if HIL_MODE == HIL_MODE_DISABLED

static int8_t
test_airspeed(uint8_t argc, const Menu::arg *argv)
{
    float airspeed_ch = pitot_analog_source.read();
	// Serial.println(pitot_analog_source.read());
    Serial.printf_P(PSTR("airspeed_ch: %.1f\n"), airspeed_ch);

	if (g.airspeed_enabled == false){
		Serial.printf_P(PSTR("airspeed: "));
		print_enabled(false);
		return (0);

	}else{
		print_hit_enter();
		zero_airspeed();
		Serial.printf_P(PSTR("airspeed: "));
		print_enabled(true);

		while(1){
			delay(20);
			read_airspeed();
			Serial.printf_P(PSTR("%.1f m/s\n"), airspeed / 100.0);

			if(Serial.available() > 0){
				return (0);
			}
		}
	}
}


static int8_t
test_pressure(uint8_t argc, const Menu::arg *argv)
{
	Serial.printf_P(PSTR("Uncalibrated relative airpressure\n"));
	print_hit_enter();

	home.alt 	= 0;
	wp_distance = 0;
	init_barometer();

	while(1){
		delay(100);
		current_loc.alt = read_barometer() + home.alt;

		Serial.printf_P(PSTR("Alt: %0.2fm, Raw: %ld Temperature: %.1f\n"),
						current_loc.alt / 100.0,
						abs_pressure, 0.1*barometer.get_temperature());

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
		if (Serial3.available()){
			digitalWrite(B_LED_PIN, LED_ON); 		// Blink Yellow LED if we are sending data to GPS
			Serial1.write(Serial3.read());
			digitalWrite(B_LED_PIN, LED_OFF);
		}
		if (Serial1.available()){
			digitalWrite(C_LED_PIN, LED_ON);		// Blink Red LED if we are receiving data from GPS
			Serial3.write(Serial1.read());
			digitalWrite(C_LED_PIN, LED_OFF);
		}
		if(Serial.available() > 0){
			return (0);
		}
  }
}
#endif // HIL_MODE == HIL_MODE_DISABLED

#endif // CLI_ENABLED
