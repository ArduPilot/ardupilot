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
static int8_t	test_ins(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
#if CONFIG_SONAR == ENABLED
static int8_t	test_sonar(uint8_t argc, 	const Menu::arg *argv);
#endif
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_modeswitch(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_logging(uint8_t argc, 		const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
	{"pwm",				test_radio_pwm},
	{"radio",			test_radio},
	{"passthru",		test_passthru},
	{"failsafe",		test_failsafe},
	{"battery",	test_battery},
	{"relay",			test_relay},
	{"waypoints",		test_wp},
	{"modeswitch",		test_modeswitch},

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
#if CONFIG_ADC == ENABLED
	{"adc", 		test_adc},
#endif
	{"gps",			test_gps},
	{"ins",			test_ins},
#if CONFIG_SONAR == ENABLED
	{"sonartest",	test_sonar},
#endif
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_SENSORS
	{"adc", 		test_adc},
	{"gps",			test_gps},
	{"ins",			test_ins},
	{"compass",		test_mag},
#elif HIL_MODE == HIL_MODE_ATTITUDE
#endif
	{"logging",		test_logging},

};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->printf_P(PSTR("Test Mode\n\n"));
	test_menu.run();
    return 0;
}

static void print_hit_enter()
{
	cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
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

		cliSerial->printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							g.channel_roll.radio_in,
							g.channel_pitch.radio_in,
							g.channel_throttle.radio_in,
							g.channel_rudder.radio_in,
							g.rc_5.radio_in,
							g.rc_6.radio_in,
							g.rc_7.radio_in,
							g.rc_8.radio_in);

		if(cliSerial->available() > 0){
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
        if (hal.rcin->valid() > 0) {
            cliSerial->print("CH:");
            for(int i = 0; i < 8; i++){
                cliSerial->print(hal.rcin->read(i));	// Print channel values
                cliSerial->print(",");
                hal.rcout->write(i, hal.rcin->read(i)); // Copy input to Servos
            }
            cliSerial->println();
        }
        if (cliSerial->available() > 0){
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

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	while(1){
		delay(20);
		read_radio();

		g.channel_roll.calc_pwm();
		g.channel_pitch.calc_pwm();
		g.channel_throttle.calc_pwm();
		g.channel_rudder.calc_pwm();

		// write out the servo PWM values
		// ------------------------------
		set_servos();

                tuning_value = constrain(((float)(g.rc_7.radio_in - g.rc_7.radio_min) / (float)(g.rc_7.radio_max - g.rc_7.radio_min)),0,1);
                
		cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d  Tuning = %2.3f\n"),
							g.channel_roll.control_in,
							g.channel_pitch.control_in,
							g.channel_throttle.control_in,
							g.channel_rudder.control_in,
							g.rc_5.control_in,
							g.rc_6.control_in,
							g.rc_7.control_in,
							g.rc_8.control_in,
                                                        tuning_value);

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_failsafe(uint8_t argc, const Menu::arg *argv)
{
	uint8_t fail_test;
	print_hit_enter();
	for(int i = 0; i < 50; i++){
		delay(20);
		read_radio();
	}

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	oldSwitchPosition = readSwitch();

	cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
	while(g.channel_throttle.control_in > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(g.channel_throttle.control_in > 0){
			cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), g.channel_throttle.control_in);
			fail_test++;
		}

		if (oldSwitchPosition != readSwitch()){
			cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
            print_flight_mode(readSwitch());
			fail_test++;
		}

		if(g.throttle_fs_enabled && g.channel_throttle.get_failsafe()){
			cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), g.channel_throttle.radio_in);
            print_flight_mode(readSwitch());
			fail_test++;
		}

		if(fail_test > 0){
			return (0);
		}
		if(cliSerial->available() > 0){
			cliSerial->printf_P(PSTR("LOS caused no change in APM.\n"));
			return (0);
		}
	}
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
if (g.battery_monitoring == 3 || g.battery_monitoring == 4) {
	print_hit_enter();
	delta_ms_medium_loop = 100;

	while(1){
		delay(100);
		read_radio();
		read_battery();
		if (g.battery_monitoring == 3){
			cliSerial->printf_P(PSTR("V: %4.4f\n"),
						battery_voltage1,
						current_amps1,
						current_total1);
		} else {
			cliSerial->printf_P(PSTR("V: %4.4f, A: %4.4f, mAh: %4.4f\n"),
						battery_voltage1,
						current_amps1,
						current_total1);
		}

		// write out the servo PWM values
		// ------------------------------
		set_servos();

		if(cliSerial->available() > 0){
			return (0);
		}
	}
} else {
	cliSerial->printf_P(PSTR("Not enabled\n"));
	return (0);
}

}

static int8_t
test_relay(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		cliSerial->printf_P(PSTR("Relay on\n"));
		relay.on();
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}

		cliSerial->printf_P(PSTR("Relay off\n"));
		relay.off();
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);

	cliSerial->printf_P(PSTR("%d waypoints\n"), (int)g.command_total);
	cliSerial->printf_P(PSTR("Hit radius: %d\n"), (int)g.waypoint_radius);

	for(uint8_t i = 0; i <= g.command_total; i++){
		struct Location temp = get_cmd_with_index(i);
		test_wp_print(&temp, i);
	}

	return (0);
}

static void
test_wp_print(struct Location *cmd, uint8_t wp_index)
{
	cliSerial->printf_P(PSTR("command #: %d id:%d options:%d p1:%d p2:%ld p3:%ld p4:%ld \n"),
		(int)wp_index,
		(int)cmd->id,
		(int)cmd->options,
		(int)cmd->p1,
		cmd->alt,
		cmd->lat,
		cmd->lng);
}

static int8_t
test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	cliSerial->printf_P(PSTR("Control CH "));

	cliSerial->println(FLIGHT_MODE_CHANNEL, DEC);

	while(1){
		delay(20);
		uint8_t switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			cliSerial->printf_P(PSTR("Position %d\n"),  switchPosition);
			oldSwitchPosition = switchPosition;
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

/*
  test the dataflash is working
 */
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->println_P(PSTR("Testing dataflash logging"));
    if (!DataFlash.CardInserted()) {
        cliSerial->println_P(PSTR("ERR: No dataflash inserted"));
        return 0;
    }
    DataFlash.ReadManufacturerID();
    cliSerial->printf_P(PSTR("Manufacturer: 0x%02x   Device: 0x%04x\n"),
                    (unsigned)DataFlash.df_manufacturer,
                    (unsigned)DataFlash.df_device);
    cliSerial->printf_P(PSTR("NumPages: %u  PageSize: %u\n"),
                    (unsigned)DataFlash.df_NumPages+1,
                    (unsigned)DataFlash.df_PageSize);
    DataFlash.StartRead(DataFlash.df_NumPages+1);
    cliSerial->printf_P(PSTR("Format version: %lx  Expected format version: %lx\n"),
                    (unsigned long)DataFlash.ReadLong(), (unsigned long)DF_LOGGING_FORMAT);
    return 0;
}


//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

#if HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

#if CONFIG_ADC == ENABLED
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	adc.Init();
	delay(1000);
	cliSerial->printf_P(PSTR("ADC\n"));
	delay(1000);

	while(1){
		for (int i=0;i<9;i++) cliSerial->printf_P(PSTR("%.1f\t"),adc.Ch(i));
		cliSerial->println();
		delay(100);
		if(cliSerial->available() > 0){
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
			cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
					g_gps->latitude,
					g_gps->longitude,
					g_gps->altitude/100,
					g_gps->num_sats);
		}else{
			cliSerial->printf_P(PSTR("."));
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
	//cliSerial->printf_P(PSTR("Calibrating."));
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate, 
             flash_leds);
    ahrs.reset();

	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// INS
			// ---
			ahrs.update();

			if(g.compass_enabled) {
				medium_loopCounter++;
				if(medium_loopCounter == 5){
					compass.read();
                    medium_loopCounter = 0;
				}
			}

			// We are using the IMU
			// ---------------------
            Vector3f gyros 	= ins.get_gyro();
            Vector3f accels = ins.get_accel();
			cliSerial->printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            gyros.x, gyros.y, gyros.z,
                            accels.x, accels.y, accels.z);
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
	if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
		print_enabled(false);
		return (0);
    }

    compass.set_orientation(MAG_ORIENTATION);
    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate, 
             flash_leds);
    ahrs.reset();

	int counter = 0;
    float heading = 0;

		//cliSerial->printf_P(PSTR("MAG_ORIENTATION: %d\n"), MAG_ORIENTATION);

    print_hit_enter();

    while(1) {
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// IMU
			// ---
			ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5){
                if (compass.read()) {
                    // Calculate heading
                    Matrix3f m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
                medium_loopCounter = 0;
            }

			counter++;
			if (counter>20) {
                if (compass.healthy) {
                    Vector3f maggy = compass.get_offsets();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                    (wrap_360(ToDeg(heading) * 100)) /100,
                                    (int)compass.mag_x,
                                    (int)compass.mag_y,
                                    (int)compass.mag_z,
                                    maggy.x,
                                    maggy.y,
                                    maggy.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
		}
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.    
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

#endif // HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

#if CONFIG_SONAR == ENABLED
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
  print_hit_enter();
	delay(1000);
	init_sonar();
	delay(1000);

	while(1){
	  delay(20);
	  if(g.sonar_enabled){
		sonar_dist = sonar->read();
	  }
    	  cliSerial->printf_P(PSTR("sonar_dist = %d\n"), (int)sonar_dist);

          if(cliSerial->available() > 0){
  		break;
	    }
  }
  return (0);
}
#endif // SONAR == ENABLED

#endif // CLI_ENABLED
